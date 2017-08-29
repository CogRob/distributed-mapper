/**
 * @file testRobustMapping.cpp
 * @author Siddharth Choudhary
 * @brief unit tests for testing robust mapping
 */

#include <DistributedMapper.h>
#include <BetweenChordalFactor.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace distributed_mapper;

// Prior
noiseModel::Diagonal::shared_ptr priorModel = //
        noiseModel::Isotropic::Variance(6, 1e-12);

std::string robotNames_("abcdefghijk");

#define USE_BETWEEN_CHORDAL_FACTOR 1


// MACRO for comparing robust rotation estimates
#define COMPARE_ESTIMATES(actual, expected, tolerance) \
{ \
    for(const Values::ConstKeyValuePair& key_value: actual){\
    Key key = key_value.key;\
    EXPECT(assert_equal(actual.at<Pose3>(key), expected.at<Pose3>(key), tolerance));\
    } \
    } \


GraphAndValues readFullGraph(size_t nrRobots, // number of robots
                             vector<GraphAndValues> graphAndValuesVec  // vector of all graphs and initials for each robot
                             ){
    std::cout << "Creating fullGraph by combining subgraphs." << std::endl;

    // Combined graph and Values
    NonlinearFactorGraph::shared_ptr combinedGraph(new NonlinearFactorGraph);
    Values::shared_ptr combinedValues(new Values);

    // Iterate over each robot
    for(size_t robot = 0; robot < nrRobots; robot++){

        // Load graph and initial
        NonlinearFactorGraph graph = *(graphAndValuesVec[robot].first);
        Values initial = *(graphAndValuesVec[robot].second);

        // Iterate over initial and push it to the combinedValues, each initial value is present only once
        for(const Values::ConstKeyValuePair& key_value: initial){
            Key key = key_value.key;
            if(!combinedValues->exists(key))
                combinedValues->insert(key, initial.at<Pose3>(key));
        }

        // Iterate over the graph and push the factor if it is not already present in combinedGraph
        for(size_t ksub=0; ksub<graph.size(); ksub++){ //for each factor in the new subgraph
            bool isPresent = false;
            for(size_t k=0; k<combinedGraph->size(); k++){

                boost::shared_ptr<BetweenFactor<Pose3> > factorSub =
                        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph.at(ksub));
                Key factorSubKey1 = factorSub->key1();
                Key factorSubKey2 = factorSub->key2();

                boost::shared_ptr<BetweenFactor<Pose3> > factorCombined =
                        boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(combinedGraph->at(k));
                Key factorCombinedKey1 = factorCombined->key1();
                Key factorCombinedKey2 = factorCombined->key2();

                // values don't match exactly that's why check with keys as well
                if(factorCombined->equals(*factorSub) || ((factorSubKey1 == factorCombinedKey1) && (factorSubKey2 == factorCombinedKey2))){
                    isPresent = true;
                    break;
                }
            }
            if(isPresent == false) // we insert the factor
                combinedGraph->add(graph.at(ksub));
        }
    }

    // Return graph and values
    return make_pair(combinedGraph, combinedValues);
}

/* ************************************************************************* */
GaussianFactorGraph buildWeightedLinearOrientationGraph(const NonlinearFactorGraph& g, std::vector<Vector> weights) {

    GaussianFactorGraph linearGraph;
    int count = 0;

    std::cout << "-----------------------------------" << endl;
    for(const boost::shared_ptr<NonlinearFactor>& factor: g) {
        Matrix3 Rij;

        // weighted noise model -- weights define the precision
        SharedDiagonal model = noiseModel::Diagonal::Precisions(weights.at(count));
\

        boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
                boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
        if (pose3Between)
            Rij = pose3Between->measured().rotation().matrix();
        else
            std::cout << "Error in buildLinearOrientationGraph" << std::endl;

        const FastVector<Key>& keys = factor->keys();
        Key key1 = keys[0], key2 = keys[1];

        // check if the factor is connected to keyAnchor -- do not use weights in that case
        if(symbolChr(keyAnchor) == symbolChr(key1) && symbolIndex(keyAnchor) == symbolIndex(key1)){
            model = noiseModel::Unit::Create(9);  // unit covariance on keyAnchor factor
        }

        // All the intra-robot edges have the same weight and are the trusted measurements
        if(symbolChr(key1) == symbolChr(key2)){
            model = noiseModel::Diagonal::Precisions(repeat(9, 1e4));
        }
        else{
            std::cout << symbolChr(key1) << symbolIndex(key1) << "--" << symbolChr(key2) << symbolIndex(key2) << ": " << weights.at(count).norm() << endl;
        }

        Matrix M9 = Z_9x9;
        M9.block(0,0,3,3) = Rij;
        M9.block(3,3,3,3) = Rij;
        M9.block(6,6,3,3) = Rij;
        linearGraph.add(key1, -I9, key2, M9, zero9, model);

        // increase count
        count++;
    }

    std::cout << "-----------------------------------" << endl;

    // prior on the anchor orientation
    noiseModel::Unit::shared_ptr priorModel = noiseModel::Unit::Create(9);  // unit covariance on prior
    linearGraph.add(keyAnchor, I9, (Vector(9) << 1.0, 0.0, 0.0,/*  */ 0.0, 1.0, 0.0, /*  */ 0.0, 0.0, 1.0).finished(), priorModel);
    return linearGraph;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Compute centralized estimate for rotations
////////////////////////////////////////////////////////////////////////////////////////////
Values centralizedRotationEstimation(NonlinearFactorGraph graph){

    // STEP 1: solve for rotations first, using chordal relaxation (centralized)
    // This is the "expected" solution
    // pose3Graph only contains between factors, as the priors are converted to between factors on an anchor key
    NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(graph);
    // this will also put a prior on the anchor
    GaussianFactorGraph centralizedLinearGraph = InitializePose3::buildLinearOrientationGraph(pose3Graph);
    VectorValues rotEstCentralized = centralizedLinearGraph.optimize();
    Values cenRot = InitializePose3::normalizeRelaxedRotations(rotEstCentralized);
    Values cenPoses = multirobot_util::pose3WithZeroTranslation(cenRot);
    return cenPoses;
}


////////////////////////////////////////////////////////////////////////////////////////////
// Compute centralized robust estimate for rotations
////////////////////////////////////////////////////////////////////////////////////////////
Values centralizedRobustRotationEstimation(NonlinearFactorGraph graph){

    // recast prior factor to between factor
    NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(graph);

    // initialize weights to ones
    std::vector<Vector> weights;
    gtsam::Vector9 unitVec = (gtsam::Vector(9) << 1, 1, 1, 1, 1, 1, 1, 1, 1).finished();
    for(size_t i = 0; i < pose3Graph.size(); i++){
        weights.push_back(unitVec);
    }

    Values prevRot;
    int maxIter = 10;
    double eps_w = 1e-10;
    Values cenRot;

    // Run IRLS iterations
    for(int iter = 0; iter < maxIter; iter++)
    {

        // build weighted linear orientation graph
        GaussianFactorGraph centralizedLinearGraph = buildWeightedLinearOrientationGraph(pose3Graph, weights);

        // solve it
        VectorValues rotEstCentralized = centralizedLinearGraph.optimize();
        cenRot = InitializePose3::normalizeRelaxedRotations(rotEstCentralized); // removes the values corresponding to keyAnchor before projecting the rotation to the manifold

        // convert Rot3 Values to VectorValues
        VectorValues vectorRot;
        for(const VectorValues::KeyValuePair& key_value: rotEstCentralized){
            Key key = key_value.first;
            if(cenRot.exists(key)){
                Rot3 R = cenRot.at<Rot3>(key);
                Matrix3 R_matrix = R.matrix();
                Vector9 R_vector = (gtsam::Vector(9) << R_matrix(0,0), R_matrix(0,1), R_matrix(0,2),/*  */ R_matrix(1,0), R_matrix(1,1), R_matrix(1,2), /*  */ R_matrix(2,0), R_matrix(2,1), R_matrix(2,2)).finished(); // stack by columns
                vectorRot.insert(key, R_vector);
            }
            else{
                // key belonging to keyAnchor // Add keyanchor vector value
                vectorRot.insert(keyAnchor, (Vector(9) << 1.0, 0.0, 0.0,/*  */ 0.0, 1.0, 0.0, /*  */ 0.0, 0.0, 1.0).finished());
            }
        }

        //centralizedLinearGraph.print("Linear Graph:");

        // residual
        Errors errorVector =  centralizedLinearGraph.gaussianErrors(vectorRot); // error is a fast list of vectors of size 9

        // update weights
        weights.clear();
        for(Errors::iterator it = errorVector.begin(); it!= errorVector.end(); it++){
            Vector9 error_i = *it;
            Vector9 weight_i;
            for(size_t j=0; j < 9; j++){
                weight_i[j] = pow((error_i[j]* error_i[j]),(-1.0f/2.0f));
                //std::cout << weight_i[j] << std::endl;
            }


            weights.push_back(weight_i);
            //std::cout << "Error: " << error_i.norm() << " (" << weight_i.norm() << ")" << std::endl;
        }

        // Compute change in estimate
        double latestChange = 0;
        if(iter!= 0){
            latestChange = cenRot.localCoordinates(prevRot).norm();
            std::cout << "Change: " << latestChange << std::endl;
        }

        prevRot = cenRot;
    }

    Values cenPoses = multirobot_util::pose3WithZeroTranslation(cenRot);
    return cenPoses;
}


/******************************************************************************/
TEST(DistributedMapper, testDistributedMapperBetweenAndPriorChordal) {
    // This compares:
    // 1) Centralized rotation estimate without outlier
    // 2) Centralized roubst rotation estimate with outliers

    int nrRobots = 2;
    std::string dataDir("../../../data/example_2robots/");
    vector<GraphAndValues> graphAndValuesVec; // vector of all graphs and initials

    // Read graphs
    for(size_t robot_i = 0; robot_i < nrRobots; robot_i++){
        string dataFile_i = dataDir + boost::lexical_cast<string>(robot_i) + ".g2o";
        GraphAndValues graphAndValuesG2o = readG2o(dataFile_i, true);
        graphAndValuesVec.push_back(graphAndValuesG2o);
    }

    // Aggregate
    GraphAndValues fullGraphAndValues = readFullGraph(nrRobots, graphAndValuesVec);

    NonlinearFactorGraph graph = *(fullGraphAndValues.first);
    Values initial = *(fullGraphAndValues.second);

    // Add prior
    graph.add(PriorFactor<Pose3>(gtsam::Symbol('a', 0), Pose3(), priorModel));
    graph.print("Graph:\n");

    // Rotation estimation without outliers
    Values centralizedRotation = centralizedRotationEstimation(graph);

    // Add outliers
    noiseModel::Diagonal::shared_ptr betweenModel = noiseModel::Unit::Create(6);
    graph.add(BetweenFactor<Pose3>(gtsam::Symbol('a', 0), gtsam::Symbol('b', 10), Pose3(Rot3::Ypr(1,1,1), Point3()), betweenModel));
    graph.add(BetweenFactor<Pose3>(gtsam::Symbol('a', 1), gtsam::Symbol('b', 11), Pose3(Rot3::Ypr(1,1,1), Point3()), betweenModel));
    //graph.add(BetweenFactor<Pose3>(gtsam::Symbol('a', 2), gtsam::Symbol('b', 12), Pose3(), betweenModel));
    //graph.add(BetweenFactor<Pose3>(gtsam::Symbol('a', 3), gtsam::Symbol('b', 13), Pose3(), betweenModel));


    // and perform robust rotation estimation
    Values centralizedRobustRotation = centralizedRobustRotationEstimation(graph);

    // Compare
    COMPARE_ESTIMATES(centralizedRotation, centralizedRobustRotation, 1e-1);
}

/****************************************************************************** */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
//******************************************************************************

