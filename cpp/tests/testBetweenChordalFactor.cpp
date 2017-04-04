/**
 * @file testBetweenChordalFactor.cpp
 * @author Siddharth Choudhary
 * @brief unit tests for between chordal factor
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

noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Variance(12, 1);
std::string robotNames_("abcdefghijk");

#define USE_BETWEEN_CHORDAL_FACTOR 1


// MACRO for comparing between chordal factor to linear chordal factor
#define COMPARE_ESTIMATES(linear, between, tolerance) \
{ \
  for(const Values::ConstKeyValuePair& key_value: linear){\
  Key key = key_value.key;\
  EXPECT(assert_equal(linear.at<Pose3>(key), between.at<Pose3>(key), tolerance));\
  } \
  } \

////////////////////////////////////////////////////////////////////////////////////////////
// UTILS
////////////////////////////////////////////////////////////////////////////////////////////
// Used only for testing:  Computes error and Jacobians of chordal between factors, but using standard betweenFactors as input
typedef boost::tuple<Key, Matrix, Key, Matrix, Vector> KiMiKjMje;
KiMiKjMje computeJacobianAndError(boost::shared_ptr<BetweenFactor<Pose3> > pose3Between, Values& pose_Values){
  Matrix3 S1 = skewSymmetric(-1,0,0); //(0 0 0, 0 0 1, 0 -1 0);
  Matrix3 S2 = skewSymmetric(0,-1,0); //sparse([0 0 -1; 0 0 0; 1  0 0]);
  Matrix3 S3 = skewSymmetric(0,0,-1); //sparse([0 1  0;-1 0 0; 0  0 0]);
  Key key_i = pose3Between->keys().at(0);
  Key key_j = pose3Between->keys().at(1);

  Pose3 relativePose = pose3Between->measured();
  Matrix3 Rij = relativePose.rotation().matrix();
  Matrix3 Rijt = Rij.transpose();
  Vector3 tij = relativePose.translation().vector();

  Pose3 Pi = pose_Values.at<Pose3>(key_i);
  Matrix3 Ri = Pi.rotation().matrix();
  Vector ti = Pi.translation().vector();

  Pose3 Pj = pose_Values.at<Pose3>(key_j);
  Matrix3 Rj = Pj.rotation().matrix();
  Vector tj = Pj.translation().vector();

  Matrix3 Ri_Rij = Ri*Rij;
  Matrix3 error_R = Ri_Rij - Rj;
  Vector error_r = multirobot_util::rowMajorVector(error_R.transpose()); // stack by columns
  Vector3 error_t = tj - ti - Ri * tij;

  // fill in residual error vector
  Vector error = (Vector(12) << error_r, error_t ).finished();

  // fill in Jacobian wrt pose of key_i
  Matrix Mi = Matrix::Zero(12,6);
  Mi.block(0,0,3,3) = Ri_Rij*S1*Rijt;
  Mi.block(3,0,3,3) = Ri_Rij*S2*Rijt;
  Mi.block(6,0,3,3) = Ri_Rij*S3*Rijt;

  Mi.block(9,0,3,3) = Ri*skewSymmetric(tij);
  Mi.block(9,3,3,3) = -eye(3); // TODO: define once outside

  // fill in Jacobian wrt pose of key_j
  Matrix Mj = Matrix::Zero(12,6);
  Mj.block(0,0,3,3) = - Rj*S1;
  Mj.block(3,0,3,3) = - Rj*S2;
  Mj.block(6,0,3,3) = - Rj*S3;

  Mj.block(9,3,3,3) = eye(3);  // TODO: define once outside

  return KiMiKjMje(key_i, Mi, key_j, Mj, error);
}

// Used only for testing: Linearized between chordal factor - replaced by betweenChordalFactor->linearize
JacobianFactor linearChordalFactor(const boost::shared_ptr<NonlinearFactor>& factor, Values& pose_Values){

  boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
      boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);

  if(pose3Between){ // it is a between factor on pose 3
      KiMiKjMje jacobianData = computeJacobianAndError(pose3Between, pose_Values);
      Key key_i = boost::get<0>(jacobianData);
      Matrix Mi = boost::get<1>(jacobianData);

      Key key_j = boost::get<2>(jacobianData);
      Matrix Mj = boost::get<3>(jacobianData);

      Vector error = boost::get<4>(jacobianData);

      return JacobianFactor(key_i, Mi, key_j, Mj, -error, model); // TODO: define the noise model just once
    }
  else{
      boost::shared_ptr<PriorFactor<Pose3> > pose3Prior =
          boost::dynamic_pointer_cast<PriorFactor<Pose3> >(factor);
      if(pose3Prior){
          Key key_i = pose3Prior->keys().at(0);
          return JacobianFactor(key_i, eye(6), zero(6), gtsam::noiseModel::Isotropic::Variance(6, 1e-12)); // TODO: define the noise model just once
        }
      else{
          cout << "Invalid Factor" << endl; exit(1);
        }
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
// Load graph
////////////////////////////////////////////////////////////////////////////////////////////
pair<NonlinearFactorGraph, Values>
loadGraphWithPrior(string dataFile){

  GraphAndValues graphAndValues = readG2o(dataFile, true);
  NonlinearFactorGraph graphCentralized = *(graphAndValues.first);
  Values initialCentralized = *(graphAndValues.second);
  // Add prior
  NonlinearFactorGraph graphWithPrior = graphCentralized;
  graphWithPrior.add(PriorFactor<Pose3>(0, Pose3(), priorModel));
  return make_pair(graphWithPrior, initialCentralized);
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
// Optimize using linear chordal factor
////////////////////////////////////////////////////////////////////////////////////////////
Values optimizeUsinglinearChordalFactor(NonlinearFactorGraph graph, Values centralizedRotation){

  GaussianFactorGraph cenGFG;
  for(size_t k = 0; k < graph.size(); k++){
      cenGFG.add( linearChordalFactor( graph[k] , centralizedRotation)  );
    }

  VectorValues cenPose_VectorValues = cenGFG.optimize();
  Values cenPoses = multirobot_util::retractPose3Global(centralizedRotation, cenPose_VectorValues);
  return cenPoses;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Iterate over input graph and create  a new graph using between chordal factors
////////////////////////////////////////////////////////////////////////////////////////////
NonlinearFactorGraph createBetweenChordalFactorGraph(NonlinearFactorGraph graph){
  NonlinearFactorGraph betweenChordalFactorGraph;
  for(size_t k = 0; k < graph.size(); k++){
      boost::shared_ptr<BetweenFactor<Pose3> > factor =
          boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph[k]);
      if(factor){
          Key key1 = factor->keys().at(0);
          Key key2 = factor->keys().at(1);
          Pose3 measured = factor->measured();
          betweenChordalFactorGraph.add(BetweenChordalFactor<Pose3>(key1, key2, measured, model));
        }
    }
  return betweenChordalFactorGraph;
}


/******************************************************************************/
TEST(DistributedMapper, testDistributedMapperBetweenChordal) {

  // Read centralized graph
  std::string dataFile("../../../data/blocks_world/2robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values initial = (graphAndValues.second);
  Values centralizedRotation = centralizedRotationEstimation(graph);

  // Linear Chordal Factor
  Values linearChordalFactorEstimate = optimizeUsinglinearChordalFactor(graph, centralizedRotation);

  // Between Chordal Factor Graph
  NonlinearFactorGraph betweenChordalFactorGraph = createBetweenChordalFactorGraph(graph);

  // Linearize and add prior
  GaussianFactorGraph betweenChordalGaussianFactorGraph = *(betweenChordalFactorGraph.linearize(centralizedRotation));
  betweenChordalGaussianFactorGraph.add(JacobianFactor(0, eye(6), zero(6), gtsam::noiseModel::Isotropic::Variance(6, 1e-12)));
  VectorValues betweenChordalVectorValues = betweenChordalGaussianFactorGraph.optimize();
  Values betweenChordalFactorEstimate = multirobot_util::retractPose3Global(centralizedRotation, betweenChordalVectorValues);

  // Compare
  COMPARE_ESTIMATES(linearChordalFactorEstimate, betweenChordalFactorEstimate, 1e-4);

}
/******************************************************************************/
TEST(DistributedMapper, testDistributedMapperBetweenAndPriorChordal) {
  // This compares:
  // 1) GFG with linearChordalFactor + optimize + retract
  // 2) NFG with betweenChordalFactor + linearize + optimize + retract

  // Read centralized graph
  std::string dataFile("../../../data/blocks_world/2robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values initial = (graphAndValues.second);
  Values centralizedRotation = centralizedRotationEstimation(graph);

  // 1) GFG with linearChordalFactor + optimize + retract
  // Linear Factor Chordal - this automatically adds prior
  Values linearChordalFactorEstimate = optimizeUsinglinearChordalFactor(graph, centralizedRotation);

  // 2) NFG with betweenChordalFactor + linearize + optimize + retract
  // Between Chordal Factor
  NonlinearFactorGraph betweenChordalFactorGraph = createBetweenChordalFactorGraph(graph);
  betweenChordalFactorGraph.add(PriorFactor<Pose3>(0, Pose3(), priorModel));

  // Linearize and Optimize
  GaussianFactorGraph betweenChordalGaussianFactorGraph = *(betweenChordalFactorGraph.linearize(centralizedRotation));
  VectorValues betweenChordalVectorValues = betweenChordalGaussianFactorGraph.optimize();
  Values betweenChordalFactorEstimate = multirobot_util::retractPose3Global(centralizedRotation, betweenChordalVectorValues);

  // Compare
  COMPARE_ESTIMATES(linearChordalFactorEstimate, betweenChordalFactorEstimate, 1e-4);
}

/******************************************************************************/
TEST(DistributedMapper, testDistributedMapperOptimizeChordalFactor) {
  // This compares:
  // 1) GFG with linearChordalFactor + optimize + retract
  // 2) 1 GN iteration on NFG with betweenChordalFactor: this does not work with tight tolerance since the retract is different

  // Read centralized graph
  std::string dataFile("../../../data/blocks_world/2robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values initial = (graphAndValues.second);
  Values centralizedRotation = centralizedRotationEstimation(graph);

  // Linear Factor Chordal
  Values linearChordalFactorEstimate = optimizeUsinglinearChordalFactor(graph, centralizedRotation);

  // Between Chordal Factor Graph
  NonlinearFactorGraph betweenChordalFactorGraph = createBetweenChordalFactorGraph(graph);
  betweenChordalFactorGraph.add(PriorFactor<Pose3>(0, Pose3(), priorModel));

  // Optimize
  GaussNewtonParams params;
  params.maxIterations = 1;
  GaussNewtonOptimizer optimizer(betweenChordalFactorGraph, centralizedRotation, params);
  Values betweenChordalFactorEstimate = optimizer.optimize();

  // Compare
  COMPARE_ESTIMATES(linearChordalFactorEstimate, betweenChordalFactorEstimate, 1);

}

/****************************************************************************** */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

