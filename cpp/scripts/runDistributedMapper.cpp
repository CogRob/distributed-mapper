/**
 * @file runDistributedMapper.cpp
 * @brief script for running g2o files
 */

#include <DistributedMapperUtils.h>
#include <MultiRobotUtils.h>
#include <BetweenChordalFactor.h>

#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


using namespace std;
using namespace gtsam;
using namespace distributed_mapper;
using namespace multirobot_util;

/**
 * @brief readFullGraph reads the full graph if it is present in the directory, otherwise creates it
 * @param nrRobots is the number of robots
 * @param graphAndValuesVec contains the graphs and initials of each robot
 */
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

/**
 * @brief copyInitial copies the initial graph to optimized graph as a fall back option
 * @param nrRobots is the number of robots
 * @param dataDir is the directory containing the initial graph
 */
void copyInitial(size_t nrRobots, std::string dataDir){
  cout << "Copying initial to optimized" << endl;
  for(size_t robot = 0; robot < nrRobots; robot++){
      string dataFile_i = dataDir + boost::lexical_cast<string>(robot) + ".g2o";
      GraphAndValues graphAndValuesG2o = readG2o(dataFile_i, true);
      NonlinearFactorGraph graph = *(graphAndValuesG2o.first);
      Values initial = *(graphAndValuesG2o.second);

      // Write optimized full graph
      string distOptimized_i = dataDir + boost::lexical_cast<string>(robot) + "_optimized.g2o";
      writeG2o(graph, initial, distOptimized_i);
    }
}


/**
 * @brief main function
 */
int main(int argc, char* argv[])
{
  //////////////////////////////////////////////////////////////////////////////////////
  //Command line arguments
  //////////////////////////////////////////////////////////////////////////////////////
  size_t nrRobots = 2; // number of robots
  string logDir("/tmp/"); // log directory
  string dataDir("/tmp/"); // data directory
  string traceFile("/tmp/runG2o"); // data directory
  bool useXY = false;
  bool useOP = false;
  bool debug = false;

  ////////////////////////////////////////////////////////////////////////////////
  // Config (specify noise covariances and maximum number iterations)
  ////////////////////////////////////////////////////////////////////////////////
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
  size_t maxIter = 1000; // Maximum number of iterations of optimizer
  double rotationEstimateChangeThreshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
  double poseEstimateChangeThreshold = 1e-1; // Difference between pose estimate provides an early stopping condition
  double gamma = 1.0f; // Gamma value for over relaxation methods
  bool useFlaggedInit = true; // to use flagged initialization or not
  DistributedMapper::UpdateType updateType = DistributedMapper::incUpdate; // updateType differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
  vector<GraphAndValues> graphAndValuesVec; // vector of all graphs and initials
  bool useBetweenNoise = false; // use between factor noise or not
  bool useChrLessFullGraph = false; // whether full graph has character indexes or not
  bool useLandmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'

  try{
    // Parse program options
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "Print help messages")
        ("nrRobots, n", po::value<size_t>(&nrRobots), "number of robots (default: 2)")
        ("dataDir, l", po::value<string>(&dataDir), "data directory (default: /tmp)")
        ("traceFile, t", po::value<string>(&traceFile), "trace file (default: runG2o)")
        ("logDir, l", po::value<string>(&logDir), "log directory (default: /tmp)")
        ("useXY, u", po::value<bool>(&useXY), "use x,y,z as naming convention or a,b,c (default: x,y,z)")
        ("useOP, o", po::value<bool>(&useOP), "use o,p,q as naming convention (default: x,y,z)")
        ("useFlaggedInit, f", po::value<bool>(&useFlaggedInit), "use flagged initialization or not (default: true)")
        ("useBetweenNoise, b", po::value<bool>(&useBetweenNoise), "use the given factor between noise instead of unit noise(default: false)")
        ("useChrLessFullGraph", po::value<bool>(&useChrLessFullGraph), "whether full graph has character indexes or not (default: false)")
        ("useLandmarks, l", po::value<bool>(&useLandmarks), "use landmarks or not (default: false)")
        ("rthresh, r", po::value<double>(&rotationEstimateChangeThreshold), "Specify difference between rotation estimate provides an early stopping condition (default: 1e-2)")
        ("pthresh, p", po::value<double>(&poseEstimateChangeThreshold), "Specify difference between pose estimate provides an early stopping condition (default: 1e-2)")
        ("maxIter, m", po::value<size_t>(&maxIter), "maximum number of iterations (default: 100000)")
        ("debug, d", po::value<bool>(&debug), "debug (default: false)");

    po::variables_map vm;
    try{
      po::store(po::parse_command_line(argc, argv, desc), vm); // can throw
      if ( vm.count("help")  ){ // --help option
          cout << "Run Distributed-Mapper" << endl << "Example: ./rung2o --dataDir ../../../example/ --nrRobots 4"
               << endl << desc << endl;
          return 0;
        }
      po::notify(vm); // throws on error, so do after help in case
    }
    catch(po::error& e){
      cerr << "ERROR: " << e.what() << endl << endl;
      cerr << desc << endl;
      return 1;
    }
  }
  catch(exception& e){
    cerr << "Unhandled Exception reached the top of main: "
         << e.what() << ", application will now exit" << endl;
    return 2;
  }

  // Config
  string robotNames_;
  if(useXY){
      robotNames_ = string("xyz"); // robot names
    }
  else{
      robotNames_ = string("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"); // robot names
    }

  if(useOP){
      robotNames_ = string("opqrstuvwxyz"); // robot names
    }

  if(useLandmarks){
      robotNames_ = string("abcdefghijklmnopqrstyvwxyz"); // robot names
      // ABC... are used for objects
    }


  bool disconnectedGraph = false; // Flag to check whether graphs are connected or not

  ////////////////////////////////////////////////////////////////////////////////
  // Distributed Optimization
  ////////////////////////////////////////////////////////////////////////////////

  // Vector of distributed optimizers, one for each robot
  vector< boost::shared_ptr<DistributedMapper> > distMappers;

  // Load subgraph and construct distMapper optimizers
  for(size_t robot = 0; robot < nrRobots; robot++){

      // Construct a distributed jacobi object with the given robot name
      boost::shared_ptr<DistributedMapper> distMapper(new DistributedMapper(robotNames_[robot], useChrLessFullGraph));

      // Read G2o files
      string dataFile_i = dataDir + boost::lexical_cast<string>(robot) + ".g2o";
      GraphAndValues graphAndValuesG2o = readG2o(dataFile_i, true);
      Values initial = *(graphAndValuesG2o.second);

      // Continue if empty
      if(initial.empty()){
          disconnectedGraph = true;
          continue;
        }

      // Construct graphAndValues using cleaned up initial values
      GraphAndValues graphAndValues =  make_pair(graphAndValuesG2o.first, boost::make_shared<Values>(initial));
      graphAndValuesVec.push_back(graphAndValues);

      // Use between noise or not in optimizePoses
      distMapper->setUseBetweenNoiseFlag(useBetweenNoise);

      // Use landmarks
      distMapper->setUseLandmarksFlag(useLandmarks);

      // Load subgraphs
      distMapper->loadSubgraphAndCreateSubgraphEdge(graphAndValues);

      // Add prior to the first robot
      if(robot==0){
          Key firstKey = KeyVector(initial.keys()).at(0);
          distMapper->addPrior(firstKey, initial.at<Pose3>(firstKey), priorModel);
        }

      // Verbosity level
      distMapper->setVerbosity(DistributedMapper::ERROR);

      // Check for graph connectivity
      std::set<char> neighboringRobots = distMapper->getNeighboringChars();
      if(neighboringRobots.size() == 0)
        disconnectedGraph = true;

      // Push to the set of optimizers
      distMappers.push_back(distMapper);
    }

  // Vectors containing logs
  vector < Values > rotationTrace;
  vector < Values > poseTrace;
  vector < Values > subgraphRotationTrace;
  vector < Values > subgraphPoseTrace;
  vector < VectorValues > rotationVectorValuesTrace;

  if(debug)
    cout << "Optimizing" << endl;
  // Distributed Estimate

  if(!disconnectedGraph){
      try{
        // try optimizing
        vector< Values > estimates =  distributedOptimizer(distMappers, maxIter, updateType, gamma,
                                                           rotationEstimateChangeThreshold, poseEstimateChangeThreshold, useFlaggedInit, useLandmarks,
                                                           debug, rotationTrace, poseTrace, subgraphRotationTrace, subgraphPoseTrace, rotationVectorValuesTrace);

        if(debug)
          cout << "Done" << endl;

        // Aggregate estimates from all the robots
        Values distributed;
        for(size_t i = 0; i< estimates.size(); i++){
            for(const Values::ConstKeyValuePair& key_value: estimates[i]){
                Key key = key_value.key;
                if(!distributed.exists(key))
                  distributed.insert(key, estimates[i].at<Pose3>(key));
              }

            // Write the corresponding estimate to disk
            string distOptimized_i = dataDir + boost::lexical_cast<string>(i) + "_optimized.g2o";
            writeG2o(*(graphAndValuesVec[i].first), estimates[i], distOptimized_i);

            // Write the corresponding estimate in TUM format
            string distOptimized_i_tum = dataDir + boost::lexical_cast<string>(i) + "_optimizedTUM.txt";
            multirobot_util::writeValuesAsTUM(estimates[i], distOptimized_i_tum);
          }

        if(debug)
          cout << "Done Aggregating" << endl;


        ////////////////////////////////////////////////////////////////////////////////
        // Read full graph and add prior
        ////////////////////////////////////////////////////////////////////////////////
        GraphAndValues fullGraphAndValues = readFullGraph(nrRobots, graphAndValuesVec);
        NonlinearFactorGraph fullGraph = *(fullGraphAndValues.first);
        Values fullInitial = *(fullGraphAndValues.second);

        // Add prior
        NonlinearFactorGraph fullGraphWithPrior = fullGraph.clone();
        Key priorKey = KeyVector(fullInitial.keys()).at(0);
        NonlinearFactor::shared_ptr prior(new PriorFactor<Pose3>(priorKey, fullInitial.at<Pose3>(priorKey), priorModel));
        fullGraphWithPrior.push_back(prior);

        // Write optimized full graph
        string distOptimized = dataDir +  "fullGraph_optimized.g2o";
        writeG2o(fullGraph, distributed, distOptimized);

        ////////////////////////////////////////////////////////////////////////////////
        // Chordal Graph
        ////////////////////////////////////////////////////////////////////////////////
        NonlinearFactorGraph chordalGraph = distributed_mapper::multirobot_util::convertToChordalGraph(fullGraph, model, useBetweenNoise);

        ////////////////////////////////////////////////////////////////////////////////
        // Initial Error
        ////////////////////////////////////////////////////////////////////////////////
        std::cout << "Initial Error: " << chordalGraph.error(fullInitial) << std::endl;

        ////////////////////////////////////////////////////////////////////////////////
        // Centralized Two Stage
        ////////////////////////////////////////////////////////////////////////////////
        Values centralized = distributed_mapper::multirobot_util::centralizedEstimation(fullGraphWithPrior, model, priorModel, useBetweenNoise);
        std::cout << "Centralized Two Stage Error: " << chordalGraph.error(centralized) << std::endl;

        ////////////////////////////////////////////////////////////////////////////////
        // Centralized Two Stage + Gauss Newton
        ////////////////////////////////////////////////////////////////////////////////
        Values chordalGN = distributed_mapper::multirobot_util::centralizedGNEstimation(fullGraphWithPrior, model, priorModel, useBetweenNoise);
        std::cout << "Centralized Two Stage + GN Error: " << chordalGraph.error(chordalGN) << std::endl;

        ////////////////////////////////////////////////////////////////////////////////
        // Distributed Error
        ////////////////////////////////////////////////////////////////////////////////
        std::cout << "Distributed Error: " << chordalGraph.error(distributed) << std::endl;

      }
      catch(...){
        // Optimization failed (maybe due to disconnected graph)
        // Copy initial to optimized g2o files in that case
        copyInitial(nrRobots, dataDir);
      }
    }
  else{
      // Graph is disconnected
      cout << "Graph is disconnected: " << endl;
      copyInitial(nrRobots, dataDir);
    }


}
