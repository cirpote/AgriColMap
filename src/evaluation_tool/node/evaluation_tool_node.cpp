#include <uav_ugv_collaboration_module/evaluation_tool/evaluation_tool.h>

using namespace std;
using namespace pcl_utils;

int main(int argc, char **argv) {
  
  std::string yaml_filename;
  if(argc < 2) 
    throw std::runtime_error("params file expected!");
  
  yaml_filename = argv[1];
  
  ros::init(argc, argv, "uav_ugv_collaboration_module");
  ros::NodeHandle n("~");
  evaluation_tool::EvaluationTool eval_tool;
  eval_tool.initFromYaml(yaml_filename);
  
  eval_tool.ReadAndComputeInitGuess();
  eval_tool.ExGConversion( WHITE, Vector3(0,255,0), WHITE, Vector3(0,255,0) );
  std::string fixed_cloud_viewer_id = "fixed_cloud_viewer_id";
  std::string moving_cloud_viewer_id = "moving_cloud_viewer_id";

  eval_tool.scalePointCloud("moving_cloud", 2.2);

  if( eval_tool.checkForScaleError() ){

    cerr << "Visualizing: " << "fixed_cloud" << "\n" << "\n";
    eval_tool.addPointCloudtoViewer("fixed_cloud", fixed_cloud_viewer_id);
    eval_tool.visualize();
    eval_tool.visualizeAndSpin();
    eval_tool.removePointCloudtoViewer(fixed_cloud_viewer_id);
    eval_tool.checkFixedCloudScaleError();

    cerr << "Visualizing: " << "moving_cloud" << "\n" << "\n";
    eval_tool.addPointCloudtoViewer("moving_cloud", moving_cloud_viewer_id);
    eval_tool.visualize();
    eval_tool.visualizeAndSpin();
    eval_tool.removePointCloudtoViewer(moving_cloud_viewer_id);
    eval_tool.checkMovingCloudScaleError();
  }

  cerr << "Visualizing: " << "fixed_cloud" << "\n" << "\n";
  eval_tool.addPointCloudtoViewer("fixed_cloud", fixed_cloud_viewer_id);
  eval_tool.visualize();
  eval_tool.visualizeAndSpin();
  eval_tool.removePointCloudtoViewer(fixed_cloud_viewer_id);
  eval_tool.storeFixedPts();

  cerr << "Visualizing: " << "moving_cloud" << "\n" << "\n";
  eval_tool.addPointCloudtoViewer("moving_cloud", moving_cloud_viewer_id);
  eval_tool.visualize();
  eval_tool.visualizeAndSpin();
  eval_tool.removePointCloudtoViewer(moving_cloud_viewer_id);
  eval_tool.storeMovingPts();

  cerr << "Computing Ground Truth Transformation ..." << "\n";
  Transform GT_transform = eval_tool.computeGtTransform();

  eval_tool.TransformingPointCloud(GT_transform, "moving_cloud");
  cerr << "Visualizing: " << " moving_cloud & fixed_cloud aligned according to GT_transform " << "\n" << "\n";
  eval_tool.addPointCloudtoViewer("fixed_cloud", fixed_cloud_viewer_id);
  eval_tool.addPointCloudtoViewer("moving_cloud", moving_cloud_viewer_id);
  eval_tool.visualize();
  eval_tool.visualizeAndSpin();

  cerr << "\n" << "Writing Output Ground Truth Transform to YAML file ..." << "\n";
  eval_tool.writeTransformYAML("Ground_Truth_Transform_Example.yaml");
  
  return 0;
  
}