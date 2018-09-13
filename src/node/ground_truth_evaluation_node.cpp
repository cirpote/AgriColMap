#include "../pointcloud_handler/ground_truth_evaluation.h"

using namespace std;

int main(int argc, char **argv) {
  
  std::string yaml_filename;
  if(argc < 2) {
    cerr << FRED("Params File Expected!") << " node_name <params_file.txt>" << "\n";
    std::exit(1);
  }

  yaml_filename = argv[1];
  cv::Size imgs_size(1000,1000);

  GroundTruthEvaluation gtEvalTool;
  gtEvalTool.initFromYaml(yaml_filename);
  gtEvalTool.loadFromDisk("fixed_cloud", "moving_cloud");

  // Scale (2,2) for row 5, (5,5) for row4, (6,6) for row3
  gtEvalTool.computeAndApplyInitialRelativeGuess("fixed_cloud", "moving_cloud");
  gtEvalTool.scalePointCloud( gtEvalTool.getInitMovScale(), "moving_cloud"); //(2.28, 2.28) row5, (5, 5) row4, (6, 6)  row3
  gtEvalTool.cropFixedPointCloud("crop_fixed_cloud");

  gtEvalTool.computeDensifiedPCLs( "crop_fixed_cloud", "moving_cloud", imgs_size ); //(-3,-2) row5, (-2,-1) row4, (4,3) row3
  gtEvalTool.Match( "crop_fixed_cloud", "moving_cloud", imgs_size, gtEvalTool.getInitMovScale() );
  //gtEvalTool.getPointCloud("moving_cloud")->affineTransformPointCloud( gtEvalTool.getGTAffineTransform(), gtEvalTool.getGTTranslation() );

  gtEvalTool.showCloud("crop_fixed_cloud");
  gtEvalTool.showCloud("moving_cloud");
  gtEvalTool.spingUntilDeath();

  return 0;
}




