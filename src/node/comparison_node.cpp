#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

int main(int argc, char **argv) {

    if(argc < 6) {
        cerr << FRED("Other Params Expected!") << " node_name <params_file.txt> <Scale_Mag> <GPS_Noise_Mag> <Yaw_Noise_Mag> <Exp_ID>" << "\n";
        std::exit(1);
    }

    string yaml_filename = argv[1];

    PointCloudAligner pclAligner;
    pclAligner.initFromYaml(yaml_filename);
    pclAligner.loadFromDisk("fixed_cloud", "moving_cloud");

    string scaleMagStr = argv[2];
    float scaleMag = stof(scaleMagStr)/100;
    string TranslNoiseMagStr = argv[3];
    float TranslNoiseMag = stof(TranslNoiseMagStr)/100;
    string YawNoiseMagStr = argv[4];
    float YawNoiseMag = stof(YawNoiseMagStr)/10;
    string ExpIDStr = argv[5];


  pclAligner.getPointCloud("moving_cloud")->addNoise(scaleMag, TranslNoiseMag, YawNoiseMag);
  pclAligner.computeAndApplyInitialRelativeGuess("fixed_cloud", "moving_cloud");
  pclAligner.cropFixedPointCloud("crop_fixed_cloud", "moving_cloud", "fixed_cloud");
  pclAligner.computeDensifiedPCLs( "crop_fixed_cloud", "moving_cloud");
  pclAligner.GroundTruthTransformPointCloud("moving_cloud");
  pclAligner.MatchGoICP("crop_fixed_cloud", "moving_cloud");

  PointCloudViz viz;
  viz.setViewerBackground(255,255,255);
  viz.showCloud( pclAligner.getPointCloud("crop_fixed_cloud")->getPointCloudFiltered(), "crop_fixed_cloud" );
  viz.showCloud( pclAligner.getPointCloud("moving_cloud")->getPointCloudFiltered(), "moving_cloud" );
  viz.spingUntilDeath();

  return 0;
}
