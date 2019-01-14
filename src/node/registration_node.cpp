#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

int main(int argc, char **argv) {

    if(argc < 6) {
        cerr << FRED("Other Params Expected!") << " node_name <params_file.txt> <Scale_Mag> <GPS_Noise_Mag> <Yaw_Noise_Mag> <Exp_ID>" << "\n";
        std::exit(1);
    }

    string yaml_filename = argv[1];
    std::string mov_cloud = "moving_cloud";
    std::string fix_cloud = "fixed_cloud";

    PointCloudAligner pclAligner;
    pclAligner.initFromYaml(yaml_filename);
    pclAligner.loadFromDisk(fix_cloud, mov_cloud);

    string scaleMagStr = argv[2];
    float scaleMag = stof(scaleMagStr)/100;
    string TranslNoiseMagStr = argv[3];
    float TranslNoiseMag = stof(TranslNoiseMagStr)/100;
    string YawNoiseMagStr = argv[4];
    float YawNoiseMag = stof(YawNoiseMagStr)/10;
    string ExpIDStr = argv[5];

    // Adding Noise to Initial Guess
    pclAligner.addNoise( mov_cloud, scaleMag, TranslNoiseMag, YawNoiseMag );
    pclAligner.computeAndApplyInitialRelativeGuess(fix_cloud, mov_cloud);
    pclAligner.computeExGFilteredPointCloud(mov_cloud, Vector3i(0,0,255));
    pclAligner.computeExGFilteredPointCloud(fix_cloud, Vector3i(255,0,0));
    pclAligner.computeEnvironmentalModels(mov_cloud, fix_cloud);
    pclAligner.Match(fix_cloud, mov_cloud, pclAligner.getInitMovScale(), ExpIDStr, cv::Size(1300,1300) );

    PointCloudViz viz("PointCloud_Viewer", 1600, 900);
    viz.VisualizePointCloud( pclAligner.getFilteredPcl(fix_cloud) );
    viz.VisualizePointCloud( pclAligner.getFilteredPcl(mov_cloud) );
    //viz.showCloud( pclAligner.getPcl(fix_cloud), fix_cloud);
    //viz.showCloud( pclAligner.getPcl(mov_cloud), mov_cloud);
    viz.spingUntilDeath();

    return 0;
}
