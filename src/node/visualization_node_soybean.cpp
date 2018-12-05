#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

int main(int argc, char **argv) {

    if( argc < 2 ){
        std::cerr << FRED("ERROR: too few parameters\n");
        std::exit(1);
    }

    string cloud_viz_type = argv[1];
    if( cloud_viz_type.compare("rgb") != 0 && cloud_viz_type.compare("exg") != 0 ){
        std::cerr << FRED("param ERROR: ") << " node_name <VIZ_TYPE = rgb/exg >\n";
        std::exit(1);
    }

    PointCloudAligner pclAligner;

    // Reading input Clouds
    pclAligner.loadFixedCloudFromDisk("point_cloud", "20180524-mavic-uav-soybean-eschikon", "cloud" );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row3", "row3_cloud", "cloud", Vector2(7, 7) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row4", "row4_cloud", "cloud", Vector2(5.3, 5.3) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row5", "row5_cloud", "cloud", Vector2(2.25, 2.25) );

    // Transforming Clouds with Grount Truth
    pclAligner.GroundTruthTransformPointCloud("row3_cloud");
    pclAligner.GroundTruthTransformPointCloud("row4_cloud");
    pclAligner.GroundTruthTransformPointCloud("row5_cloud");

    // Computing Filtered ExG Clouds
    pclAligner.computeExGFilteredPointCloud("cloud", Vector3i(0,200,0) );
    pclAligner.computeExGFilteredPointCloud("row3_cloud", Vector3i(255,0,255) );
    pclAligner.computeExGFilteredPointCloud("row4_cloud", Vector3i(255,0,0) );
    pclAligner.computeExGFilteredPointCloud("row5_cloud", Vector3i(0,0,255) );


    /*pclAligner.getPointCloud("cloud")->downsamplePointCloud(0.04);
    pclAligner.getPointCloud("row3_cloud")->downsamplePointCloud(0.01);
    pclAligner.getPointCloud("row4_cloud")->downsamplePointCloud(0.01);
    pclAligner.getPointCloud("row5_cloud")->downsamplePointCloud(0.01);*/


    // Enhance Brightness for the UAV Cloud
    int brightness = 100;
    pclAligner.BrightnessEnhancement("cloud", 75);
    
    // Visualize
    PointCloudViz viz;
    viz.setViewerBackground(255,255,255);

    if( cloud_viz_type.compare("exg") == 0 ){
        viz.showCloud( pclAligner.getFilteredPcl("row3_cloud"), "row3_cloud" );
        viz.showCloud( pclAligner.getFilteredPcl("row4_cloud"), "row4_cloud" );
        viz.showCloud( pclAligner.getFilteredPcl("row5_cloud"), "row5_cloud" );
        viz.showCloud( pclAligner.getFilteredPcl("cloud"), "cloud" );
    } else if ( cloud_viz_type.compare("rgb") == 0 ){
        viz.showCloud( pclAligner.getPcl("row3_cloud"), "row3_cloud" );
        viz.showCloud( pclAligner.getPcl("row4_cloud"), "row4_cloud" );
        viz.showCloud( pclAligner.getPcl("row5_cloud"), "row5_cloud" );
        viz.showCloud( pclAligner.getPcl("cloud"), "cloud" );
    }

    viz.setViewerPosition(0,0,80,-1,0,0);
    viz.spingUntilDeath();

    return 0;
}
