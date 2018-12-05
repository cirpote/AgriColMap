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
    pclAligner.loadFixedCloudFromDisk("point_cloud", "20180524-mavic-uav-20m-sugarbeet-eschikon", "cloud" );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-sugarbeet-eschikon-row3_20m", "row3_cloud", "cloud", Vector2(1, 1) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-sugarbeet-eschikon-row4_20m", "row4_cloud", "cloud", Vector2(1, 1) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-sugarbeet-eschikon-row5_20m", "row5_cloud", "cloud", Vector2(1, 1) );

    // Transforming Clouds with Grount Truth
    pclAligner.GroundTruthTransformPointCloud("row3_cloud");
    pclAligner.GroundTruthTransformPointCloud("row4_cloud");
    pclAligner.GroundTruthTransformPointCloud("row5_cloud");

    // Computing Filtered ExG Clouds
    pclAligner.computeExGFilteredPointCloud("cloud", Vector3i(0,200,0) );
    pclAligner.computeExGFilteredPointCloud("row3_cloud", Vector3i(255,0,255) );
    pclAligner.computeExGFilteredPointCloud("row4_cloud", Vector3i(255,0,0) );
    pclAligner.computeExGFilteredPointCloud("row5_cloud", Vector3i(0,0,255) );

    /*
    pclAligner.getPointCloud("cloud")->downsamplePointCloud(0.03);
    pclAligner.getPointCloud("row3_cloud")->downsamplePointCloud(0.01);
    pclAligner.getPointCloud("row4_cloud")->downsamplePointCloud(0.01);
    pclAligner.getPointCloud("row5_cloud")->downsamplePointCloud(0.01);

    Transform z_up(Transform::Identity());
    z_up.translation() << 0,0,10;

    pclAligner.getPointCloud("cloud")->transformPointCloud(z_up);
    pclAligner.getPointCloud("row3_cloud")->transformPointCloud(z_up);
    pclAligner.getPointCloud("row4_cloud")->transformPointCloud(z_up);
    pclAligner.getPointCloud("row5_cloud")->transformPointCloud(z_up);*/

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
