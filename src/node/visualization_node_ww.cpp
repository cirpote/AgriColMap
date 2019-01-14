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
    pclAligner.loadFixedCloudFromDisk("point_cloud", "20180311-mavic-ww-eschikon", "cloud" );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180311-mavic-ugv-ww-eschikon", "row1_cloud", "cloud", Vector2(1, 1) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180311-mavic-ugv-ww-eschikon-2", "row2_cloud", "cloud", Vector2(1, 1) );

    // Transforming Clouds with Grount Truth
    pclAligner.GroundTruthTransformPointCloud("row1_cloud");
    pclAligner.GroundTruthTransformPointCloud("row2_cloud");

    // Computing Filtered ExG Clouds
    pclAligner.computeExGFilteredPointCloud("cloud", Vector3i(0,200,0) );
    pclAligner.computeExGFilteredPointCloud("row1_cloud", Vector3i(255,0,0) );
    pclAligner.computeExGFilteredPointCloud("row2_cloud", Vector3i(0,0,255) );

    // Visualizer
    PointCloudViz viz("PointCloud_Viewer", 1600, 900);

    if( cloud_viz_type.compare("exg") == 0 ){
        viz.VisualizePointCloud( pclAligner.getFilteredPcl("row1_cloud") );
        viz.VisualizePointCloud( pclAligner.getFilteredPcl("row2_cloud") );
        viz.VisualizePointCloud( pclAligner.getFilteredPcl("cloud") );
    } else if ( cloud_viz_type.compare("rgb") == 0 ){
        viz.VisualizePointCloud( pclAligner.getPcl("row1_cloud") );
        viz.VisualizePointCloud( pclAligner.getPcl("row2_cloud") );
        viz.VisualizePointCloud( pclAligner.getPcl("cloud") );
    }

    viz.spingUntilDeath();

    return 0;
}
