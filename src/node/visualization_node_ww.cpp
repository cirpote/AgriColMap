#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

int main(int argc, char **argv) {

    PointCloudAligner pclAligner;

    // Reading input Clouds
    pclAligner.loadFixedCloudFromDisk("point_cloud", "20180311-mavic-ww-eschikon", "cloud" );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180311-mavic-ugv-ww-eschikon", "row1_cloud", "cloud", Vector2(1, 1) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180311-mavic-ugv-ww-eschikon-2", "row2_cloud", "cloud", Vector2(1, 1) );

    pclAligner.getPointCloud("cloud")->downsamplePointCloud(0.03);
    pclAligner.getPointCloud("row1_cloud")->downsamplePointCloud(0.02);
    pclAligner.getPointCloud("row2_cloud")->downsamplePointCloud(0.02);

    // Computing Filtered ExG Clouds
    pclAligner.getPointCloud("cloud")->computeFilteredPcl( Vector3i(0,200,0) );
    pclAligner.getPointCloud("row1_cloud")->computeFilteredPcl( Vector3i(255,0,0) );
    pclAligner.getPointCloud("row2_cloud")->computeFilteredPcl( Vector3i(0,0,255) );

    // Transforming Clouds with Grount Truth
    pclAligner.GroundTruthTransformPointCloud("row1_cloud");
    pclAligner.GroundTruthTransformPointCloud("row2_cloud");

    // Enhance Brightness for the UAV Cloud
    /*int brightness = 100;
    pclAligner.getPointCloud("cloud")->BrightnessEnhancement(brightness);*/

    // Visualize
    PointCloudViz viz;
    viz.setViewerBackground(255,255,255);

    viz.showCloud( pclAligner.getPointCloud("row1_cloud")->getPointCloudFiltered(), "row1_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row2_cloud")->getPointCloudFiltered(), "row2_cloud" );
    viz.showTransparentCloud( pclAligner.getPointCloud("cloud")->getPointCloudFiltered(), "cloud" );

    /*viz.showCloud( pclAligner.getPointCloud("row1_cloud")->getPointCloud(), "row1_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row2_cloud")->getPointCloud(), "row2_cloud" );
    viz.showTransparentCloud( pclAligner.getPointCloud("cloud")->getPointCloud(), "cloud" );
*/
    viz.setViewerPosition(0,0,80,-1,0,0);
    viz.spingUntilDeath();

    return 0;
}
