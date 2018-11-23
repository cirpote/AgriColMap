#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

int main(int argc, char **argv) {

    PointCloudAligner pclAligner;

    // Reading input Clouds
    pclAligner.loadFixedCloudFromDisk("point_cloud", "20180524-mavic-uav-maize-eschikon", "cloud" );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-maize-eschikon-row1", "row1_cloud", "cloud", Vector2(1, 1) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-maize-eschikon-row2", "row2_cloud", "cloud", Vector2(1, 1) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-maize-eschikon-row3", "row3_cloud", "cloud", Vector2(1, 1) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-maize-eschikon-row5", "row5_cloud", "cloud", Vector2(1, 1) );

    // Computing Filtered ExG Clouds
    pclAligner.getPointCloud("row1_cloud")->computeFilteredPcl( Vector3i(255,0,0) );
    pclAligner.getPointCloud("row2_cloud")->computeFilteredPcl( Vector3i(0,255,0) );
    pclAligner.getPointCloud("row3_cloud")->computeFilteredPcl( Vector3i(0,0,255) );
    pclAligner.getPointCloud("row5_cloud")->computeFilteredPcl( Vector3i(0,255,255) );

    // Transforming Clouds with Grount Truth
    pclAligner.GroundTruthTransformPointCloud("row1_cloud");
    pclAligner.GroundTruthTransformPointCloud("row2_cloud");
    pclAligner.GroundTruthTransformPointCloud("row3_cloud");
    pclAligner.GroundTruthTransformPointCloud("row5_cloud");

    // Enhance Brightness for the UAV Cloud
    int brightness = 100;
    pclAligner.getPointCloud("cloud")->BrightnessEnhancement(brightness);

    // Visualize
    PointCloudViz viz;
    viz.setViewerBackground(255,255,255);
    viz.showCloud( pclAligner.getPointCloud("row1_cloud")->getPointCloudFiltered(), "row1_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row2_cloud")->getPointCloudFiltered(), "row2_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row3_cloud")->getPointCloudFiltered(), "row3_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row5_cloud")->getPointCloudFiltered(), "row5_cloud" );
    viz.showTransparentCloud( pclAligner.getPointCloud("cloud")->getPointCloud(), "cloud" );
    viz.spingUntilDeath();

    return 0;
}
