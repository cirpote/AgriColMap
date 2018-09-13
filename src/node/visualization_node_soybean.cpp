#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

int main(int argc, char **argv) {

    PointCloudAligner pclAligner;

    // Reading input Clouds
    pclAligner.loadFixedCloudFromDisk("point_cloud", "20180524-mavic-uav-soybean-eschikon", "cloud" );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row3", "row3_cloud", "cloud", Vector2(7, 7) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row4", "row4_cloud", "cloud", Vector2(5.3, 5.3) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row5", "row5_cloud", "cloud", Vector2(2.25, 2.25) );

    /*pclAligner.getPointCloud("cloud")->downsamplePointCloud(0.04);
    pclAligner.getPointCloud("row3_cloud")->downsamplePointCloud(0.01);
    pclAligner.getPointCloud("row4_cloud")->downsamplePointCloud(0.01);
    pclAligner.getPointCloud("row5_cloud")->downsamplePointCloud(0.01);*/

    // Computing Filtered ExG Clouds
    pclAligner.getPointCloud("cloud")->computeFilteredPcl( Vector3i(0,200,0) );
    pclAligner.getPointCloud("row3_cloud")->computeFilteredPcl( Vector3i(255,0,0) );
    pclAligner.getPointCloud("row4_cloud")->computeFilteredPcl( Vector3i(0,0,255) );
    pclAligner.getPointCloud("row5_cloud")->computeFilteredPcl( Vector3i(255,0,255) );

    // Transforming Clouds with Grount Truth
    pclAligner.GroundTruthTransformPointCloud("row3_cloud");
    pclAligner.GroundTruthTransformPointCloud("row4_cloud");
    pclAligner.GroundTruthTransformPointCloud("row5_cloud");

    // Enhance Brightness for the UAV Cloud
    //int brightness = 100;
    //pclAligner.getPointCloud("cloud")->BrightnessEnhancement(brightness);

    // Visualize
    PointCloudViz viz;
    viz.setViewerBackground(255,255,255);

    viz.showCloud( pclAligner.getPointCloud("row3_cloud")->getPointCloudFiltered(), "row3_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row4_cloud")->getPointCloudFiltered(), "row4_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row5_cloud")->getPointCloudFiltered(), "row5_cloud" );
    viz.showCloud( pclAligner.getPointCloud("cloud")->getPointCloudFiltered(), "cloud" );

    /*viz.showCloud( pclAligner.getPointCloud("row3_cloud")->getPointCloud(), "row3_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row4_cloud")->getPointCloud(), "row4_cloud" );
    viz.showCloud( pclAligner.getPointCloud("row5_cloud")->getPointCloud(), "row5_cloud" );
    viz.showCloud( pclAligner.getPointCloud("cloud")->getPointCloud(), "cloud" );*/

    viz.setViewerPosition(0,0,80,-1,0,0);
    viz.spingUntilDeath();

    return 0;
}
