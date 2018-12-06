#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

int main(int argc, char **argv) {

    PointCloudAligner pclAligner;

    // Reading input Clouds
    pclAligner.loadFixedCloudFromDisk("point_cloud", "20180524-mavic-uav-soybean-eschikon", "cloud" );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row4", "row4_cloud", "cloud", Vector2(5.3, 5.3) );
    pclAligner.loadMovingCloudFromDisk("point_cloud", "20180524-mavic-ugv-soybean-eschikon-row4", "row4_cloud_init_guess", "cloud", Vector2(5.3, 5.3) );

    // Transforming Clouds with Grount Truth
    pclAligner.GroundTruthTransformPointCloud("row4_cloud");
    pclAligner.GroundTruthTransformPointCloud("row4_cloud_init_guess");

    // Computing Filtered ExG Clouds
    pclAligner.computeExGFilteredPointCloud("cloud", Vector3i(0,200,0));
    pclAligner.computeExGFilteredPointCloud("row4_cloud", Vector3i(255,0,0));
    pclAligner.computeExGFilteredPointCloud("row4_cloud_init_guess", Vector3i(0,0,255));
    
    // Visualize
    PointCloudViz viz;
    viz.setViewerBackground(255,255,255);

    viz.showCloud( pclAligner.getFilteredPcl("cloud"), "cloud" );
    viz.showCloud( pclAligner.getFilteredPcl("row4_cloud"), "row4_cloud" );
    viz.setViewerPosition(0,0,80,-1,0,0);

    int count = 0;
    while(true){

        srand (count);
        Vector2 vd(1.f, 0.f);
        Vector2 v(1.f,0.f);
        float CircleSamplingAngle = ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * 6.28;

        float transMag = ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * 6;
        float yawMag = ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * .6;
        float scaleMag = ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * .4;

        // Translational Noise With Sampling over a Fixed Input Magnitude
        Vector2 trans = Eigen::Rotation2Df( CircleSamplingAngle )*vd;
        trans(0) *= transMag +  ( .025*transMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * transMag ) );
        trans(1) *= transMag +  ( .025*transMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * transMag ) );

        // Translational Noise With Sampling over a Fixed Input Magnitude
        float yaw = yawMag + ( .025*yawMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * yawMag ) );

        // Translational Noise With Sampling over a Fixed Input Magnitude
        Vector2 scale = Eigen::Rotation2Df( CircleSamplingAngle )*v;
        scale(0) *= scaleMag + (.025*scaleMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * scaleMag ));
        scale(1) *= scaleMag + (.025*scaleMag - ( static_cast <float> (rand()) / static_cast <float> (RAND_MAX) ) * (.05 * scaleMag ));
        scale = Vector2(1.f,1.f) + scale;

        Transform noise_T(Transform::Identity());
        noise_T.translation() << trans(0), trans(1), 0;
        noise_T.rotate( Eigen::AngleAxisf( yaw, Eigen::Vector3f::UnitZ() ) );

        pclAligner.scalePointCloud(scale, "row4_cloud_init_guess", "exg");
        pclAligner.transformPointCloud(noise_T, "row4_cloud_init_guess", "exg");
        viz.showCloud( pclAligner.getFilteredPcl("row4_cloud_init_guess"), "row4_cloud_init_guess" );

        viz.spinOnce();
        sleep(1);

        std::cerr << transMag << " " << trans.transpose() << " " << yawMag << " " 
                  << yaw << " " << scaleMag << " " << scale.transpose() << "\n";

        viz.removeCloud("row4_cloud_init_guess");
        pclAligner.transformPointCloud(noise_T.inverse(), "row4_cloud_init_guess", "exg");
        pclAligner.scalePointCloud(Vector2(1/scale(0), 1/scale(1)), "row4_cloud_init_guess", "exg");
        count++;
    }

    return 0;
}
