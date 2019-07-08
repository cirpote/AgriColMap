#include "pix4d_input_reader.h"

using namespace std;

template<typename T>
void back_project(T params, Eigen::Vector2d& uv);

int main(int argc, char **argv) {

    // Pix4d Calib Parameters
    string input_calibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_camera_parameters" + ".txt";
    string input_extcalibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_external_camera_parameters" + ".txt";

    // Pix4d output PointCloud 
    string input_pcl_str_xyz = _package_path + "/src/node/08may2019_jesi_bis/2_densification/point_cloud/08may2019_jesi_bis_group1_densified_point_cloud.xyz";

    // Multispectral Calib Parameters
    string input_calibcanparams_str_nir = _package_path + "/src/node/" + "nir_calib" + ".txt";
    string input_calibcanparams_str_gre = _package_path + "/src/node/" + "gre_calib" + ".txt";
    string input_calibcanparams_str_red = _package_path + "/src/node/" + "red_calib" + ".txt";
    string input_calibcanparams_str_reg = _package_path + "/src/node/" + "reg_calib" + ".txt";

    // Stereo Calib Parameters
    string input_calibcanparams_str_gre_nir = _package_path + "/src/node/" + "gre_nir_stereo" + ".txt";
    string input_calibcanparams_str_gre_red = _package_path + "/src/node/" + "gre_red_stereo" + ".txt";
    string input_calibcanparams_str_gre_reg = _package_path + "/src/node/" + "gre_reg_stereo" + ".txt";
    string input_calibcanparams_str_gre_rgb = _package_path + "/src/node/" + "gre_rgb_stereo" + ".txt";

    pix4dInputReader pix4dReader(input_calibcanparams_str);
    pix4dReader.readParamFile();
    pix4dReader.readExtCamCalibParams(input_extcalibcanparams_str);

    pix4dReader.readParamFileMultiSpectral(input_calibcanparams_str_nir,
                                           input_calibcanparams_str_gre,
                                           input_calibcanparams_str_red,
                                           input_calibcanparams_str_reg);

    pix4dReader.readStereoParams(input_calibcanparams_str_gre_nir,
                                 input_calibcanparams_str_gre_red,
                                 input_calibcanparams_str_gre_reg,
                                 input_calibcanparams_str_gre_rgb);

    //string ciao = "IMG_190522_085454_0024_RGB.JPG";
    //int ciao1 = 0;
    //pix4dReader.printParams(ciao1);

    //pix4dReader.printMultiSpectralParams();
    //pix4dReader.printStereoParams();
    
    PCLPointCloudXYZRGB::Ptr _pcl_data( new PCLPointCloudXYZRGB() );
    pix4dReader.loadXYZPointCloud( input_pcl_str_xyz, _pcl_data );
    // pcl::io::loadPLYFile<PCLptXYZRGB> ( input_pcl_str_xyz, *_pcl_data);
    std::cout << "dimensione PCL " << _pcl_data->points.size() << "\n";

    PCLPointCloudXYZRGB::Ptr _pcl_data_gre( new PCLPointCloudXYZRGB( (uint32_t)_pcl_data->points.size() , (uint32_t)1, PCLptXYZRGB(0,0,0) ) );

    int id_prova = 60; // 112, 90
    pix4dReader.printParams(id_prova);
    CalibCamParams&& CalibData = pix4dReader.getCalibData(id_prova);

    std::cout << "qui " << "/home/ciro/AgriColMap/src/node/08may2019_jesi_bis/IMG_190508_100143_0125_GRE.TIF" << "\n";
    cv::Mat curr_gre_img = cv::imread(  "/home/ciro/AgriColMap/src/node/08may2019_jesi_bis/IMG_190508_100143_0125_GRE.TIF", CV_LOAD_IMAGE_GRAYSCALE );
    // cv::imshow("gre_img", curr_gre_img);
    // cv::waitKey(0);

    cout << CalibData.Rx_ext << "\n\n";
    cout << CalibData.Ry_ext << "\n\n";
    cout << CalibData.Rz_ext << "\n\n";

    cout << CalibData.Rx_ext * CalibData.Ry_ext * CalibData.Rz_ext << "\n\n";

    cout << CalibData.Rz_ext * CalibData.Ry_ext * CalibData.Rx_ext << "\n\n";

    cv::Mat output_img( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC3, cv::Scalar(0,0,0) );

    //std::cout << "qui\n";

    int index = 0;
    for( PCLptXYZRGB& pt : *_pcl_data){

        // Eigen::Vector3d cam_pt =  CalibData.Rx_ext.transpose() * CalibData.cam_R.transpose() *  (  CalibData.Rx_ext.transpose() * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        Eigen::Vector3d cam_pt = ( CalibData.Rx_ext * CalibData.Ry_ext * CalibData.Rz_ext ).transpose() * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        Eigen::Vector2d uv_pt;
        uv_pt(0) = ( cam_pt(0)/cam_pt(2) );
        uv_pt(1) = ( cam_pt(1)/cam_pt(2) );
        back_project( CalibData, uv_pt );


        // Eigen::Vector3d cam_pt_gre = - pix4dReader.gre_rgb_extrncs_.t_ + CalibData.cam_R.transpose()*( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        // Eigen::Vector2d uv_pt_gre( cam_pt_gre(0)/cam_pt_gre(2), cam_pt_gre(1)/cam_pt_gre(2) );
        // back_project( pix4dReader.greParams_, uv_pt_gre  );

        // if( uv_pt_gre(0) > 0 && uv_pt_gre(0) < pix4dReader.greParams_.img_width && uv_pt_gre(1) > 0 && uv_pt_gre(1) < pix4dReader.greParams_.img_height ){
        //     PCLptXYZRGB& GREpt = _pcl_data_gre->points[index];
        //     GREpt.x = pt.x;
        //     GREpt.y = pt.y;
        //     GREpt.z = pt.z;
        //     //std::cout << "crasho qui\n";
        //     int greColor = curr_gre_img.at<uchar>( uv_pt_gre(1), uv_pt_gre(0) );
        //     GREpt.r = greColor;
        //     GREpt.g = greColor;
        //     GREpt.b = greColor;
        // }
        //std::cout << " qui\n";

        if( uv_pt(0) > 0 && uv_pt(0) < CalibData.img_width && uv_pt(1) > 0 && uv_pt(1) < CalibData.img_height ){
            int ExG = computeExGforXYZRGBPoint(pt);
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[0] = pt.b;//0;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[1] = pt.g;//ExG*5;//(int)pt.g;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[2] = pt.r;//0;//(int)pt.r;
        }

        index++;
    }

    std::cout << "sono qui\n";
    
    cv::imwrite("/home/ciro/AgriColMap/output.png", output_img);
    
    //pcl::io::savePLYFileBinary( "/home/ciro/AgriColMap/greProva2.ply", *_pcl_data);
    // PointCloudViz viz;
    // viz.setViewerBackground(255,255,255);
    // viz.showCloud( _pcl_data, "row_cloud" );

    // viz.setViewerPosition(0,0,80,-1,0,0);
    // viz.spingUntilDeath();

    return 0;
}


template<typename T>
void back_project(T params, Eigen::Vector2d& uv){

    // float r_sq = uv(0)*uv(0) + uv(1)*uv(1);

    // Eigen::Vector2d uv_ud;

    // uv_ud(0) = (1 + params.r_dist_coeffs(0) * r_sq + params.r_dist_coeffs(1) * r_sq * r_sq + params.r_dist_coeffs(2) * r_sq * r_sq * r_sq) * uv(0) +
    //            2 * params.t_dist_coeffs(0) * uv(0) * uv(1) + params.t_dist_coeffs(1) * ( r_sq + 2 * uv(0) * uv(0) );

    // uv_ud(1) = (1 + params.r_dist_coeffs(0) * r_sq + params.r_dist_coeffs(1) * r_sq * r_sq + params.r_dist_coeffs(2) * r_sq * r_sq * r_sq) * uv(1) +
    //            2 * params.t_dist_coeffs(1) * uv(0) * uv(1) + params.t_dist_coeffs(0) * ( r_sq + 2 * uv(1) * uv(1) );

    // uv(0) = uv_ud(0) * params.K(0,0) + params.K(0,2);
    // uv(1) = - uv_ud(1) * params.K(1,1) + params.K(1,2); 

    uv(0) = uv(0) * params.K(0,0) + params.K(0,2);
    uv(1) = - uv(1) * params.K(1,1) + params.K(1,2); 

}