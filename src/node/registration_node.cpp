#include "pix4d_input_reader.h"

using namespace std;

template<typename T>
void back_project(T params, Eigen::Vector2d& uv);

int main(int argc, char **argv) {

    // Pix4d Calib Parameters
    string input_calibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_camera_parameters" + ".txt";
    string input_extcalibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_external_camera_parameters" + ".txt";

    // Pix4d output PointCloud 
    string input_pcl_str_xyz = _package_path + "/src/node/08may2019_jesi_bis/2_densification/point_cloud/08may2019_jesi_bis_group1_densified_point_cloud.ply";

    // Multispectral Calib Parameters
    string input_calibcanparams_str_nir = _package_path + "/src/node/" + "nir_calib" + ".txt";
    string input_calibcanparams_str_gre = _package_path + "/src/node/" + "gre_calib" + ".txt";
    string input_calibcanparams_str_red = _package_path + "/src/node/" + "red_calib" + ".txt";
    string input_calibcanparams_str_reg = _package_path + "/src/node/" + "reg_calib" + ".txt";

    // Stereo Calib Parameters
    string input_calibcanparams_str_gre_nir = _package_path + "/src/node/" + "gre_nir_stereo" + ".txt";
    string input_calibcanparams_str_gre_red = _package_path + "/src/node/" + "gre_red_stereo" + ".txt";
    string input_calibcanparams_str_gre_reg = _package_path + "/src/node/" + "gre_reg_stereo" + ".txt";
    string input_calibcanparams_str_gre_rgb = _package_path + "/src/node/" + "rgb_gre_stereo" + ".txt";

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
    // pix4dReader.loadXYZPointCloud( input_pcl_str_xyz, _pcl_data );
    pcl::io::loadPLYFile<PCLptXYZRGB> ( input_pcl_str_xyz, *_pcl_data);
    

    int id_prova = 54; // 112, 90
    pix4dReader.printParams(id_prova);
    pix4dReader.nirParams_.print();
    pix4dReader.gre_rgb_extrn_.print();
    pix4dReader.gre_nir_extrn_.print();
    CalibCamParams&& CalibData = pix4dReader.getCalibData(id_prova);

    cv::Mat output_img( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC3, cv::Scalar(0,0,0) );
    cv::Mat output_img_nir( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC1, cv::Scalar(0) );
    cv::Mat nir_img = cv::imread(_package_path + "/src/node/08may2019_jesi_bis/" + CalibData.nir_img, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat gre_img = cv::imread(_package_path + "/src/node/08may2019_jesi_bis/" + CalibData.gre_img, CV_LOAD_IMAGE_GRAYSCALE);

            cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
            intrinsic.at<float>(0, 0) = pix4dReader.nirParams_.K(0,0); 
            intrinsic.at<float>(0, 1) = 0; 
            intrinsic.at<float>(0, 2) = pix4dReader.nirParams_.K(0,2); 
            intrinsic.at<float>(1, 0) = 0; 
            intrinsic.at<float>(1, 1) = pix4dReader.nirParams_.K(1,1); 
            intrinsic.at<float>(1, 2) = pix4dReader.nirParams_.K(1,2); 
            intrinsic.at<float>(2, 0) = 0; 
            intrinsic.at<float>(2, 1) = 0; 
            intrinsic.at<float>(2, 2) = 1; 

            cv::Mat distcoeffs = cv::Mat( 1, 5, CV_32FC1);
            distcoeffs.at<float>(0) = pix4dReader.nirParams_.r_dist_coeffs(0);
            distcoeffs.at<float>(1) = pix4dReader.nirParams_.r_dist_coeffs(1);
            distcoeffs.at<float>(2) = pix4dReader.nirParams_.t_dist_coeffs(0);
            distcoeffs.at<float>(3) = pix4dReader.nirParams_.t_dist_coeffs(1);
            distcoeffs.at<float>(4) = pix4dReader.nirParams_.r_dist_coeffs(2);

            std::cout << intrinsic << "\n\n";
            std::cout << distcoeffs << "\n";

            cv::Mat nir_img_und, newK;
            cv::undistort(nir_img, nir_img_und, intrinsic, distcoeffs, newK);

    cv::flip(nir_img, nir_img, 1);
    cv::imshow("distorted", nir_img);

    cv::flip(nir_img_und, nir_img_und, 1);
    cv::imshow("undistorted", nir_img_und);

    cv::waitKey(0);

    pix4dReader.gre_rgb_extrn_.print();

    for( PCLptXYZRGB& pt : *_pcl_data){

        // Eigen::Vector3d cam_pt = CalibData.K * CalibData.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.K * CalibData.cam_R * CalibData.cam_t;
        Eigen::Vector3d cam_pt = CalibData.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_R * CalibData.cam_t;
        Eigen::Vector2d uv_pt;
        uv_pt(0) = CalibData.K (0,0) * ( cam_pt(0)/cam_pt(2) ) + CalibData.K (0,2);
        uv_pt(1) = CalibData.K (1,1) * ( cam_pt(1)/cam_pt(2) ) + CalibData.K (1,2);

	// CODICE PER LEGGERE DA IMMAGINE NIR
        if( uv_pt(0) > 0 && uv_pt(0) < CalibData.img_width && uv_pt(1) > 0 && uv_pt(1) < CalibData.img_height ){
            int ExG = computeExGforXYZRGBPoint(pt);
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[0] = 0;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[1] = ExG*5;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[2] = 0;

            Eigen::Vector3d cam_pt_nir = CalibData.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_R * CalibData.cam_t;
            // cam_pt_nir = pix4dReader.nirParams_.K*pix4dReader.gre_nir_extrn_.R_*pix4dReader.gre_rgb_extrn_.R_.transpose()*pix4dReader.nirParams_.K.inverse()*cam_pt_nir + 
            // pix4dReader.nirParams_.K * ( - pix4dReader.gre_nir_extrn_.R_*pix4dReader.gre_rgb_extrn_.t_ + pix4dReader.gre_nir_extrn_.t_ );
            cam_pt_nir = pix4dReader.gre_nir_extrn_.R_*pix4dReader.gre_rgb_extrn_.R_*cam_pt_nir + ( pix4dReader.gre_nir_extrn_.R_*pix4dReader.gre_rgb_extrn_.t_ + pix4dReader.gre_nir_extrn_.t_ );
            Eigen::Vector2d uv_pt_nir;//(cam_pt_nir(0)/cam_pt_nir(2), cam_pt_nir(1)/cam_pt_nir(2));
            uv_pt_nir(0) = pix4dReader.nirParams_.K(0,0) * ( cam_pt_nir(0)/cam_pt_nir(2) ) + pix4dReader.nirParams_.K(0,2);
            uv_pt_nir(1) = pix4dReader.nirParams_.K(1,1) * ( cam_pt_nir(1)/cam_pt_nir(2) ) + pix4dReader.nirParams_.K(1,2);
            //back_project(pix4dReader.nirParams_, uv_pt_nir);

            int color = 0;
            if( uv_pt_nir(0) > 0 && uv_pt_nir(0) < pix4dReader.nirParams_.img_width && uv_pt_nir(1) > 0 && uv_pt_nir(1) < pix4dReader.nirParams_.img_height )
                 color = nir_img.at<uchar>( uv_pt_nir(1), uv_pt_nir(0) );

            output_img_nir.at<uchar>( uv_pt(1), uv_pt(0) ) = color;
        }
    }

    cv::imwrite("/home/ciro/AgriColMap/output.png", output_img);
    cv::imwrite("/home/ciro/AgriColMap/output_nir.png", output_img_nir);

    /*   GENERATING NIR CLOUD   */

    // cout << CalibData.Rx_ext * CalibData.Ry_ext * CalibData.Rz_ext << "\n\n";
    // cout << CalibData.Rz_ext * CalibData.Ry_ext * CalibData.Rx_ext << "\n\n";
    
    // std::vector<cv::Mat> nirImgs(pix4dReader.getCalibDataSize(), cv::Mat( cv::Size(pix4dReader.greParams_.img_width, pix4dReader.greParams_.img_height), CV_8UC1, cv::Scalar(0) ) );
    // PCLPointCloudXYZRGB::Ptr _pcl_data_gre( _pcl_data );
    // std::cout << "dimensione PCL " << _pcl_data->points.size() << "\n";
    // std::cout << "dimensione PCL " << _pcl_data_gre->points.size() << "\n";

    // for(unsigned int iter = 0; iter < pix4dReader.getCalibDataSize(); ++iter){

    //     CalibCamParams&& CalibDatatemp = pix4dReader.getCalibData(iter);
    //     // std::cout << _package_path + "/src/node/08may2019_jesi_bis/" + CalibDatatemp.nir_img << "\n";
    //     nirImgs[iter] = cv::imread( _package_path + "/src/node/08may2019_jesi_bis/" + CalibDatatemp.nir_img );
    //     cv::flip(nirImgs[iter], nirImgs[iter], 1);

    // }

    // for( PCLptXYZRGB& pt : *_pcl_data_gre){

    //     for( unsigned int iter = 0; iter < pix4dReader.getCalibDataSize(); ++iter ){

    //         CalibCamParams&& CalibDatatemp = pix4dReader.getCalibData(iter);

    //         Eigen::Vector3d cam_pt = CalibDatatemp.K * CalibDatatemp.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibDatatemp.K * CalibDatatemp.cam_R * CalibDatatemp.cam_t;
    //         Eigen::Vector2d uv_pt;
    //         uv_pt(0) = ( cam_pt(0)/cam_pt(2) );
    //         uv_pt(1) = ( cam_pt(1)/cam_pt(2) );

    //         if( uv_pt(0) > 0 && uv_pt(0) < CalibData.img_width && uv_pt(1) > 0 && uv_pt(1) < CalibData.img_height ){
    //             Eigen::Vector3d cam_pt_nir = pix4dReader.nirParams_.K * CalibDatatemp.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - pix4dReader.nirParams_.K * CalibDatatemp.cam_R * CalibDatatemp.cam_t;
    //             cam_pt_nir = cam_pt_nir + pix4dReader.nirParams_.K * ( - pix4dReader.gre_rgb_extrn_.t_ + pix4dReader.gre_nir_extrn_.t_ );
    //             Eigen::Vector2d uv_pt_nir;
    //             uv_pt_nir(0) = ( cam_pt_nir(0)/cam_pt_nir(2) );
    //             uv_pt_nir(1) = ( cam_pt_nir(1)/cam_pt_nir(2) );
    //             int color = 0;
    //             if( uv_pt_nir(0) > 0 && uv_pt_nir(0) < pix4dReader.nirParams_.img_width && uv_pt_nir(1) > 0 && uv_pt_nir(1) < pix4dReader.nirParams_.img_height ){
    //                 color = nirImgs[iter].at<uchar>( uv_pt_nir(1), uv_pt_nir(0) );
    //                 pt.r = color;
    //                 pt.g = color;
    //                 pt.b = color;
    //                 break;
    //             }

    //         }
    //     }
    // }

    // pcl::io::savePLYFileBinary( "/home/ciro/AgriColMap/greProva2.ply", *_pcl_data_gre);






    /*  BACK PROJECTION RGB  */


    // cv::Mat output_img( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC3, cv::Scalar(0,0,0) );
    // int index = 0;
    // for( PCLptXYZRGB& pt : *_pcl_data){

    //     // Eigen::Vector3d cam_pt =  CalibData.Rx_ext.transpose() * CalibData.cam_R.transpose() *  (  CalibData.Rx_ext.transpose() * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
    //     Eigen::Vector3d cam_pt = ( CalibData.Rx_ext * CalibData.Ry_ext * CalibData.Rz_ext ).transpose() * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
    //     Eigen::Vector2d uv_pt;
    //     uv_pt(0) = ( cam_pt(0)/cam_pt(2) );
    //     uv_pt(1) = ( cam_pt(1)/cam_pt(2) );
    //     back_project( CalibData, uv_pt );


    //     // Eigen::Vector3d cam_pt_gre = - pix4dReader.gre_rgb_extrncs_.t_ + CalibData.cam_R.transpose()*( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
    //     // Eigen::Vector2d uv_pt_gre( cam_pt_gre(0)/cam_pt_gre(2), cam_pt_gre(1)/cam_pt_gre(2) );
    //     // back_project( pix4dReader.greParams_, uv_pt_gre  );

    //     // if( uv_pt_gre(0) > 0 && uv_pt_gre(0) < pix4dReader.greParams_.img_width && uv_pt_gre(1) > 0 && uv_pt_gre(1) < pix4dReader.greParams_.img_height ){
    //     //     PCLptXYZRGB& GREpt = _pcl_data_gre->points[index];
    //     //     GREpt.x = pt.x;
    //     //     GREpt.y = pt.y;
    //     //     GREpt.z = pt.z;
    //     //     //std::cout << "crasho qui\n";
    //     //     int greColor = curr_gre_img.at<uchar>( uv_pt_gre(1), uv_pt_gre(0) );
    //     //     GREpt.r = greColor;
    //     //     GREpt.g = greColor;
    //     //     GREpt.b = greColor;
    //     // }
    //     //std::cout << " qui\n";

    //     if( uv_pt(0) > 0 && uv_pt(0) < CalibData.img_width && uv_pt(1) > 0 && uv_pt(1) < CalibData.img_height ){
    //         int ExG = computeExGforXYZRGBPoint(pt);
    //         output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[0] = pt.b;//0;
    //         output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[1] = pt.g;//ExG*5;//(int)pt.g;
    //         output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[2] = pt.r;//0;//(int)pt.r;
    //     }

    //     index++;
    // }

    // std::cout << "sono qui\n";
    // cv::imwrite("/home/ciro/AgriColMap/output.png", output_img);
    
    //pcl::io::savePLYFileBinary( "/home/ciro/AgriColMap/greProva2.ply", *_pcl_data);
    // PointCloudViz viz;
    // viz.setViewerBackground(255,255,255);
    // viz.showCloud( _pcl_data, "    // uv(0) = - uv(0) * params.K(0,0) + params.K(0,2);
    // uv(1) = - uv(1) * params.K(1,1) + params.K(1,2); row_cloud" );

    // viz.setViewerPosition(0,0,80,-1,0,0);
    // viz.spingUntilDeath();

    return 0;
}


template<typename T>
void back_project(T params, Eigen::Vector2d& uv){

    float r_sq = uv(0)*uv(0) + uv(1)*uv(1);

    Eigen::Vector2d uv_ud;

    uv_ud(0) = (1 + params.r_dist_coeffs(0) * r_sq + params.r_dist_coeffs(1) * r_sq * r_sq + params.r_dist_coeffs(2) * r_sq * r_sq * r_sq) * uv(0) +
               2 * params.t_dist_coeffs(0) * uv(0) * uv(1) + params.t_dist_coeffs(1) * ( r_sq + 2 * uv(0) * uv(0) );

    uv_ud(1) = (1 + params.r_dist_coeffs(0) * r_sq + params.r_dist_coeffs(1) * r_sq * r_sq + params.r_dist_coeffs(2) * r_sq * r_sq * r_sq) * uv(1) +
               2 * params.t_dist_coeffs(1) * uv(0) * uv(1) + params.t_dist_coeffs(0) * ( r_sq + 2 * uv(1) * uv(1) );

    uv(0) = uv_ud(0) * params.K(0,0) + params.K(0,2);
    uv(1) = uv_ud(1) * params.K(1,1) + params.K(1,2); 

    // uv(0) = - uv(0) * params.K(0,0) + params.K(0,2);
    // uv(1) = - uv(1) * params.K(1,1) + params.K(1,2); 

    // uv(0) = uv(0) * 3659.61 + 2310.04;
    // uv(1) = - uv(1) * 3659.61 + 1706.11;

}
