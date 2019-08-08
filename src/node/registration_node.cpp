#include "pix4d_input_reader.h"

using namespace std;

struct PointXYZNGRdRg {
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_RGB;
  PCL_ADD_INTENSITY_8U;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

template<typename T>
void back_project(T params, Eigen::Vector2d& uv);

int main(int argc, char **argv) {

    // string input_pcl_str_xyz = _package_path + "/src/node/msp_cloud.ply";
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _pcl_data_MSP( new pcl::PointCloud<pcl::PointXYZRGBA>() );
    // pcl::io::loadPLYFile<pcl::PointXYZRGBA> ( input_pcl_str_xyz, *_pcl_data_MSP);

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pcl_viz( new pcl::PointCloud<pcl::PointXYZRGB>( _pcl_data_MSP->size(), 1 ) ); 

    // pcl::PointCloud<pcl::PointXYZRGB>::iterator it_cloud = _pcl_viz->begin();
    // for( pcl::PointXYZRGBA& pt : *_pcl_data_MSP){
    //     it_cloud->x = pt.x;
    //     it_cloud->y = pt.y;
    //     it_cloud->z = pt.z;
    //     it_cloud->b = pt.a;
    //     it_cloud->g = pt.a;
    //     it_cloud->r = pt.a;
    //     it_cloud++;
    // }

    // PointCloudViz viz;
    // viz.setViewerBackground(255,255,255);
    // viz.showCloud( _pcl_viz, "row_cloud" );
    // viz.setViewerPosition(0,0,80,-1,0,0);
    // viz.spingUntilDeath();

    // Eigen::Matrix3d R_rgb_nir( Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitZ()) *   // -0.02
    //                            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *  // -0.015
    //                            Eigen::AngleAxisd(0.0f, Eigen::Vector3d::UnitX()) );
    // Eigen::Vector3d t_rgb_nir( -1.0e-02, -7.5e-03, 8.0e-03 );

    string extrnscs_calibcamparams_str = _package_path + "/src/node/8may_jesi_nir/1_initial/params/8may_jesi_nir_calibrated_camera_parameters.txt";
    string cam_id_map_str = _package_path + "/src/node/8may_jesi_nir/1_initial/params/8may_jesi_nir_calibrated_rig_parameters.txt";
    string instrnscs_calibcamparams_str = _package_path + "/src/node/8may_jesi_nir/1_initial/params/8may_jesi_nir_pix4d_calibrated_internal_camera_parameters.cam";
    pix4dInputReader pix4dReader( extrnscs_calibcamparams_str, instrnscs_calibcamparams_str, cam_id_map_str );
    pix4dReader.readMSPFile();

    pix4dReader.printCameraID("0");
    pix4dReader.printCameraID("1");
    pix4dReader.printCameraID("2");
    pix4dReader.printCameraID("3");

    pix4dReader.printCameraIntrinsic("GRE");
    pix4dReader.printCameraIntrinsic("RED");
    pix4dReader.printCameraIntrinsic("REG");
    pix4dReader.printCameraIntrinsic("NIR");

    pix4dReader.printCameraExtrinsic("NIR", 0);

    // string input_pcl_str_xyz = _package_path + "/src/node/8may_jesi_nir/2_densification/point_cloud/8may_jesi_nir_NIR_densified_point_cloud.ply";
    // //PCLPointCloudXYZRGB::Ptr _pcl_data( new PCLPointCloudXYZRGB() );
    // //pcl::io::loadPLYFile<PCLptXYZRGB> ( input_pcl_str_xyz, *_pcl_data);

    // PCLPointCloudXYZRGB::Ptr _pcl_data_RED( new PCLPointCloudXYZRGB() );
    // pcl::io::loadPLYFile<PCLptXYZRGB> ( input_pcl_str_xyz, *_pcl_data_RED);
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud( new pcl::PointCloud<pcl::PointXYZRGBA>( _pcl_data_RED->size(), 1));
    // //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud( new pcl::PointCloud<pcl::PointXYZRGB>( _pcl_data_RED->size(), 1));

    // std::vector<cv::Mat> RED_imgs_( pix4dReader.RED_strs_.size(), cv::Mat( cv::Size(1280, 960), CV_16UC1, cv::Scalar(0) ) );
    // std::vector<cv::Mat> NIR_imgs_( pix4dReader.NIR_strs_.size(), cv::Mat( cv::Size(1280, 960), CV_16UC1, cv::Scalar(0) ) );
    // std::vector<cv::Mat> REG_imgs_( pix4dReader.REG_strs_.size(), cv::Mat( cv::Size(1280, 960), CV_16UC1, cv::Scalar(0) ) );
    // std::vector<cv::Mat> GRE_imgs_( pix4dReader.GRE_strs_.size(), cv::Mat( cv::Size(1280, 960), CV_16UC1, cv::Scalar(0) ) );

    // std::vector<cv::Mat> RGB_imgs_( pix4dReader.RED_strs_.size(), cv::Mat( cv::Size(4608, 3456), CV_8UC3, cv::Scalar(0, 0, 0) ) );

    // std::vector<MspCalibCamParams> RED_parms( pix4dReader.RED_strs_.size(), MspCalibCamParams());
    // std::vector<MspCalibCamParams> NIR_parms( pix4dReader.NIR_strs_.size(), MspCalibCamParams());
    // std::vector<MspCalibCamParams> REG_parms( pix4dReader.REG_strs_.size(), MspCalibCamParams());
    // std::vector<MspCalibCamParams> GRE_parms( pix4dReader.GRE_strs_.size(), MspCalibCamParams());

    // std::cout << pix4dReader.RED_strs_.size() << " " << RED_imgs_.size() << " " << RED_parms.size() << "\n";

    // int index = 0;
    // list< string >::iterator it = pix4dReader.RED_strs_.begin();
    // list< string >::iterator it_nir = pix4dReader.NIR_strs_.begin();
    // list< string >::iterator it_gre = pix4dReader.GRE_strs_.begin();
    // list< string >::iterator it_reg = pix4dReader.REG_strs_.begin();
    // for ( ; it != pix4dReader.RED_strs_.end(); ++it, ++index, ++it_nir, ++it_gre, ++it_reg ){
        
    //     std::string curr_img = *it;
    //     int str_to_replace_pos = curr_img.find("TIF", 0);
    //     curr_img.replace(str_to_replace_pos, 3, "jpg");
    //     RED_imgs_.at(index) = cv::imread(_package_path + "/src/node/8may_jesi_nir/1_initial/project_data/normalised/" + curr_img, CV_LOAD_IMAGE_UNCHANGED);
        
    //     curr_img = *it_nir;
    //     str_to_replace_pos = curr_img.find("TIF", 0);
    //     curr_img.replace(str_to_replace_pos, 3, "jpg");
    //     NIR_imgs_.at(index) = cv::imread(_package_path + "/src/node/8may_jesi_nir/1_initial/project_data/normalised/" + curr_img, CV_LOAD_IMAGE_UNCHANGED);
        
    //     curr_img = *it_reg;
    //     str_to_replace_pos = curr_img.find("TIF", 0);
    //     curr_img.replace(str_to_replace_pos, 3, "jpg");
    //     REG_imgs_.at(index) = cv::imread(_package_path + "/src/node/8may_jesi_nir/1_initial/project_data/normalised/" + curr_img, CV_LOAD_IMAGE_UNCHANGED);
        
    //     curr_img = *it_gre;
    //     str_to_replace_pos = curr_img.find("TIF", 0);
    //     curr_img.replace(str_to_replace_pos, 3, "jpg");
    //     GRE_imgs_.at(index) = cv::imread(_package_path + "/src/node/8may_jesi_nir/1_initial/project_data/normalised/" + curr_img, CV_LOAD_IMAGE_UNCHANGED);


    //     // str_to_replace_pos = curr_img.find("NIR.jpg", 0);
    //     // curr_img.replace(str_to_replace_pos, 7, "RGB.JPG");
    //     // RGB_imgs_.at(index) = cv::imread(_package_path + "/src/node/08may2019_jesi_bis/undistorted_images/" + curr_img, CV_LOAD_IMAGE_UNCHANGED);
        
    //     RED_parms.at(index) = pix4dReader.RED_params_.at( *it );
    //     NIR_parms.at(index) = pix4dReader.NIR_params_.at( *it_nir );
    //     REG_parms.at(index) = pix4dReader.REG_params_.at( *it_reg );
    //     GRE_parms.at(index) = pix4dReader.GRE_params_.at( *it_gre );
        
    //     // std::cout << curr_img << " " << GRE_imgs_.at(index).type() << "\n"; 
    //     // cv::imshow("ciao", GRE_imgs_.at(index));
    //     // cv::waitKey(10);
    // }



    // pcl::PointCloud<pcl::PointXYZRGBA>::iterator it_cloud = output_cloud->begin();
    // for( PCLptXYZRGB& pt : *_pcl_data_RED){

    //     it_cloud->x = pt.x;
    //     it_cloud->y = pt.y;
    //     it_cloud->z = pt.z;

    //     int min_red = 1000, min_reg = 1000, min_gre = 1000, min_nir = 1000;

    //     for(unsigned int iter = 0; iter < RED_parms.size(); ++iter){ 

    //         // RED
    //         Eigen::Vector3d cam_pt_red =  RED_parms.at(iter).cam_R * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - RED_parms.at(iter).cam_t); 

    //         float dist_red = sqrt( cam_pt_red(0)*cam_pt_red(0) + cam_pt_red(1)*cam_pt_red(1) );
    //         float theta = 2/M_PI * atan2(dist_red, cam_pt_red(2));
    //         float p = theta + 0.006852437*theta*theta - 0.15810*theta*theta*theta;

    //         Eigen::Vector2d xh_red( ( p*cam_pt_red(0) )/dist_red, ( p*cam_pt_red(1) )/dist_red);
    //         Eigen::Vector2d uv_pt_red;
    //         uv_pt_red(0) = 1674.2487 * xh_red(0) + 663.5946;
    //         uv_pt_red(1) = 1674.2487 * xh_red(1) + 486.5140;

    //         if( uv_pt_red(0) > 0 && uv_pt_red(0) < 1280 && uv_pt_red(1) > 0 && uv_pt_red(1) < 960 ){
    //             if( (uint8_t)RED_imgs_.at(iter).at<uchar>( uv_pt_red(1), uv_pt_red(0) ) < min_red )
    //                 min_red = (uint8_t)RED_imgs_.at(iter).at<uchar>( uv_pt_red(1), uv_pt_red(0) );

    //         }

    //         // NIR
    //         Eigen::Vector3d cam_pt_nir =  NIR_parms.at(iter).cam_R * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - NIR_parms.at(iter).cam_t); 

    //         float dist_nir = sqrt( cam_pt_nir(0)*cam_pt_nir(0) + cam_pt_nir(1)*cam_pt_nir(1) );
    //         float theta_nir = 2/M_PI * atan2(dist_nir, cam_pt_nir(2));
    //         float p_nir = theta_nir + 0.009845531*theta_nir*theta_nir - 0.1620564*theta_nir*theta_nir*theta_nir;

    //         Eigen::Vector2d xh_nir( ( p_nir*cam_pt_nir(0) )/dist_nir, ( p*cam_pt_nir(1) )/dist_nir);
    //         Eigen::Vector2d uv_pt_nir;
    //         uv_pt_nir(0) = 1675.65935 * xh_nir(0) + 668.33149;
    //         uv_pt_nir(1) = 1675.65935 * xh_nir(1) + 502.16601;

    //         if( uv_pt_nir(0) > 0 && uv_pt_nir(0) < 1280 && uv_pt_nir(1) > 0 && uv_pt_nir(1) < 960 ){
    //             if( (uint8_t)NIR_imgs_.at(iter).at<uchar>( uv_pt_nir(1), uv_pt_nir(0) ) < min_nir )
    //                 min_nir = (uint8_t)NIR_imgs_.at(iter).at<uchar>( uv_pt_nir(1), uv_pt_nir(0) );

    //         }

    //         // GRE
    //         Eigen::Vector3d cam_pt_gre =  GRE_parms.at(iter).cam_R * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - GRE_parms.at(iter).cam_t); 

    //         float dist_gre = sqrt( cam_pt_gre(0)*cam_pt_gre(0) + cam_pt_gre(1)*cam_pt_gre(1) );
    //         float theta_gre = 2/M_PI * atan2(dist_gre, cam_pt_gre(2));
    //         float p_gre = theta_gre + 0.00802037*theta_gre*theta_gre - 0.1608063*theta_gre*theta_gre*theta_gre;

    //         Eigen::Vector2d xh_gre( ( p_gre*cam_pt_gre(0) )/dist_gre, ( p_gre*cam_pt_gre(1) )/dist_gre);
    //         Eigen::Vector2d uv_pt_gre;
    //         uv_pt_gre(0) = 1667.51496 * xh_gre(0) + 666.812349;
    //         uv_pt_gre(1) = 1667.51496 * xh_gre(1) + 501.219773;

    //         if( uv_pt_gre(0) > 0 && uv_pt_gre(0) < 1280 && uv_pt_gre(1) > 0 && uv_pt_gre(1) < 960 ){
    //             if( (uint8_t)GRE_imgs_.at(iter).at<uchar>( uv_pt_gre(1), uv_pt_gre(0) ) < min_gre )
    //                 min_gre = (uint8_t)GRE_imgs_.at(iter).at<uchar>( uv_pt_gre(1), uv_pt_gre(0) );

    //         }

    //         // REG
    //         Eigen::Vector3d cam_pt_reg =  REG_parms.at(iter).cam_R * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - REG_parms.at(iter).cam_t); 

    //         float dist_reg = sqrt( cam_pt_reg(0)*cam_pt_reg(0) + cam_pt_reg(1)*cam_pt_reg(1) );
    //         float theta_reg = 2/M_PI * atan2(dist_reg, cam_pt_reg(2));
    //         float p_reg = theta_reg + 0.00439209*theta_reg*theta_reg - 0.153902*theta_reg*theta_reg*theta_reg;

    //         Eigen::Vector2d xh_reg( ( p_gre*cam_pt_reg(0) )/dist_reg, ( p_gre*cam_pt_reg(1) )/dist_reg);
    //         Eigen::Vector2d uv_pt_reg;
    //         uv_pt_reg(0) = 1684.53929 * xh_reg(0) + 655.9213;
    //         uv_pt_reg(1) = 1684.53929 * xh_reg(1) + 497.6731;

    //         if( uv_pt_reg(0) > 0 && uv_pt_reg(0) < 1280 && uv_pt_reg(1) > 0 && uv_pt_reg(1) < 960 ){
    //             if( (uint8_t)REG_imgs_.at(iter).at<uchar>( uv_pt_reg(1), uv_pt_reg(0) ) < min_reg )
    //                 min_reg = (uint8_t)REG_imgs_.at(iter).at<uchar>( uv_pt_reg(1), uv_pt_reg(0) );

    //         }

    //     }


    //     it_cloud->b = min_nir;
    //     it_cloud->g = min_gre;
    //     it_cloud->r = min_red;
    //     it_cloud->a = min_reg;
    //     it_cloud++;

    // }

    // pcl::io::savePLYFileASCII (_package_path + "/src/node/" + "msp_cloud.ply", *output_cloud);



    // PointCloudViz viz;
    // viz.setViewerBackground(255,255,255);
    // viz.showCloud( output_cloud, "row_cloud" );
    // viz.setViewerPosition(0,0,80,-1,0,0);
    // viz.spingUntilDeath();







    // int id = 83; // 112, 90, 54
    
    // MspCalibCamParams NIR_parms, RED_parms;
    // int index = 0;
    // list< string >::iterator it_red = pix4dReader.RED_strs_.begin();
    // for ( list< string >::iterator it = pix4dReader.NIR_strs_.begin(); it != pix4dReader.NIR_strs_.end(); ++it ){
    //      if(index == id){
    //         NIR_parms = pix4dReader.NIR_params_.at( *it ); 
    //         RED_parms = pix4dReader.RED_params_.at( *it_red );
    //         std::cout << *it << "\n";
    //         std::cout << *it_red << "\n";
    //      }
    //      it_red++;
    //      index++;
    // }

    // NIR_parms.print();
    // RED_parms.print();
    // std::cout << "\n\n" << R_rgb_nir << "\n\n";

    // cv::Mat output_img_nir( cv::Size(NIR_parms.img_width, NIR_parms.img_height), CV_8UC1, cv::Scalar(0) );
    // cv::Mat output_img_red( cv::Size(NIR_parms.img_width, NIR_parms.img_height), CV_8UC1, cv::Scalar(0) );
    // cv::Mat output_img_rgb( cv::Size(NIR_parms.img_width, NIR_parms.img_height), CV_8UC3, cv::Scalar(0) );


    // cv::Mat input_red_img = cv::imread( _package_path + "/IMG_190508_100100_0096_RED_DIST.png", CV_LOAD_IMAGE_GRAYSCALE);
    // cv::Mat input_rgb_img = cv::imread( _package_path + "/src/node/08may2019_jesi_bis/undistorted_images/IMG_190508_100100_0096_RGB.JPG", CV_LOAD_IMAGE_COLOR); 

    // for( PCLptXYZRGB& pt : *_pcl_data){

    //     Eigen::Vector3d cam_pt = NIR_parms.cam_R * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - NIR_parms.cam_t);  
    //     float dist = sqrt( cam_pt(0)*cam_pt(0) + cam_pt(1)*cam_pt(1) );
    //     float theta = 2/M_PI * atan2(dist, cam_pt(2));
    //     float p = theta + 0.011754218*theta*theta - 0.1532418*theta*theta*theta;
    //     Eigen::Vector2d xh( ( p*cam_pt(0) )/dist, ( p*cam_pt(1) )/dist);
    //     Eigen::Vector2d uv_pt;
    //     uv_pt(0) = 1665.8031408 * xh(0) + 668.4252991;
    //     uv_pt(1) = 1665.8031408 * xh(1) + 502.3771077;
    //     // Eigen::Vector2d uv_pt;
    //     // uv_pt(0) = 1066.66 * (cam_pt(0)/cam_pt(2)) + 668.4252991;
    //     // uv_pt(1) = 1066.66 * (cam_pt(1)/cam_pt(2)) + 502.3771077;

	//     // CODICE PER LEGGERE DA IMMAGINE NIR
    //     if( uv_pt(0) > 0 && uv_pt(0) < NIR_parms.img_width && uv_pt(1) > 0 && uv_pt(1) < NIR_parms.img_height ){
    //         output_img_nir.at<uchar>( uv_pt(1), uv_pt(0) ) = (int)pt.r;
        

    //         Eigen::Vector3d cam_pt_red =  RED_parms.cam_R * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - RED_parms.cam_t); 
    //         float dist_red = sqrt( cam_pt_red(0)*cam_pt_red(0) + cam_pt_red(1)*cam_pt_red(1) );
    //         float theta_red = 2/M_PI * atan2(dist_red, cam_pt_red(2));
    //         float p_red = theta_red + 0.00996840760527645600*theta_red*theta_red - 0.15348309752973424147*theta_red*theta_red*theta_red;

    //         Eigen::Vector2d xh_red( ( p_red*cam_pt_red(0) )/dist_red, ( p_red*cam_pt_red(1) )/dist_red);
    //         Eigen::Vector2d uv_pt_red;
    //         uv_pt_red(0) = 1667.12338109618553971814 * xh_red(0) + 663.82503813747575804882;
    //         uv_pt_red(1) = 1667.12338109618553971814 * xh_red(1) + 486.70522613753013274618;

    //         // Eigen::Vector2d uv_pt_red;
    //         // uv_pt_red(0) = 1.76*1667.12338109618553971814 / 3.1415926 * ( cam_pt_red(0) / cam_pt_red(2) ) + 663.82503813747575804882;
    //         // uv_pt_red(1) = 1.76*1667.12338109618553971814 / 3.1415926 * ( cam_pt_red(1) / cam_pt_red(2) ) + 486.70522613753013274618;

    //         if( uv_pt_red(0) > 0 && uv_pt_red(0) < NIR_parms.img_width && uv_pt_red(1) > 0 && uv_pt_red(1) < NIR_parms.img_height ){
    //             output_img_red.at<uchar>( uv_pt(1), uv_pt(0) ) = input_red_img.at<uchar>( uv_pt_red(1), uv_pt_red(0) );
    //         }

    //         Eigen::Vector3d cam_pt_rgb = R_rgb_nir.inverse() * ( NIR_parms.cam_R * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - NIR_parms.cam_t ) + t_rgb_nir );
    //         Eigen::Vector2d uv_rgb;
    //         uv_rgb(0) = 4077.51760458475973791792*( cam_pt_rgb(0)/cam_pt_rgb(2) ) + 2333.78678725349209344131;
    //         uv_rgb(1) = 4077.51760458475973791792*( cam_pt_rgb(1)/cam_pt_rgb(2) ) + 1757.52698328850738107576;

    //         if( uv_rgb(0) > 0 && uv_rgb(0) < 4608 && uv_rgb(1) > 0 && uv_rgb(1) < 3456 ){
    //             output_img_rgb.at<cv::Vec3b>(uv_pt(1), uv_pt(0))[0] = input_rgb_img.at<cv::Vec3b>( uv_rgb(1), uv_rgb(0) )[0];
    //             output_img_rgb.at<cv::Vec3b>(uv_pt(1), uv_pt(0))[1] = input_rgb_img.at<cv::Vec3b>( uv_rgb(1), uv_rgb(0) )[1];
    //             output_img_rgb.at<cv::Vec3b>(uv_pt(1), uv_pt(0))[2] = input_rgb_img.at<cv::Vec3b>( uv_rgb(1), uv_rgb(0) )[2];
            
    //         }

    //     }
    // }

    // cv::imwrite("/home/ciro/AgriColMap/back_proj_NIR.png", output_img_nir);
    // cv::imwrite("/home/ciro/AgriColMap/back_proj_RED.png", output_img_red);
    // cv::imwrite("/home/ciro/AgriColMap/back_proj_RGB.png", output_img_rgb);









    // PointCloudViz viz;
    // viz.setViewerBackground(255,255,255);
    // viz.showCloud( _pcl_data, "row_cloud" );
    // viz.setViewerPosition(0,0,80,-1,0,0);
    // viz.spingUntilDeath();










    // // Pix4d Calib Parameters
    // string input_calibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_camera_parameters" + ".txt";
    // string input_extcalibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_external_camera_parameters" + ".txt";

    // // Pix4d output PointCloud 
    // string input_pcl_str_xyz = _package_path + "/src/node/08may2019_jesi_bis/2_densification/point_cloud/08may2019_jesi_bis_group1_densified_point_cloud_half.ply";

    // // Multispectral Calib Parameters
    // string input_calibcanparams_str_nir = _package_path + "/src/node/" + "nir_calib" + ".txt";
    // string input_calibcanparams_str_gre = _package_path + "/src/node/" + "gre_calib" + ".txt";
    // string input_calibcanparams_str_red = _package_path + "/src/node/" + "red_calib" + ".txt";
    // string input_calibcanparams_str_reg = _package_path + "/src/node/" + "reg_calib" + ".txt";

    // // Stereo Calib Parameters
    // string input_calibcanparams_str_gre_nir = _package_path + "/src/node/" + "rgb_nir_stereo" + ".txt";
    // string input_calibcanparams_str_gre_red = _package_path + "/src/node/" + "gre_red_stereo" + ".txt";
    // string input_calibcanparams_str_gre_reg = _package_path + "/src/node/" + "gre_reg_stereo" + ".txt";
    // string input_calibcanparams_str_gre_rgb = _package_path + "/src/node/" + "rgb_gre_stereo" + ".txt";

    // pix4dInputReader pix4dReader(input_calibcanparams_str);
    // pix4dReader.readParamFile();
    // pix4dReader.readExtCamCalibParams(input_extcalibcanparams_str);

    // pix4dReader.readParamFileMultiSpectral(input_calibcanparams_str_nir,
    //                                        input_calibcanparams_str_gre,
    //                                        input_calibcanparams_str_red,
    //                                        input_calibcanparams_str_reg);

    // pix4dReader.readStereoParams(input_calibcanparams_str_gre_nir,
    //                              input_calibcanparams_str_gre_red,
    //                              input_calibcanparams_str_gre_reg,
    //                              input_calibcanparams_str_gre_rgb);

    // PCLPointCloudXYZRGB::Ptr _pcl_data( new PCLPointCloudXYZRGB() );
    // // pix4dReader.loadXYZPointCloud( input_pcl_str_xyz, _pcl_data );
    // // pcl::io::loadPLYFile<PCLptXYZRGB> ( input_pcl_str_xyz, *_pcl_data);
    

    // int id_prova = 54; // 112, 90, 54
    // pix4dReader.printParams(id_prova);
    // pix4dReader.nirParams_.print();
    // pix4dReader.gre_rgb_extrn_.print();
    // pix4dReader.gre_nir_extrn_.print();
    // CalibCamParams&& CalibData = pix4dReader.getCalibData(id_prova);

    // cv::Mat output_img( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC3, cv::Scalar(0,0,0) );
    // cv::Mat output_img_nir( cv::Size(CalibData.img_width, CalibData.img_height), CV_16UC1, cv::Scalar(0) );

    // int id = 0;
    // for(unsigned int id = 0; id < pix4dReader.getCalibDataSize(); ++id){
    //     CalibCamParams&& curr_id = pix4dReader.getCalibData(id);
    //     cv::Mat rgb_img = cv::imread(_package_path + "/src/node/08may2019_jesi_bis/" + curr_id.rgb_img, CV_LOAD_IMAGE_UNCHANGED);
    //     cv::Mat gray_img, res_rgb_img, rgb_ushort;
    //     cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);
    //     cv::resize(gray_img, res_rgb_img, cv::Size(1280,960));
    //     //res_rgb_img.convertTo(rgb_ushort, CV_16UC1);
    //     rgb_ushort = cv::Mat( res_rgb_img.size(), CV_16UC1, cv::Scalar(0) );
    //     for( unsigned int c = 0; c < 1280; ++c){
    //         for( unsigned int r = 0; r < 960; ++r)
    //             rgb_ushort.at<ushort>(r,c) = (ushort)res_rgb_img.at<uchar>(r,c) << 8;
    //     }

    //     cv::imwrite(_package_path + "/src/node/08may2019_jesi_bis/rgb_tif/" + curr_id.nir_img, rgb_ushort);
    // }


    // cv::Mat nir_img = cv::imread(_package_path + "/src/node/08may2019_jesi_bis/" + CalibData.nir_img, CV_LOAD_IMAGE_UNCHANGED);
    // cv::Mat gre_img = cv::imread(_package_path + "/src/node/08may2019_jesi_bis/" + CalibData.gre_img, CV_LOAD_IMAGE_UNCHANGED);

    // std::cout << "QUI\n";
    // std::cout << nir_img.type() << " " << gre_img.type() << "\n";

    // Eigen::Quaterniond q_( pix4dReader.gre_nir_extrn_.R_ );
    // float roll_ = rollFromQuaternion( q_ );
    // float pitch_ = pitchFromQuaternion( q_ );
    // float yaw_ = yawFromQuaternion( q_ );
    // std::cout << roll_ << " " << pitch_ << " " << yaw_ << "\n\n";

    // Eigen::AngleAxisd rollAngle(roll_, Eigen::Vector3d::UnitX()); //-0.0046
    // Eigen::AngleAxisd pitchAngle(pitch_, Eigen::Vector3d::UnitY()); //0.0075
    // Eigen::AngleAxisd yawAngle(yaw_, Eigen::Vector3d::UnitZ()); //-0.018
    // Eigen::Matrix3d RR( yawAngle * pitchAngle * rollAngle);

    // // pix4dReader.gre_rgb_extrn_.print();

    // for( PCLptXYZRGB& pt : *_pcl_data){

    //     // Eigen::Vector3d cam_pt = CalibData.K * CalibData.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.K * CalibData.cam_R * CalibData.cam_t;
    //     Eigen::Vector3d cam_pt = CalibData.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_R * CalibData.cam_t;
    //     Eigen::Vector2d uv_pt;
    //     uv_pt(0) = CalibData.K (0,0) * ( cam_pt(0)/cam_pt(2) ) + CalibData.K (0,2);
    //     uv_pt(1) = CalibData.K (1,1) * ( cam_pt(1)/cam_pt(2) ) + CalibData.K (1,2);

	// // CODICE PER LEGGERE DA IMMAGINE NIR
    //     if( uv_pt(0) > 0 && uv_pt(0) < CalibData.img_width && uv_pt(1) > 0 && uv_pt(1) < CalibData.img_height ){
    //         int ExG = computeExGforXYZRGBPoint(pt);
    //         output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[0] = 0;
    //         output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[1] = ExG*5;
    //         output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[2] = 0;

    //         Eigen::Vector3d cam_pt_nir = CalibData.cam_R * Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_R * CalibData.cam_t;
            
    //         cam_pt_nir = RR*cam_pt_nir + pix4dReader.gre_nir_extrn_.t_;
    //         Eigen::Vector2d uv_pt_nir;
    //         uv_pt_nir(0) = pix4dReader.nirParams_.K(0,0) * ( cam_pt_nir(0)/cam_pt_nir(2) ) + pix4dReader.nirParams_.K(0,2);
    //         uv_pt_nir(1) = pix4dReader.nirParams_.K(1,1) * ( cam_pt_nir(1)/cam_pt_nir(2) ) + pix4dReader.nirParams_.K(1,2);

    //         // cam_pt_nir = pix4dReader.gre_nir_extrn_.R_*pix4dReader.gre_rgb_extrn_.R_*cam_pt_nir + ( pix4dReader.gre_nir_extrn_.R_*pix4dReader.gre_rgb_extrn_.t_ + pix4dReader.gre_nir_extrn_.t_ );
    //         // Eigen::Vector2d uv_pt_nir;
    //         // uv_pt_nir(0) = pix4dReader.nirParams_.K(0,0) * ( cam_pt_nir(0)/cam_pt_nir(2) ) + pix4dReader.nirParams_.K(0,2);
    //         // uv_pt_nir(1) = pix4dReader.nirParams_.K(1,1) * ( cam_pt_nir(1)/cam_pt_nir(2) ) + pix4dReader.nirParams_.K(1,2);

    //         // cam_pt_nir = pix4dReader.gre_rgb_extrn_.R_*cam_pt_nir + pix4dReader.gre_rgb_extrn_.t_;
    //         // Eigen::Vector2d uv_pt_nir;
    //         // uv_pt_nir(0) = pix4dReader.greParams_.K(0,0) * ( cam_pt_nir(0)/cam_pt_nir(2) ) + pix4dReader.greParams_.K(0,2);
    //         // uv_pt_nir(1) = pix4dReader.greParams_.K(1,1) * ( cam_pt_nir(1)/cam_pt_nir(2) ) + pix4dReader.greParams_.K(1,2);

    //         if( uv_pt_nir(0) > 0 && uv_pt_nir(0) < pix4dReader.nirParams_.img_width && uv_pt_nir(1) > 0 && uv_pt_nir(1) < pix4dReader.nirParams_.img_height )
    //              output_img_nir.at<ushort>( uv_pt(1), uv_pt(0) ) = nir_img.at<ushort>( uv_pt_nir(1), uv_pt_nir(0) );

    //     }
    // }

    // cv::imwrite("/home/ciro/AgriColMap/output.png", output_img);
    // cv::imwrite("/home/ciro/AgriColMap/output_nir.png", output_img_nir);




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
