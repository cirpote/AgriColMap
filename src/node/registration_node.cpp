#include "pix4d_input_reader.h"

using namespace std;



int main(int argc, char **argv) {
    // Pix4d Calib Parameters
    string input_calibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_camera_parameters" + ".txt";
    string input_extcalibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_external_camera_parameters" + ".txt";

    // Pix4d output PointCloud 
    string input_pcl_str = _package_path + "/src/node/08may2019_jesi_bis/2_densification/point_cloud/08may2019_jesi_bis_group1_densified_point_cloud.ply";

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

    /*pix4dReader.printMultiSpectralParams();
    pix4dReader.printStereoParams();*/

    std::shared_ptr<open3d::PointCloud> cloud_ptr ( new open3d::PointCloud() );
    if (!ReadPointCloud(input_pcl_str, *cloud_ptr)) 
        cerr << FGRN("Failed to read: ") << input_pcl_str << "\n";
    
    int id_prova = 91;
    pix4dReader.printParams(id_prova);
    CalibCamParams&& CalibData = pix4dReader.getCalibData(id_prova);

    cv::Mat output_img( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC3, cv::Scalar(0,0,0) );

    for( unsigned int iter = 0; iter < cloud_ptr->points_.size(); ++iter){
        
        Eigen::Vector3d& pt = cloud_ptr->points_[iter]; 
        Eigen::Vector3d& pt_color = cloud_ptr->colors_[iter];
        Eigen::Vector3d cam_pt = CalibData.cam_R.transpose()*( pt - CalibData.cam_t );
        //std::cout << cam_pt.transpose() << "\n";
        Eigen::Vector2d uv_pt;

        uv_pt(0) = -cam_pt(0)/cam_pt(2)*CalibData.K(0,0) + CalibData.K(0,2);
        uv_pt(1) = -cam_pt(1)/cam_pt(2)*CalibData.K(1,1) + CalibData.K(1,2);

        if( uv_pt(0) > 0 && uv_pt(0) < CalibData.img_width && uv_pt(1) > 0 && uv_pt(1) < CalibData.img_height ){
            //cout << pt_color.transpose() * 255 << "\n";
            output_img.at<cv::Vec3b>( uv_pt(0), uv_pt(1) )[0] = pt_color(0)*255.f;
            output_img.at<cv::Vec3b>( uv_pt(0), uv_pt(1) )[1] = pt_color(1)*255.f;
            output_img.at<cv::Vec3b>( uv_pt(0), uv_pt(1) )[2] = pt_color(2)*255.f;
        }

    }

    cv::imwrite("/home/ciro/AgriColMap/output.jpg", output_img);


    return 0;
}
