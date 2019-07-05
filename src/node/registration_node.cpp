#include "pix4d_input_reader.h"

using namespace std;

int main(int argc, char **argv) {

    string ciao = _package_path + "/src/node/20180524-mavic-uav-soybean-eschikon/point_cloud.ply";

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
    
    PCLPointCloudXYZRGB::Ptr _pcl_data( new PCLPointCloudXYZRGB() );
    pcl::io::loadPLYFile<PCLptXYZRGB> ( input_pcl_str, *_pcl_data);
    
    int id_prova = 112;
    pix4dReader.printParams(id_prova);
    CalibCamParams&& CalibData = pix4dReader.getCalibData(id_prova);

    cv::Mat output_img( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC3, cv::Scalar(0,0,0) );
    cv::Mat output_gre( cv::Size(pix4dReader.greParams_.img_width, pix4dReader.greParams_.img_height), CV_8UC1, 0 );

    for( PCLptXYZRGB& pt : *_pcl_data){
        Eigen::Vector3d cam_pt = CalibData.cam_R.transpose()*( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        Eigen::Vector2d uv_pt;
        uv_pt(0) = -( cam_pt(0)/cam_pt(2) ) * CalibData.K(0,0) + CalibData.K(0,2);
        uv_pt(1) = -( cam_pt(1)/cam_pt(2) ) * CalibData.K(1,1) + CalibData.K(1,2);

        /*Eigen::Vector3d cam_pt_gre = - pix4dReader.gre_rgb_extrncs_.t_ + CalibData.cam_R.transpose()*( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        Eigen::Vector2d uv_pt_gre;
        uv_pt_gre(0) = -( uv_pt_gre(0)/uv_pt_gre(2) ) * pix4dReader.greParams_.K(0,0) + pix4dReader.greParams_.K(0,2);
        uv_pt_gre(1) = -( uv_pt_gre(1)/uv_pt_gre(2) ) * pix4dReader.greParams_.K(1,1) + pix4dReader.greParams_.K(1,2);*/

        if( uv_pt(0) > 0 && uv_pt(0) < pix4dReader.greParams_.img_width && uv_pt(1) > 0 && uv_pt(1) < pix4dReader.greParams_.img_height ){
            int ExG = computeExGforXYZRGBPoint(pt);
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[0] = pt.b;//0;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[1] = pt.g;//ExG*5;//(int)pt.g;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[2] = pt.r;//0;//(int)pt.r;
        }

    }


    cv::imwrite("/home/ciro/AgriColMap/output.png", output_img);

    PointCloudViz viz;
    viz.setViewerBackground(255,255,255);
    viz.showCloud( _pcl_data, "row_cloud" );

    viz.setViewerPosition(0,0,80,-1,0,0);
    viz.spingUntilDeath();

    return 0;
}
