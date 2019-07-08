#include "pix4d_input_reader.h"

using namespace std;

Eigen::Vector3d offset(351785, 4817095, 102);

template<typename T>
void back_project(T params, Eigen::Vector2d& uv);

void loadXYZPointCloud( std::string& cloud_path, PCLPointCloudXYZRGB::Ptr cloud, Eigen::Matrix3d& Rx );

int main(int argc, char **argv) {

    Eigen::Matrix3d Rx, Rz, Ry;
    float x_angle = -5.887865 * (M_PI / 180);
    float z_angle = -15.888577 * (M_PI / 180);
    float y_angle = 1.024912 * (M_PI / 180);
    Rz << cos(z_angle), -sin(z_angle), 0, sin(z_angle), cos(z_angle), 0, 0, 0, 1;
    Rx << 1, 0, 0, 0, cos(x_angle),  - sin(x_angle), 0, sin(x_angle), cos(x_angle);
    Ry << cos(y_angle), 0, sin(y_angle), 0, 1, 0, -sin(y_angle), 0, cos(y_angle);


    string ciao = _package_path + "/src/node/20180524-mavic-uav-soybean-eschikon/point_cloud.ply";

    // Pix4d Calib Parameters
    string input_calibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_camera_parameters" + ".txt";
    string input_extcalibcanparams_str = _package_path + "/src/node/08may2019_jesi_bis/1_initial/params/" + "08may2019_jesi_bis_calibrated_external_camera_parameters" + ".txt";

    // Pix4d output PointCloud 
    string input_pcl_str = _package_path + "/src/node/08may2019_jesi_bis/2_densification/point_cloud/08may2019_jesi_bis_group1_densified_point_cloud2.ply";
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
    loadXYZPointCloud( input_pcl_str_xyz, _pcl_data, Rx );
    //pcl::io::loadPLYFile<PCLptXYZRGB> ( input_pcl_str, *_pcl_data);
    std::cout << "dimensione PCL " << _pcl_data->points.size() << "\n";

    PCLPointCloudXYZRGB::Ptr _pcl_data_gre( new PCLPointCloudXYZRGB( (uint32_t)_pcl_data->points.size() , (uint32_t)1, PCLptXYZRGB(0,0,0) ) );

    int id_prova = 112;
    pix4dReader.printParams(id_prova);
    CalibCamParams&& CalibData = pix4dReader.getCalibData(id_prova);

    std::cout << "qui " << "/home/ciro/AgriColMap/src/node/08may2019_jesi_bis/IMG_190508_100143_0125_GRE.TIF" << "\n";
    cv::Mat curr_gre_img = cv::imread(  "/home/ciro/AgriColMap/src/node/08may2019_jesi_bis/IMG_190508_100143_0125_GRE.TIF", CV_LOAD_IMAGE_GRAYSCALE );
    // cv::imshow("gre_img", curr_gre_img);
    // cv::waitKey(0);

    //std::cout << "qui " << pix4dReader.greParams_.img_width << " " << pix4dReader.greParams_.img_height << "\n";

    std::cout << Rx << "\n";
    std::cout << Rz << "\n";

    cv::Mat output_img( cv::Size(CalibData.img_width, CalibData.img_height), CV_8UC3, cv::Scalar(0,0,0) );

    //std::cout << "qui\n";

    int index = 0;
    for( PCLptXYZRGB& pt : *_pcl_data){

        Eigen::Vector3d cam_pt = Rx.transpose() * CalibData.cam_R.transpose() * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        //Eigen::Vector3d cam_pt =  (Ry * Rz) * ( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        Eigen::Vector2d uv_pt;
        uv_pt(0) = ( cam_pt(0)/cam_pt(2) );
        uv_pt(1) = ( cam_pt(1)/cam_pt(2) );
        back_project( CalibData, uv_pt );


        Eigen::Vector3d cam_pt_gre = - pix4dReader.gre_rgb_extrncs_.t_ + CalibData.cam_R.transpose()*( Eigen::Vector3d(pt.x, pt.y, pt.z) - CalibData.cam_t );
        Eigen::Vector2d uv_pt_gre( cam_pt_gre(0)/cam_pt_gre(2), cam_pt_gre(1)/cam_pt_gre(2) );
        back_project( pix4dReader.greParams_, uv_pt_gre  );


        if( uv_pt_gre(0) > 0 && uv_pt_gre(0) < pix4dReader.greParams_.img_width && uv_pt_gre(1) > 0 && uv_pt_gre(1) < pix4dReader.greParams_.img_height ){
            PCLptXYZRGB& GREpt = _pcl_data_gre->points[index];
            GREpt.x = pt.x;
            GREpt.y = pt.y;
            GREpt.z = pt.z;
            //std::cout << "crasho qui\n";
            int greColor = curr_gre_img.at<uchar>( uv_pt_gre(1), uv_pt_gre(0) );
            GREpt.r = greColor;
            GREpt.g = greColor;
            GREpt.b = greColor;
        }
        //std::cout << " qui\n";

        if( uv_pt(0) > 0 && uv_pt(0) < CalibData.img_width && uv_pt(1) > 0 && uv_pt(1) < CalibData.img_height ){
            int ExG = computeExGforXYZRGBPoint(pt);
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[0] = 0;//0;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[1] = ExG*5;//ExG*5;//(int)pt.g;
            output_img.at<cv::Vec3b>( uv_pt(1), uv_pt(0) )[2] = 0;//0;//(int)pt.r;
        }

        index++;
    }

    std::cout << "sono qui\n";
    
    cv::imwrite("/home/ciro/AgriColMap/output.png", output_img);
    pcl::io::savePLYFileBinary( "/home/ciro/AgriColMap/greProva2.ply", *_pcl_data);


    // PointCloudViz viz;
    // viz.setViewerBackground(255,255,255);
    // viz.showCloud( _pcl_data, "row_cloud" );

    // viz.setViewerPosition(0,0,80,-1,0,0);
    // viz.spingUntilDeath();

    return 0;
}


template<typename T>
void back_project(T params, Eigen::Vector2d& uv){

    float r_sq = uv(0)*uv(0) + uv(1)*uv(1);

    uv(0) = (1 + params.r_dist_coeffs(0) * r_sq + params.r_dist_coeffs(1) * r_sq * r_sq + params.r_dist_coeffs(2) * r_sq * r_sq * r_sq) * uv(0) +
             2 * params.t_dist_coeffs(0) * uv(0) * uv(1) + params.t_dist_coeffs(1) * ( r_sq + 2 * uv(0) * uv(0) );

    uv(1) = (1 + params.r_dist_coeffs(0) * r_sq + params.r_dist_coeffs(1) * r_sq * r_sq + params.r_dist_coeffs(2) * r_sq * r_sq * r_sq) * uv(1) +
             2 * params.t_dist_coeffs(1) * uv(0) * uv(1) + params.t_dist_coeffs(0) * ( r_sq + 2 * uv(1) * uv(1) );

    uv(0) = - uv(0) * params.K(0,0) + params.K(0,2);
    uv(1) = - uv(1) * params.K(1,1) + params.K(1,2); 
}

void loadXYZPointCloud( std::string& cloud_path, PCLPointCloudXYZRGB::Ptr cloud, Eigen::Matrix3d& Rx ){

    istringstream* strstream_( new istringstream() );
    ifstream* instream_( new ifstream(cloud_path) );

    std::string line_str;
    while(getline( *instream_, line_str)){
        vector<string> line_chunks; 
        pix4dInputReader::split(line_str, line_chunks, ' ');

        PCLptXYZRGB pt;
        Eigen::Vector3d pt3d;
        Eigen::Vector3i pt3i;
        strstream_->str(line_chunks[0]);
        *strstream_ >> pt3d(0); strstream_->clear();
        strstream_->str(line_chunks[1]);
        *strstream_ >> pt3d(1); strstream_->clear();
        strstream_->str(line_chunks[2]);
        *strstream_ >> pt3d(2); strstream_->clear();
        strstream_->str(line_chunks[3]);
        *strstream_ >> pt3i(0); strstream_->clear();
        strstream_->str(line_chunks[4]);
        *strstream_ >> pt3i(1); strstream_->clear();
        strstream_->str(line_chunks[5]);
        *strstream_ >> pt3i(2); strstream_->clear();

        pt3d -= Eigen::Vector3d(351785, 4817095, 102);
        pt3d = Rx.transpose() * pt3d;

        pt.x = pt3d(0);
        pt.y = pt3d(1);
        pt.z = pt3d(2);
        pt.r = (uint8_t)pt3i(0);
        pt.g = (uint8_t)pt3i(1);
        pt.b = (uint8_t)pt3i(2);

        // cout.precision(10);
        // cout << line_chunks[0] << " " << line_chunks[1] << "\n";
        // cout << pt3d.transpose() << " " << pt3i.transpose() << "\n";
        // cout << pt.x << " " << pt.y << " " << pt.z << " " << (int)pt.r << " " << (int)pt.g << " " << (int)pt.b << "\n";
        // std::exit(1);

        cloud->points.push_back(pt);

    }

}