#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

struct StereoCalibCamParams{

    Eigen::Vector3d t_;
    Eigen::Matrix3d R_;

    StereoCalibCamParams() : t_( Eigen::Vector3d::Zero() ), R_( Eigen::Matrix3d::Identity() ){};

    void print(){
        cout << "Rotation Matrix:\n" << R_ << "\n\n";
        cout << "Translation:\n" << t_ <<  "\n\n";
    }
};

struct MultiSpectralCalibParams{

    Eigen::Matrix3d K;
    int img_width, img_height;
    Eigen::Vector3d r_dist_coeffs;
    Eigen::Vector2d t_dist_coeffs;

    MultiSpectralCalibParams() : K(Eigen::Matrix3d::Identity()), img_width(0), img_height(0), 
                                 r_dist_coeffs(Eigen::Vector3d::Zero()), t_dist_coeffs(Eigen::Vector2d::Zero()){};

    void print(){
        cout << "Image size: " << img_width << " " << img_height << "\n\n";
        cout << "Calibration Matrix:\n" << K << "\n\n";
        cout << "Distorsion Coefficients:\n" << r_dist_coeffs.transpose() << " " << t_dist_coeffs.transpose() <<  "\n\n";
    }
};

struct CalibCamParams{

    char rgb_img[50], reg_img[50], gre_img[50], red_img[50], nir_img[50];
    Eigen::Matrix3d K;
    int img_width, img_height;
    Eigen::Vector3d r_dist_coeffs;
    Eigen::Vector2d t_dist_coeffs;
    Eigen::Vector3d cam_t;
    Eigen::Matrix<double,6,1> external_cam_T; 
    Eigen::Matrix3d cam_R, Rx_ext, Ry_ext, Rz_ext;

    CalibCamParams() : K(Eigen::Matrix3d::Identity()), img_width(0), img_height(0), r_dist_coeffs(Eigen::Vector3d::Zero()), 
                       t_dist_coeffs(Eigen::Vector2d::Zero()), cam_t(Eigen::Vector3d::Zero()), cam_R(Eigen::Matrix3d::Identity()),
                       external_cam_T( Eigen::Matrix<double,6,1>::Zero() ), Rx_ext(Eigen::Matrix3d::Identity() ),
                       Ry_ext(Eigen::Matrix3d::Identity() ), Rz_ext(Eigen::Matrix3d::Identity() ) {};
    
    void print(){
        cout.precision(10);
        cout << "Image size: " << img_width << " " << img_height << "\n\n";
        cout << "Calibration Matrix:\n" << K << "\n\n";
        cout << "Distorsion Coefficients:\n" << r_dist_coeffs.transpose() << " " << t_dist_coeffs.transpose() <<  "\n\n";
        cout << "Camera Position:\n" << cam_t.transpose() << "\n\n";
        cout << "Camera Rotation Matrix:\n" << cam_R << "\n\n";
        cout << "Camera External Parameters:\n" << external_cam_T.transpose() << "\n\n";
    }
};


class pix4dInputReader{

    public:
        pix4dInputReader(string& input_file);
        ~pix4dInputReader();
        void readParamFile();
        void printParams(int& id);
        void printParams(string& key);
        void printMultiSpectralParams();
        void printStereoParams();
        int getCalibDataSize();
        CalibCamParams getCalibData(int& id);

        void readParamFileMultiSpectral(string& nir_str, 
                                        string& gre_str, 
                                        string& red_str, 
                                        string& reg_str);

        void readStereoParams(string& gre_nir_str, 
                              string& gre_red_str, 
                              string& gre_reg_str, 
                              string& gre_rgb_str);

        void readExtCamCalibParams(string& ext_params_str);

        void readOffset(string& offset_str);

        void loadXYZPointCloud( std::string& cloud_path, PCLPointCloudXYZRGB::Ptr cloud );

        static size_t split(const string &txt, vector<string> &strs, char ch);

        MultiSpectralCalibParams nirParams_, redParams_, greParams_, regParams_;
        StereoCalibCamParams gre_nir_extrn_, gre_red_extrn_, gre_reg_extrn_, gre_rgb_extrncs_;
        Eigen::Vector3d offset;

    private:
        bool getImgsAndSize(CalibCamParams& params, const char* str);

        void getSize(MultiSpectralCalibParams& params);
        void getStereoCalibParams(StereoCalibCamParams& params);

        template <typename T>
        void getK(T& params);

        template <typename T>
        void getDistCoeffs(T& params);

        void getCamPose(CalibCamParams& params);
        void dump(ostream &out, const vector<string> &v);

        istringstream* strstream_;
        ifstream* instream_;
        unordered_map<string, CalibCamParams> pix4dCalibData_;
        string curr_line_;
        list<string> imgs_;

};