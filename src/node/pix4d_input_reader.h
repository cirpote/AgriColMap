#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

struct CalibCamParams{

    char rgb_img[50], reg_img[50], gre_img[50], red_img[50], nir_img[50];
    Eigen::Matrix3d K;
    int img_width, img_height;
    Eigen::Vector3d r_dist_coeffs;
    Eigen::Vector2d t_dist_coeffs;
    Eigen::Vector3d cam_t;
    Eigen::Matrix3d cam_R;

    CalibCamParams() : K(Eigen::Matrix3d::Identity()), img_width(0), img_height(0), r_dist_coeffs(Eigen::Vector3d::Zero()), 
                       t_dist_coeffs(Eigen::Vector2d::Zero()), cam_t(Eigen::Vector3d::Zero()), cam_R(Eigen::Matrix3d::Identity()) {};
    
    void print(){
        cout << "Image size: " << img_width << " " << img_height << "\n\n";
        cout << "Calibration Matrix:\n" << K << "\n\n";
        cout << "Distorsion Coefficients:\n" << r_dist_coeffs.transpose() << " " << t_dist_coeffs.transpose() <<  "\n\n";
        cout << "Camera Position:\n" << cam_t.transpose() << "\n\n";
        cout << "Camera Rotation Matrix:\n" << cam_R << "\n\n";
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
        int getCalibDataSize();

        //  TODO: write these functions to read the mulstrispectral calib params
        void readParamFileMultiSpectral(string& str);

    private:
        bool getImgsAndSize(CalibCamParams& params, const char* str);
        void getK(CalibCamParams& params);
        void getDistCoeffs(CalibCamParams& params);
        void getCamPose(CalibCamParams& params);
        size_t split(const string &txt, vector<string> &strs, char ch);
        void dump(ostream &out, const vector<string> &v);

        istringstream* strstream_;
        ifstream* instream_;
        CalibCamParams nirParams_, redParams_, greParams_, regParams_;
        unordered_map<string, CalibCamParams>* pix4dCalibData_;
        string curr_line_;
        list<string> imgs_;

};