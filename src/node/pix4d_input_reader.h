#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

struct CameraParams{

    Eigen::Matrix3d K;
    int img_width, img_height;
    Eigen::Vector3d r_dist_coeffs;
    Eigen::Vector2d t_dist_coeffs;
    Eigen::Vector3d cam_t;
    Eigen::Matrix3d cam_R;

    CameraParams() : K(Eigen::Matrix3d::Identity()), img_width(0), img_height(0), r_dist_coeffs(Eigen::Vector3d::Zero()), 
                     t_dist_coeffs(Eigen::Vector2d::Zero()), cam_t(Eigen::Vector3d::Zero()), cam_R(Eigen::Matrix3d::Identity()) {}

    void print(){
    cout << "Image size: " << img_width << " " << img_height << "\n\n";
    cout << "Calibration Matrix:\n" << K << "\n\n";
    cout << "Distorsion Coefficients:\n" << r_dist_coeffs.transpose() << " " << t_dist_coeffs.transpose() <<  "\n\n";
    cout << "Camera Position:\n" << cam_t.transpose() << "\n\n";
    cout << "Camera Rotation Matrix:\n" << cam_R << "\n\n";
    }

};



struct CalibCamParams{

    char rgb_img[50], reg_img[50], gre_img[50], red_img[50], nir_img[50];
    CameraParams rgbParams, nirParams, greParams, redParams, regParams;

    CalibCamParams() : rgbParams(CameraParams()), nirParams(CameraParams()), greParams(CameraParams()),
                       redParams(CameraParams()), regParams(CameraParams()) {};
    
    void print(){
        cout << rgb_img << "\n";
        cout << "\nPrinting rgbParams: \n\n";
        rgbParams.print();
        cout << "\nPrinting nirParams: \n\n";
        nirParams.print();
        cout << "\nPrinting greParams: \n\n";
        greParams.print();
        cout << "\nPrinting redParams: \n\n";
        redParams.print();
        cout << "\nPrinting regParams: \n\n";
        regParams.print();
    }
};


class pix4dInputReader{

    public:
        pix4dInputReader(string& input_file);
        ~pix4dInputReader();
        void readParamFile();
        void printParams(int& id);
        void printParams(string& key);
        int getCalibDataSize();

        //  TODO: write these functions to read the mulstrispectral calib params
        void readParamFileNIR();
        void readParamFileGRE();
        void readParamFileRED();
        void readParamFileREG();

    private:
        bool getImgsAndSize(CalibCamParams& params);
        void getK(CalibCamParams& params);
        void getDistCoeffs(CalibCamParams& params);
        void getCamPose(CalibCamParams& params);
        size_t split(const string &txt, vector<string> &strs, char ch);
        void dump(ostream &out, const vector<string> &v);

        istringstream* strstream_;
        ifstream* instream_;
        unordered_map<string, CalibCamParams>* pix4dCalibData_;
        string curr_line_;
        list<string> imgs_;

};