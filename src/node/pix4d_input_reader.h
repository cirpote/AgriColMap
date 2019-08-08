#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;

struct MspIntrnscsParams{

    float px, py;
    float Aff;
    float p1, p2;

    MspIntrnscsParams() : px(0.f), py(0.f), Aff(0.f), p1(0.f), p2(0.f) {};
    
    void print(){
        cout.precision(10);
        cout << "Principal Point: " << px << " " << py << "\n\n";
        cout << "Affine Value: " << Aff<< "\n\n";
        cout << "Polynomial Coeffs:\n" << p1 << " " << p2 << "\n\n";
    }
};

struct MspCalibCamParams{

    char img[50];
    int img_width, img_height;
    Eigen::Vector3d cam_t;
    Eigen::Matrix3d cam_R;

    MspCalibCamParams() : cam_t(Eigen::Vector3d::Zero()), cam_R(Eigen::Matrix3d::Identity()) {};
    
    void print(){
        cout.precision(10);
        cout << "Image size: " << img_width << " " << img_height << "\n\n";
        cout << "Camera Position:\n" << cam_t.transpose() << "\n\n";
        cout << "Camera Rotation Matrix:\n" << cam_R << "\n\n";
    }
};




class pix4dInputReader{

    public:
        pix4dInputReader(string& extrnscs_file_path, string& intrnscs_file_path, string& cam_id_file_path);
        ~pix4dInputReader();
        void readMSPFile();

    private:
        void readCamIdParams();
        void readInstrinsicParams();
        void readExtrinsicParams();
        bool getImgsSizePose(MspCalibCamParams& params);
        void readCameraInstrinsic(string& str);


        template<typename T>
        void ifstreamToScalar(int idx, T& dest);
        template<typename T>
        void ifstreamToVector(T&& dest);
        void moveNlineAhead(int N);
        size_t split(const string &txt, vector<string> &strs, char ch);
        void dump(ostream &out, const vector<string> &v);

        // MSP Parameters
        unordered_map<string, MspCalibCamParams> NIR_params_, GRE_params_, RED_params_, REG_params_;
        list<string> NIR_strs_, GRE_strs_, RED_strs_, REG_strs_;
        unordered_map<string, MspIntrnscsParams> intrinsics_calib_params_;
        unordered_map<std::string, std::string> cam_id_map_;

        // Input File Path
        string extrnscs_params_file_path_, intrnscs_params_file_path_, cam_id_map_file_path_;

        // File Opener and Reader
        istringstream* strstream_;
        ifstream* instream_;

        // Miscellaneous 
        string curr_line_;
        list<string> imgs_;
        vector<string> line_chunks;

};