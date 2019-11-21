#include "file_reader.h"

using namespace std;


class pix4dInputReader{

    public:
        pix4dInputReader(string& extrnscs_file_path, string& intrnscs_file_path, string& cam_id_file_path);
        ~pix4dInputReader();
        void readMSPFile();

        void printCameraIntrinsic(string&&);
        void printCameraExtrinsic(string&&, int&&);
        void printCameraID(string&&);

    //private:

        // MSP Parameters
        // unordered_map<string, MspCalibCamParams> NIR_params_, GRE_params_, RED_params_, REG_params_;
        // list<string> NIR_strs_, GRE_strs_, RED_strs_, REG_strs_;
        multimap<string,string> imgs_;
        multimap<string, MspCalibCamParams> MSP_params_;
        unordered_map<string, MspIntrnscsParams> intrinsics_calib_params_;
        unordered_map<std::string, std::string> cam_id_map_;

        shared_ptr<InputFileReader> input_reader_;


};