#include "../pointcloud_handler/pointcloud_aligner.h"
#include "../visualizer/visualizer.h"

using namespace std;


class InputFileReader{

    public:
        InputFileReader(string& extrnscs_file_path, string& intrnscs_file_path, string& cam_id_file_path);
        ~InputFileReader();

        void readCamIdParams(unordered_map<std::string, std::string>&);
        void readInstrinsicParams(unordered_map<string, MspIntrnscsParams>&,
                                  unordered_map<std::string, std::string>&);
        void readExtrinsicParams(multimap<string, MspCalibCamParams>&,
                                 multimap<string, string>&);

    private:
        
        bool getImgsSizePose(MspCalibCamParams&);
        void readCameraInstrinsic(string&, unordered_map<string, MspIntrnscsParams>&);

        template<typename T>
        void ifstreamToScalar(int idx, T& dest);
        template<typename T>
        void ifstreamToVector(T&& dest);
        void moveNlineAhead(int N);
        size_t split(const string &txt, vector<string> &strs, char ch);
        void dump(ostream &out, const vector<string> &v);

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