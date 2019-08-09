#include "file_reader.h"

using namespace std;

InputFileReader::InputFileReader(string& extrnscs_file_path, 
                                   string& intrnscs_file_path, 
                                   string& cam_id_file_path) 
                                   : instream_( new ifstream(cam_id_file_path) ), strstream_( new istringstream() ),
                                   extrnscs_params_file_path_(extrnscs_file_path), 
                                   intrnscs_params_file_path_(intrnscs_file_path),
                                   cam_id_map_file_path_(cam_id_file_path) {

    std::cout << FBLU("\nextrnscs_params_file_path_: " ) << extrnscs_params_file_path_ <<"\n";
    std::cout << FBLU("intrnscs_params_file_path_: " ) << intrnscs_params_file_path_ <<"\n";
    std::cout << FBLU("cam_id_map_file_path_: " ) << cam_id_map_file_path_ <<"\n";


    if( !boost::filesystem::exists(extrnscs_params_file_path_) || 
        !boost::filesystem::exists(intrnscs_params_file_path_) ||
        !boost::filesystem::exists(cam_id_map_file_path_) )
        std::cout << FRED("One or More input files donot exist ...") << "\n";

}

InputFileReader::~InputFileReader() {}


bool InputFileReader::getImgsSizePose(MspCalibCamParams& params){

    split(curr_line_, line_chunks, ' ');

    string& str = line_chunks[0];
    int str_size = str.size();
    str.copy( params.img, str_size, 0);

    ifstreamToScalar(1, params.img_width);
    ifstreamToScalar(2, params.img_height);

    getline(*instream_, curr_line_);
    ifstreamToVector( params.cam_t.transpose() );

    getline(*instream_, curr_line_);
    ifstreamToVector( params.cam_R.row(0) );

    getline(*instream_, curr_line_);
    ifstreamToVector( params.cam_R.row(1) );

    getline(*instream_, curr_line_);
    ifstreamToVector( params.cam_R.row(2) );
}



void InputFileReader::readCamIdParams(unordered_map<std::string, std::string>& cam_id_map_){

    while ( getline( *instream_, curr_line_)){
        if( curr_line_.find("NIR") != string::npos ){
            split(curr_line_, line_chunks, ' ');
            cam_id_map_.insert( pair<string,string>(line_chunks[0], "NIR") );
        } else if(curr_line_.find("Green") != string::npos ){
            split(curr_line_, line_chunks, ' ');
            cam_id_map_.insert( pair<string,string>(line_chunks[0], "GRE") );
        } else if(curr_line_.find("Red edge") != string::npos ){
            split(curr_line_, line_chunks, ' ');
            cam_id_map_.insert( pair<string,string>(line_chunks[0], "REG") );
        } else if(curr_line_.find("Red") != string::npos ){
            split(curr_line_, line_chunks, ' ');
            cam_id_map_.insert( pair<string,string>(line_chunks[0], "RED") );
        }
    }
    instream_->close();
}

void InputFileReader::readInstrinsicParams(unordered_map<string, MspIntrnscsParams>& intrinsics_calib_params_,
                                           unordered_map<std::string, std::string>& cam_id_map_){

    instream_->open(intrnscs_params_file_path_);
    while ( getline( *instream_, curr_line_)){
        if( curr_line_.find("Pix4D camera calibration file") != string::npos ){
            split(curr_line_, line_chunks, ' ');
            if( !strcmp( line_chunks[4].c_str(), "0" ) )
                readCameraInstrinsic(cam_id_map_.at("0"), intrinsics_calib_params_);
            else if( !strcmp( line_chunks[4].c_str(), "1" ) )
                readCameraInstrinsic(cam_id_map_.at("1"), intrinsics_calib_params_);
            else if( !strcmp( line_chunks[4].c_str(), "2" ) )
                readCameraInstrinsic(cam_id_map_.at("2"), intrinsics_calib_params_);
            else if( !strcmp( line_chunks[4].c_str(), "3" ) )
                readCameraInstrinsic(cam_id_map_.at("3"),intrinsics_calib_params_);
        }

    }
    instream_->close();
}

void InputFileReader::readCameraInstrinsic(string& str, unordered_map<string, MspIntrnscsParams>& intrinsics_calib_params_){
    MspIntrnscsParams curr_prms;
    moveNlineAhead(5);
    split(curr_line_, line_chunks, ' ');
    ifstreamToScalar(1, curr_prms.px);
    getline( *instream_, curr_line_);
    split(curr_line_, line_chunks, ' ');
    ifstreamToScalar(1, curr_prms.py);    
    moveNlineAhead(2);
    split(curr_line_, line_chunks, ' ');
    ifstreamToScalar(1, curr_prms.Aff);
    moveNlineAhead(2);
    split(curr_line_, line_chunks, ' ');
    ifstreamToScalar(1, curr_prms.p1);
    ifstreamToScalar(1, curr_prms.p2);
    intrinsics_calib_params_.insert( pair<string,MspIntrnscsParams>(str,curr_prms) );
}

void InputFileReader::readExtrinsicParams(multimap<string, MspCalibCamParams>& MSP_params,
                                          multimap<string, string>& imgs){

    instream_->open(extrnscs_params_file_path_);
    while ( getline( *instream_, curr_line_)){
        MspCalibCamParams params;
        if( curr_line_.find("GRE") != string::npos ){
            getImgsSizePose(params);
            imgs.insert(std::make_pair("GRE", params.img));
            MSP_params.insert( std::make_pair("GRE", params) );
        }  else if(curr_line_.find("NIR") != string::npos ){
            getImgsSizePose(params);
            imgs.insert(std::make_pair("NIR", params.img));
            MSP_params.insert( std::make_pair("NIR", params) );
        } else if(curr_line_.find("RED") != string::npos ){
            getImgsSizePose(params);
            imgs.insert(std::make_pair("RED", params.img));
            MSP_params.insert( std::make_pair("RED", params) );
        } else if(curr_line_.find("REG") != string::npos ){
            getImgsSizePose(params);
            imgs.insert(std::make_pair("REG", params.img));
            MSP_params.insert( std::make_pair("REG", params) );
        }

    }
    instream_->close();
}



template<typename T>
void InputFileReader::ifstreamToScalar(int idx, T& dest){
    strstream_->str(line_chunks[idx]);
    *strstream_ >> dest;
    strstream_->clear();
}

template<typename T>
void InputFileReader::ifstreamToVector(T&& dest){
    strstream_->str(curr_line_);
    for(unsigned int iter = 0; iter < dest.cols(); ++iter)
        *strstream_ >> dest(iter);
    strstream_->clear();
}

void InputFileReader::moveNlineAhead(int N){

    for(unsigned int iter = 0; iter < N; ++iter)
        getline(*instream_, curr_line_);
}

void InputFileReader::dump(std::ostream &out, const std::vector<std::string> &v) {

    for(size_t i = 0; i < v.size(); ++i) {
        out << '\'' << v[ i ] << '\'' << ' ';
    }
 
    out << std::endl;
}
 
size_t InputFileReader::split(const std::string &txt, std::vector<std::string> &strs, char ch){

    size_t pos = txt.find( ch );
    size_t initialPos = 0;
    strs.clear();
 
    // Decompose statement
    while( pos != std::string::npos ) {
        strs.push_back( txt.substr( initialPos, pos - initialPos ) );
        initialPos = pos + 1;
        pos = txt.find( ch, initialPos );
    }
 
    // Add the last one
    strs.push_back( txt.substr( initialPos, std::min( pos, txt.size() ) - initialPos + 1 ) );
    return strs.size();
}