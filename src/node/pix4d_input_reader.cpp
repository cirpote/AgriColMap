#include "pix4d_input_reader.h"

using namespace std;

pix4dInputReader::pix4dInputReader(string& extrnscs_file_path, 
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

pix4dInputReader::~pix4dInputReader() {}


bool pix4dInputReader::getImgsSizePose(MspCalibCamParams& params){

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

void pix4dInputReader::readMSPFile(){

    std::cout << FBLU("Reading Camera-ID Params File ") << cam_id_map_file_path_ << "\n";
    readCamIdParams();
    std::cout << FBLU("Reading Camera-ID Params Done! \n\n");
    std::cout << FBLU("Reading Instrisic Params File ") << intrnscs_params_file_path_ << "\n";
    readInstrinsicParams();
    std::cout << FBLU("Reading Instrisic Params Done! \n\n");
    std::cout << FBLU("Reading Extrinsic Params File ") << extrnscs_params_file_path_ << "\n";
    readExtrinsicParams();
    std::cout << FBLU("Reading Extrinsic Params Done! \n\n");
}

void pix4dInputReader::readCamIdParams(){

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

void pix4dInputReader::readInstrinsicParams(){
    instream_->open(intrnscs_params_file_path_);
    while ( getline( *instream_, curr_line_)){
        if( curr_line_.find("Pix4D camera calibration file") != string::npos ){
            split(curr_line_, line_chunks, ' ');
            if( !strcmp( line_chunks[4].c_str(), "0" ) )
                readCameraInstrinsic(cam_id_map_.at("0"));
            else if( !strcmp( line_chunks[4].c_str(), "1" ) )
                readCameraInstrinsic(cam_id_map_.at("1"));
            else if( !strcmp( line_chunks[4].c_str(), "2" ) )
                readCameraInstrinsic(cam_id_map_.at("2"));
            else if( !strcmp( line_chunks[4].c_str(), "3" ) )
                readCameraInstrinsic(cam_id_map_.at("3"));
        }

    }
    instream_->close();
}

void pix4dInputReader::readCameraInstrinsic(string& str){
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

void pix4dInputReader::readExtrinsicParams(){

    instream_->open(extrnscs_params_file_path_);
    while ( getline( *instream_, curr_line_)){
        MspCalibCamParams params;
        if( curr_line_.find("GRE") != string::npos ){
            getImgsSizePose(params);
            GRE_strs_.push_back(params.img);
            GRE_params_.insert( pair<string, MspCalibCamParams>(params.img, params) );
        } else if(curr_line_.find("NIR") != string::npos ){
            getImgsSizePose(params);
            NIR_strs_.push_back(params.img);
            NIR_params_.insert( pair<string, MspCalibCamParams>(params.img, params) );
        } else if(curr_line_.find("RED") != string::npos ){
            getImgsSizePose(params);
            RED_strs_.push_back(params.img);
            RED_params_.insert( pair<string, MspCalibCamParams>(params.img, params) );
        } else if(curr_line_.find("REG") != string::npos ){
            getImgsSizePose(params);
            REG_strs_.push_back(params.img);
            REG_params_.insert( pair<string, MspCalibCamParams>(params.img, params) );
        }

    }
    instream_->close();
}

void pix4dInputReader::printCameraIntrinsic(string&& idx_str){

    std::cout << FGRN( "Intrinsic params for " ) << idx_str << FGRN(" camera:\n");
    intrinsics_calib_params_.at(idx_str).print();

}

void pix4dInputReader::printCameraExtrinsic(string&& idx_str, int&& i){

    if( i > NIR_params_.size() )
        FRED("Param index out of bounds!");

    std::cout << FGRN( "Extrinsic params for " ) << idx_str << FGRN(" camera:\n");
    if( !strcmp( idx_str.c_str(), "NIR" ) ){ 
        list<string>::iterator it = NIR_strs_.begin();
        advance(it, i);
        std::cout << "Image name: " << *it << "\n\n";
        NIR_params_.at( *it ).print();
    } else if( !strcmp( idx_str.c_str(), "RED" ) ){ 
        list<string>::iterator it = RED_strs_.begin();
        advance(it, i);
        std::cout << "Image name: " << *it << "\n\n";
        RED_params_.at( *it ).print();
    } else if( !strcmp( idx_str.c_str(), "REG" ) ){
        list<string>::iterator it = REG_strs_.begin();
        advance(it, i); 
        std::cout << "Image name: " << *it << "\n\n";
        REG_params_.at( *it ).print();
    } else if( !strcmp( idx_str.c_str(), "GRE" ) ){ 
        list<string>::iterator it = GRE_strs_.begin();
        advance(it, i);
        std::cout << "Image name: " << *it << "\n\n";
        GRE_params_.at( *it ).print();
    }    
}

void pix4dInputReader::printCameraID(string&& idx_str){

    std::cout << FGRN( "Camera for ID " ) << idx_str << FGRN( " ====> " ) << cam_id_map_.at(idx_str) << "\n\n";
}

template<typename T>
void pix4dInputReader::ifstreamToScalar(int idx, T& dest){
    strstream_->str(line_chunks[idx]);
    *strstream_ >> dest;
    strstream_->clear();
}

template<typename T>
void pix4dInputReader::ifstreamToVector(T&& dest){
    strstream_->str(curr_line_);
    for(unsigned int iter = 0; iter < dest.cols(); ++iter)
        *strstream_ >> dest(iter);
    strstream_->clear();
}

void pix4dInputReader::moveNlineAhead(int N){

    for(unsigned int iter = 0; iter < N; ++iter)
        getline(*instream_, curr_line_);
}

void pix4dInputReader::dump(std::ostream &out, const std::vector<std::string> &v) {

    for(size_t i = 0; i < v.size(); ++i) {
        out << '\'' << v[ i ] << '\'' << ' ';
    }
 
    out << std::endl;
}
 
size_t pix4dInputReader::split(const std::string &txt, std::vector<std::string> &strs, char ch){

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