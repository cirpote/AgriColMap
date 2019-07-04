#include "pix4d_input_reader.h"

using namespace std;

pix4dInputReader::pix4dInputReader(string& input_file) : instream_( new ifstream(input_file) ),
                                                         strstream_( new istringstream() ) {

    if(!(*instream_))
        ExitWithErrorMsg("File Does Not Exist: " + input_file);
    // pix4dCalibData_ = new unordered_map<string,CalibCamParams>();

}

pix4dInputReader::~pix4dInputReader() {}

int pix4dInputReader::getCalibDataSize(){

    return pix4dCalibData_.size();
}

CalibCamParams pix4dInputReader::getCalibData(int& id){
    int index = 0;
    for ( list< string >::iterator it = imgs_.begin(); it != imgs_.end(); ++it ){
        if(index == id)
            return pix4dCalibData_.at( *it ); 
        index++;
    }
    cout << "ID: " << id << " Does not exist!" << "\n\n";
}

void pix4dInputReader::readParamFile(){

   while ( getline( *instream_, curr_line_)){
        CalibCamParams params;
        if(getImgsAndSize(params, "IMG_")){
            getK(params);
            getDistCoeffs(params);
            getCamPose(params);
            pix4dCalibData_.insert( pair<string, CalibCamParams>(params.rgb_img, params) );
        }
   }
    instream_->close();
}

void pix4dInputReader::printParams(int& id){
    
    int index = 0;
    for ( list< string >::iterator it = imgs_.begin(); it != imgs_.end(); ++it ){
        if(index == id){
            cout << "Printing Calib Data for Image: " << *it << "\n\n";
            pix4dCalibData_.at( *it ).print();
            return;
        }      
        index++;
    }
    cerr << "element with ID: " << id << "not found\n\n";
}

void pix4dInputReader::printMultiSpectralParams(){
    cout << "\n" << "NIR Params: \n\n";
    nirParams_.print();
    cout << "\n" << "GRE Params: \n\n";
    greParams_.print();
    cout << "\n" << "RED Params: \n\n";
    redParams_.print();
    cout << "\n" << "REG Params: \n\n";
    regParams_.print(); 
}

void pix4dInputReader::printStereoParams(){
    cout << "\n" << "GRE_NIR Stereo Params: \n\n";
    gre_nir_extrn_.print();
    cout << "\n" << "GRE_RED Stereo Params: \n\n";
    gre_red_extrn_.print();
    cout << "\n" << "GRE_REG Stereo Params: \n\n";
    gre_reg_extrn_.print();
    cout << "\n" << "GRE_RGB Stereo Params: \n\n";
    gre_rgb_extrncs_.print(); 
}

void pix4dInputReader::printParams(string& key){
    pix4dCalibData_.at(key).print();
}

void pix4dInputReader::getSize(MultiSpectralCalibParams& params){
    getline( *instream_, curr_line_);
    vector<string> line_chunks; 
    split(curr_line_, line_chunks, ' ');

    strstream_->str(line_chunks[0]);
    *strstream_ >> nirParams_.img_width;
    strstream_->clear();
    strstream_->str(line_chunks[1]);
    *strstream_ >> nirParams_.img_height;
    strstream_->clear(); 
}

void pix4dInputReader::readExtCamCalibParams(string& ext_params_str){

    instream_->open(ext_params_str);
    getline( *instream_, curr_line_);
    while(getline( *instream_, curr_line_)){
        vector<string> line_chunks; 
        split(curr_line_, line_chunks, ' ');
        Eigen::Matrix<double,6,1>& curr_ext_Tf = pix4dCalibData_.at(line_chunks[0]).external_cam_T; 
        strstream_->str(line_chunks[1]);
        *strstream_ >> curr_ext_Tf(0);
        strstream_->clear();
        strstream_->str(line_chunks[2]);
        *strstream_ >> curr_ext_Tf(1);
        strstream_->clear();
        strstream_->str(line_chunks[3]);
        *strstream_ >> curr_ext_Tf(2);
        strstream_->clear();
        strstream_->str(line_chunks[4]);
        *strstream_ >> curr_ext_Tf(3);
        strstream_->clear();
        strstream_->str(line_chunks[5]);
        *strstream_ >> curr_ext_Tf(4);
        strstream_->clear();
        strstream_->str(line_chunks[6]);
        *strstream_ >> curr_ext_Tf(5);  
        strstream_->clear(); 
    }
    instream_->close();
}

void pix4dInputReader::readStereoParams(string& gre_nir_str, 
                                        string& gre_red_str, 
                                        string& gre_reg_str, 
                                        string& gre_rgb_str){

    instream_->open(gre_nir_str);
    getStereoCalibParams(gre_nir_extrn_);
    instream_->close();

    instream_->open(gre_red_str);
    getStereoCalibParams(gre_red_extrn_);
    instream_->close();

    instream_->open(gre_reg_str);
    getStereoCalibParams(gre_reg_extrn_);
    instream_->close();

    instream_->open(gre_rgb_str);
    getStereoCalibParams(gre_rgb_extrncs_);
    instream_->close();
}

void pix4dInputReader::getStereoCalibParams(StereoCalibCamParams& params){

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.R_(0,0);
    *strstream_ >> params.R_(0,1);
    *strstream_ >> params.R_(0,2);
    strstream_->clear();

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.R_(1,0);
    *strstream_ >> params.R_(1,1);
    *strstream_ >> params.R_(1,2);
    strstream_->clear();

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.R_(2,0);
    *strstream_ >> params.R_(2,1);
    *strstream_ >> params.R_(2,2);
    strstream_->clear();

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.t_(0);
    *strstream_ >> params.t_(1);
    *strstream_ >> params.t_(2);
    strstream_->clear();

}

void pix4dInputReader::readParamFileMultiSpectral(string& nir_str, 
                                                  string& gre_str, 
                                                  string& red_str, 
                                                  string& reg_str){

    instream_->open(nir_str);
    getSize(nirParams_);
    getK(nirParams_);
    getDistCoeffs(nirParams_);
    instream_->close();

    instream_->open(gre_str);
    getSize(greParams_);
    getK(greParams_);
    getDistCoeffs(greParams_);
    instream_->close();

    instream_->open(red_str);
    getSize(redParams_);
    getK(redParams_);
    getDistCoeffs(redParams_);
    instream_->close();

    instream_->open(reg_str);
    getSize(regParams_);
    getK(regParams_);
    getDistCoeffs(regParams_);
    instream_->close();

}

bool pix4dInputReader::getImgsAndSize(CalibCamParams& params, const char* str){

    if( !curr_line_.find(str) ){
            vector<string> line_chunks; 
            split(curr_line_, line_chunks, ' ');

            string& str = line_chunks[0];
            int str_size = str.size();
            
            imgs_.push_back(str);
            str.copy( params.rgb_img, str_size, 0);
            str.copy( params.reg_img, str_size, 0);
            params.reg_img[23] = 'R'; params.reg_img[24] = 'E'; params.reg_img[25] = 'G';
            params.reg_img[27] = 'T'; params.reg_img[28] = 'I'; params.reg_img[29] = 'F'; params.reg_img[30] = '\0';

            str.copy( params.gre_img, str_size, 0);
            params.gre_img[23] = 'G'; params.gre_img[24] = 'R'; params.gre_img[25] = 'E';
            params.gre_img[27] = 'T'; params.gre_img[28] = 'I'; params.gre_img[29] = 'F'; params.gre_img[30] = '\0';

            str.copy( params.red_img, str_size, 0);
            params.red_img[23] = 'R'; params.red_img[24] = 'E'; params.red_img[25] = 'D';
            params.red_img[27] = 'T'; params.red_img[28] = 'I'; params.red_img[29] = 'F'; params.red_img[30] = '\0';

            str.copy( params.nir_img, str_size, 0); 
            params.nir_img[23] = 'N'; params.nir_img[24] = 'I'; params.nir_img[25] = 'R';
            params.nir_img[27] = 'T'; params.nir_img[28] = 'I'; params.nir_img[29] = 'F'; params.nir_img[30] = '\0'; 

            strstream_->str(line_chunks[1]);
            *strstream_ >> params.img_width;
            strstream_->clear();

            strstream_->str(line_chunks[2]);
            *strstream_ >> params.img_height;
            strstream_->clear();  

            return true;
    }

    return false;
}

template <typename T>
void pix4dInputReader::getK(T& params){

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.K(0,0);
    *strstream_ >> params.K(0,1);
    *strstream_ >> params.K(0,2);
    strstream_->clear();

    std::getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.K(1,0);
    *strstream_ >> params.K(1,1);
    *strstream_ >> params.K(1,2);
    strstream_->clear();

    std::getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.K(2,0);
    *strstream_ >> params.K(2,1);
    *strstream_ >> params.K(2,2);
    strstream_->clear();

}

template <typename T>
void pix4dInputReader::getDistCoeffs(T& params){

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.r_dist_coeffs(0);
    *strstream_ >> params.r_dist_coeffs(1);
    *strstream_ >> params.r_dist_coeffs(2);
    strstream_->clear();

    std::getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.t_dist_coeffs(0);
    *strstream_ >> params.t_dist_coeffs(1);
    strstream_->clear();

}

void pix4dInputReader::getCamPose(CalibCamParams& params){

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.cam_t(0);
    *strstream_ >> params.cam_t(1);
    *strstream_ >> params.cam_t(2);
    strstream_->clear();

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.cam_R(0,0);
    *strstream_ >> params.cam_R(0,1);
    *strstream_ >> params.cam_R(0,2);
    strstream_->clear();

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.cam_R(1,0);
    *strstream_ >> params.cam_R(1,1);
    *strstream_ >> params.cam_R(1,2);
    strstream_->clear();

    getline(*instream_, curr_line_);
    strstream_->str(curr_line_);
    *strstream_ >> params.cam_R(2,0);
    *strstream_ >> params.cam_R(2,1);
    *strstream_ >> params.cam_R(2,2);
    strstream_->clear();

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