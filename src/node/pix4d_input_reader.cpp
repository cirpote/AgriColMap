#include "pix4d_input_reader.h"

using namespace std;

pix4dInputReader::pix4dInputReader(string& input_file) : instream_( new ifstream(input_file) ),
                                                         strstream_( new istringstream() ) {

    if(!(*instream_))
        ExitWithErrorMsg("File Does Not Exist: " + input_file);
    pix4dCalibData_ = new unordered_map<string,CalibCamParams>();

}

pix4dInputReader::~pix4dInputReader() {}

void pix4dInputReader::readParamFile(){

   while ( getline( *instream_, curr_line_) ){
        CalibCamParams params;
        if(getImgsAndSize(params)){
            getK(params);
            getDistCoeffs(params);
            getCamPose(params);
            pix4dCalibData_->insert( pair<string, CalibCamParams>(params.rgb_img, params) );
        }
   }


}

void pix4dInputReader::printParams(int& id){
    
    int index = 0;
    for ( list< string >::iterator it = imgs_.begin(); it != imgs_.end(); ++it ){
        if(index == id){
            cout << "Printing Calib Data for Image: " << *it << "\n\n";
            pix4dCalibData_->at( *it ).print();
            return;
        }      
        index++;
    }
    cerr << "element with ID: " << id << "not found\n\n";
}

void pix4dInputReader::printParams(string& key){
    pix4dCalibData_->at(key).print();
}

bool pix4dInputReader::getImgsAndSize(CalibCamParams& params){

    if( !curr_line_.find("IMG_") ){
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

void pix4dInputReader::getK(CalibCamParams& params){

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

void pix4dInputReader::getDistCoeffs(CalibCamParams& params){

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