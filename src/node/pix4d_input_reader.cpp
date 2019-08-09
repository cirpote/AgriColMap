#include "pix4d_input_reader.h"

using namespace std;

pix4dInputReader::pix4dInputReader(string& extrnscs_file_path, 
                                   string& intrnscs_file_path, 
                                   string& cam_id_file_path) 
                                   : input_reader_( new InputFileReader(extrnscs_file_path, intrnscs_file_path, cam_id_file_path) ){}

pix4dInputReader::~pix4dInputReader() {}


void pix4dInputReader::readMSPFile(){

    std::cout << FBLU("Reading Camera-ID Params File ...");
    input_reader_->readCamIdParams(cam_id_map_);
    std::cout << FBLU("Reading Camera-ID Params Done! \n\n");
    std::cout << FBLU("Reading Instrisic Params File ");
    input_reader_->readInstrinsicParams(intrinsics_calib_params_, cam_id_map_);
    std::cout << FBLU("Reading Instrisic Params Done! \n\n");
    std::cout << FBLU("Reading Extrinsic Params File ");
    input_reader_->readExtrinsicParams(MSP_params_, imgs_);
    std::cout << FBLU("Reading Extrinsic Params Done! \n\n");
}


void pix4dInputReader::printCameraIntrinsic(string&& idx_str){

    std::cout << FGRN( "Intrinsic params for " ) << idx_str << FGRN(" camera:\n");
    intrinsics_calib_params_.at(idx_str).print();

}

void pix4dInputReader::printCameraExtrinsic(string&& idx_str, int&& i){

    if( i > MSP_params_.count("GRE") )
        FRED("Param index out of bounds!");

    std::cout << FGRN( "Extrinsic params for " ) << idx_str << FGRN(" camera:\n");
    if( !strcmp( idx_str.c_str(), "NIR" ) ){ 
        std::multimap<string,string>::iterator it = imgs_.find(idx_str);
        std::multimap<string,MspCalibCamParams>::iterator it_msp = MSP_params_.find("NIR");
        advance(it, i);
        advance(it_msp, i);
        std::cout << "Image name: " << it->second << "\n\n";
        it_msp->second.print();
    } else if( !strcmp( idx_str.c_str(), "RED" ) ){ 
        std::multimap<string,string>::iterator it = imgs_.find(idx_str);
        std::multimap<string,MspCalibCamParams>::iterator it_msp = MSP_params_.find("RED");
        advance(it, i);
        advance(it_msp, i);
        std::cout << "Image name: " << it->second << "\n\n";
        it_msp->second.print();
    } else if( !strcmp( idx_str.c_str(), "REG" ) ){
        std::multimap<string,string>::iterator it = imgs_.find(idx_str);
        std::multimap<string,MspCalibCamParams>::iterator it_msp = MSP_params_.find("REG");
        advance(it, i);
        advance(it_msp, i);
        std::cout << "Image name: " << it->second << "\n\n";
        it_msp->second.print();
    } else if( !strcmp( idx_str.c_str(), "GRE" ) ){ 
        std::multimap<string,string>::iterator it = imgs_.find(idx_str);
        std::multimap<string,MspCalibCamParams>::iterator it_msp = MSP_params_.find("GRE");
        advance(it, i);
        advance(it_msp, i);
        std::cout << "Image name: " << it->second << "\n\n";
        it_msp->second.print();
    }    
}

void pix4dInputReader::printCameraID(string&& idx_str){

    std::cout << FGRN( "Camera for ID " ) << idx_str << FGRN( " ====> " ) << cam_id_map_.at(idx_str) << "\n\n";
}