#include "pix4d_input_reader.h"

using namespace std;



int main(int argc, char **argv) {
    // Pix4d Calib Parameters
    string input_calibcanparams_str = _package_path + "/src/node/" + "22may_jesi_rgb_calibrated_camera_parameters" + ".txt";

    // Multispectral Calib Parameters
    string input_calibcanparams_str_nir = _package_path + "/src/node/" + "nir_calib" + ".txt";
    string input_calibcanparams_str_gre = _package_path + "/src/node/" + "gre_calib" + ".txt";
    string input_calibcanparams_str_red = _package_path + "/src/node/" + "red_calib" + ".txt";
    string input_calibcanparams_str_reg = _package_path + "/src/node/" + "reg_calib" + ".txt";

    // Stereo Calib Parameters
    string input_calibcanparams_str_gre_nir = _package_path + "/src/node/" + "gre_nir_stereo" + ".txt";
    string input_calibcanparams_str_gre_red = _package_path + "/src/node/" + "gre_red_stereo" + ".txt";
    string input_calibcanparams_str_gre_reg = _package_path + "/src/node/" + "gre_reg_stereo" + ".txt";
    string input_calibcanparams_str_gre_rgb = _package_path + "/src/node/" + "gre_rgb_stereo" + ".txt";

    pix4dInputReader pix4dReader(input_calibcanparams_str);
    pix4dReader.readParamFile();

    pix4dReader.readParamFileMultiSpectral(input_calibcanparams_str_nir,
                                           input_calibcanparams_str_gre,
                                           input_calibcanparams_str_red,
                                           input_calibcanparams_str_reg);

    pix4dReader.readStereoParams(input_calibcanparams_str_gre_nir,
                                 input_calibcanparams_str_gre_red,
                                 input_calibcanparams_str_gre_reg,
                                 input_calibcanparams_str_gre_rgb);

    string ciao = "IMG_190522_085454_0024_RGB.JPG";
    int ciao1 = 0;
    pix4dReader.printParams(ciao1);

    pix4dReader.printMultiSpectralParams();
    pix4dReader.printStereoParams();

    return 0;
}
