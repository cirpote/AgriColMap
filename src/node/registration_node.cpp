#include "pix4d_input_reader.h"

using namespace std;



int main(int argc, char **argv) {

    string input_calibcanparams_str = _package_path + "/src/node/" + "22may_jesi_rgb_calibrated_camera_parameters" + ".txt";
    string input_calibcanparams_str_nir = _package_path + "/src/node/" + "22may_jesi_nir_calibrated_camera_parameters" + ".txt";
    pix4dInputReader pix4dReader(input_calibcanparams_str);
    pix4dReader.readParamFile();
    pix4dReader.readParamFileMultiSpectral(input_calibcanparams_str_nir);

    string ciao = "IMG_190522_085454_0024_RGB.JPG";
    int ciao1 = 0;
    pix4dReader.printParams(ciao1);

    pix4dReader.printMultiSpectralParams();

    return 0;
}
