#include "pix4d_input_reader.h"

using namespace std;



int main(int argc, char **argv) {

    string input_calibcanparams_str = _package_path + "/src/node/" + "22may_jesi_rgb_calibrated_camera_parameters" + ".txt";
    pix4dInputReader pix4dReader(input_calibcanparams_str);
    pix4dReader.readParamFile();

    return 0;
}
