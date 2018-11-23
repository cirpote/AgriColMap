#include <uav_ugv_collaboration_module/utils.h>

using namespace std;

int main(int argc, char **argv) {
  
  std::string fixed_yaml_filename, moving_yaml_filename;
  if(argc < 3) {
    cerr << FRED("Params Files Expected!") << " evaluation_node_name <fixed_cloud_params_file.txt> <moving_cloud_params_file.txt>" << "\n";
    std::exit(1);
  }

  fixed_yaml_filename = argv[1];
  Matrix3 fix_R; Vector3 fix_t; Vector2 fix_rel_scale;
  ifstream fixed_params( fixed_yaml_filename );
  string fixed_params_str;
  getline(fixed_params, fixed_params_str);
  AffineTransformFromString(fixed_params_str, fix_R, fix_t, fix_rel_scale);

  cerr << "\n";
  cerr << FBLU("fixed params: ") << "\n";
  cerr << fix_R << "\n";
  cerr << fix_t.transpose() << "\n";
  cerr << fix_rel_scale.transpose() << "\n" << "\n";

  moving_yaml_filename = argv[2];
  Matrix3 mov_R; Vector3 mov_t; Vector2 mov_rel_scale;
  ifstream moving_params( moving_yaml_filename );
  string moving_params_str;
  getline(moving_params, moving_params_str);
  AffineTransformFromString(moving_params_str, mov_R, mov_t, mov_rel_scale);

  cerr << "\n";
  cerr << FBLU("moving params: ") << "\n";
  cerr << mov_R << "\n";
  cerr << mov_t.transpose() << "\n";
  cerr << mov_rel_scale.transpose() << "\n" << "\n";


  float d_scl_x = mov_rel_scale(0) / fix_rel_scale(0);
  float d_scl_y = mov_rel_scale(1) / fix_rel_scale(1);

  mov_R(0,0) *= d_scl_x; mov_R(1,0) *= d_scl_x; mov_R(2,0) *= d_scl_x;
  mov_R(0,1) *= d_scl_y; mov_R(1,1) *= d_scl_y; mov_R(2,1) *= d_scl_y;

  cerr << "\n";
  cerr << FBLU("Normalized moving Affine Matrix: ") << "\n";
  cerr << mov_R << "\n" << "\n";

  Vector3 mov_scale = getScaleFromAffineMatrix(mov_R);
  mov_R.col(0) /= mov_scale(0); mov_R.col(1) /= mov_scale(1); mov_R.col(2) /= mov_scale(2);

  Vector3 fix_scale = getScaleFromAffineMatrix(fix_R);
  fix_R.col(0) /= fix_scale(0); fix_R.col(1) /= fix_scale(1); fix_R.col(2) /= fix_scale(2);

  float angle_err = std::fmax( 0.005, computeAngle( mov_R.inverse()*fix_R ) );
  float scale_err = std::fmax( 0.005, (fix_scale - mov_scale).norm() );
  float transl_err = std::fmax( 0.005, (fix_t - mov_t).norm() );
  cerr << FBLU("Scale Err: ") << scale_err << FBLU(" Angle Err: ") << angle_err << FBLU(" Transl Err: ") << transl_err << "\n" << "\n";

  moving_yaml_filename.erase( moving_yaml_filename.end()-4, moving_yaml_filename.end());

  std::ofstream errors(moving_yaml_filename + "_errors.txt");
  errors << scale_err << " " << angle_err << " " << transl_err;
  errors.close();

  return 0;
}
