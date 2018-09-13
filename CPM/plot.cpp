#include "CPM.h"
#include "OpticFlowIO.h"
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
using namespace cv;

std::vector< vector<int> > ReadMatches(const char *filename)
{

    std::ifstream FILE(filename);
    std::string line;
    std::vector< vector<int> > optical_flow;
    while (std::getline(FILE, line)){
        std::vector<int> curr_line;
        std::string offset_cpy = line;
		std::string delimiter = " ";
		size_t pos = 0;
		std::string token;
		while ((pos = offset_cpy.find(delimiter)) != std::string::npos) {
			token = offset_cpy.substr(0, pos);
			curr_line.push_back( atof(token.c_str()) );
			offset_cpy.erase(0, pos + delimiter.length());
		}
		curr_line.push_back( atof(offset_cpy.c_str()) );
        optical_flow.push_back(curr_line);
    }

    FILE.close ();
    return optical_flow;
}

int main(int argc, char** argv)
{
	/*if (argc < 4){
		printf("USAGE: CPM image1 image2 inMatchText\n");
		return -1;
	}*/

	cv::Mat img1, img2, img1_xyz, img2_xyz;

	img1 = cv::imread("../imgs/fixed_cloud_color.png", CV_LOAD_IMAGE_UNCHANGED);
	img2 = cv::imread("../imgs/moving_cloud_color.png", CV_LOAD_IMAGE_UNCHANGED);

	img1_xyz = cv::imread("../imgs/fixed_cloud_X.png", CV_LOAD_IMAGE_UNCHANGED);
	//img2_xyz = cv::imread("../imgs/moving_cloud_XYZ.png", CV_LOAD_IMAGE_UNCHANGED);

	//cv::cvtColor(img1, img1, CV_GRAY2RGB);
	//cv::cvtColor(img2, img2, CV_GRAY2RGB);

    std::cout << "Reading Input Text File: " << "../imgs/output_rgb" << "\n";
    std::vector< vector<int> > optical_flow = ReadMatches("../imgs/output_rgb");

	std::cout << "matches size: " << optical_flow.size() << "\n";

    cv::Mat drawImg( cv::Size(2000,1000), CV_8UC3, cv::Scalar(0,0,0) );
    img1.copyTo( drawImg( cv::Rect(0, 0, 1000, 1000) )  );
    img2.copyTo( drawImg( cv::Rect(1000, 0, 1000, 1000) )  );


	std::vector<Vector3f> voting_vector;
	float u1 = optical_flow[0][0];
	float v1 = optical_flow[0][1];
	float u2 = optical_flow[0][2];
	float v2 = optical_flow[0][3];
	voting_vector.push_back( Vector3f( u2-u1, v2-v1, 1 ) );

	for( unsigned int i = 1; i < optical_flow.size(); ++i ) {

		u1 = optical_flow[i][0];
		v1 = optical_flow[i][1];
		u2 = optical_flow[i][2];
		v2 = optical_flow[i][3];
		Vector3f currFlow(u2-u1, v2-v1, 1);

		int size = voting_vector.size();
		for(unsigned int ii = 0; ii < size; ++ii){

			if( ( currFlow.head(2) - voting_vector[ii].head(2) ).norm() < 15){
				voting_vector[ii](2) += 1;
				break;
			} else {
				voting_vector.push_back(currFlow);
				break;
			}

		}
	}

	float max = 0;
	int max_index;

	for(unsigned int i = 0; i <  voting_vector.size(); ++i){
		if( voting_vector[i](2) > max ){
			max = voting_vector[i](2);
			max_index = i;
		}
	}

	bool _isValid[optical_flow.size()];
	for( unsigned int i = 1; i < optical_flow.size(); ++i ) {
		u1 = optical_flow[i][0];
		v1 = optical_flow[i][1];
		u2 = optical_flow[i][2];
		v2 = optical_flow[i][3];
		Vector3f currFlow(u2-u1, v2-v1, 1);
		if( ( currFlow.head(2) - voting_vector[max_index].head(2) ).norm() < 15){
			_isValid[i] = true;
		} else {
			_isValid[i] = false;
		}
	}

	std::vector< vector<int> > validFlow;
	for( unsigned int iter = 0; iter < optical_flow.size(); ++iter ) {
        if( !(optical_flow[iter][0] % 1) && !(optical_flow[iter][1] % 1) && _isValid[iter] ){ 
    	    cv::line( drawImg, cv::Point(optical_flow[iter][0], optical_flow[iter][1]), cv::Point(optical_flow[iter][2]+1000, optical_flow[iter][3]), cv::Scalar(0, 255, 0));
			validFlow.push_back(optical_flow[iter]);
		}
    }

	
	/*Mat warp_mat( 2, 3, CV_32FC1 );
  	Point2f srcTri[validFlow.size()];
  	Point2f dstTri[validFlow.size()];

	for( unsigned int iter = 0; iter < validFlow.size(); ++iter ) {
		srcTri[iter] = Point2f(validFlow[iter][0], validFlow[iter][1]);
		dstTri[iter] = Point2f(validFlow[iter][2], validFlow[iter][3]);
	}

	warp_mat = getAffineTransform( srcTri, dstTri );
	std::cout << warp_mat << "\n" << "\n";*/


	std::cout << img1_xyz.type() << "\n";

	cv::Mat m2(img1_xyz.rows, img1_xyz.cols, CV_32FC1, img1_xyz.data);
	std::cout << m2.at<float>(0,0) << " "
			  << m2.at<float>(0,1) << " "
			  << m2.at<float>(0,2) << " "
			  << m2.at<float>(0,3) << " "
			  << m2.at<float>(0,4) << "\n";

    cv::imshow("optical_flow", drawImg);
    cv::waitKey(0);

	return 0;
}