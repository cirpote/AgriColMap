/*

Code of the Coarse-to-Fine PatchMatch, published at CVPR 2016 in
"Efficient Coarse-to-Fine PatchMatch for Large Displacement Optical Flow"
by Yinlin.Hu, Rui Song and Yunsong Li.

Email: huyinlin@gmail.com

Version 1.2

Copyright (C) 2016 Yinlin.Hu

Usages:

The program "cpm.exe" has been built and tested on Windows 7.

USAGE: cpm.exe img1Name img2Name outMatchName

Explanations:

The output of the program is a text file, which is in the format of "x1,y1,x2,y2"
corresponding to one match per line.

*/

#ifndef _CPM_H_
#define _CPM_H_

#include "ImagePyramid.h"
#include <unordered_map>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>

class CPM
{
public:
    CPM();
	~CPM();

    void Match2Flow(FImage& inMat, FImage& ou, FImage& ov, int w, int h);
    int Matching(FImage& img1, FImage &img1Cloud, FImage& img2, FImage &img2Cloud, FImage& outMatches);
    void VotingSchemeHough(FImage& inpMatches, FImage& outMatches, const cv::Mat &rgb, const cv::Mat &rgb2);
    void VotingScheme(FImage& inpMatches, FImage& outMatches, const cv::Mat &rgb, const cv::Mat &rgb2);
    void addFlowToAccumulator(const Eigen::Vector2f& pt, cv::Mat& acc);
	void SetStereoFlag(int needStereo);
	void SetStep(int step);
    void SetParams(int step, bool useVisualFeatures, bool useGeometricFeatures);
    void SetMatchingWeights(float vis_weight, float geom_weight);


private:
    void imDaisy(FImage& img, FImage &imgCloud, const float &cloud_ratio, UCImage& outFtImg, UCImage& outFtImg_Elev);
	void CrossCheck(IntImage& seeds, FImage& seedsFlow, FImage& seedsFlow2, IntImage& kLabel2, int* valid, float th);
    float MatchCost(FImage& img1, FImage& img2, UCImage* im1_exg, UCImage* im1_elev, UCImage *im2_exg, UCImage *im2_elev, int x1, int y1, int x2, int y2);

	// a good initialization is already stored in bestU & bestV
    int Propogate(FImagePyramid& pyd1, FImagePyramid& pyd2, UCImage* pyd1_exg, UCImage* pyd1_elev, UCImage* pyd2_exg, UCImage* pyd2_elev, int level, float* radius, int iterCnt, IntImage* pydSeeds, IntImage& neighbors, FImage* pydSeedsFlow, float* bestCosts);
    void PyramidRandomSearch(FImagePyramid& pyd1, FImagePyramid& pyd2, UCImage* im1_exg, UCImage* im1_elev, UCImage *im2_exg, UCImage *im2_elev, IntImage* pydSeeds, IntImage& neighbors, FImage* pydSeedsFlow);
    void OnePass(FImagePyramid& pyd1, FImagePyramid& pyd2, UCImage* im1_exg, UCImage* im1_elev, UCImage* im2_exg, UCImage* im2_elev, IntImage& seeds, IntImage& neighbors, FImage* pydSeedsFlow);
	void UpdateSearchRadius(IntImage& neighbors, FImage* pydSeedsFlow, int level, float* outRadius);

    // Creating Pcl from cv::Mat
    void CreateXYZCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const cv::Mat& orgCloud, cv::Mat& indexes);
    void NormalsAndFPFHEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh, const float& ratio);

	// minimum circle
	struct Point{
		double x, y;
	};
	double dist(Point a, Point b);
	Point intersection(Point u1, Point u2, Point v1, Point v2);
	Point circumcenter(Point a, Point b, Point c);
	// return the radius of the minimal circle
	float MinimalCircle(float* x, float*y, int n, float* centerX = NULL, float* centerY = NULL);

	//
    bool _useVisFeats;
    bool _useGeomFeats;
	int _step;
	int _maxIters;
	float _stopIterRatio;
	float _pydRatio;

	int _isStereo;
	int _maxDisplacement;
	float _checkThreshold;
	int _borderWidth;

	IntImage _kLabels, _kLabels2;

	FImagePyramid _pyd1;
    FImagePyramid _pyd1_cloud;
	FImagePyramid _pyd2;
    FImagePyramid _pyd2_cloud;

    UCImage* _im1_exg;
    UCImage* _im1_elev;
    UCImage* _im2_exg;
    UCImage* _im2_elev;

	FImage* _pydSeedsFlow;
	FImage* _pydSeedsFlow2;

	IntImage _seeds;
	IntImage _seeds2;
	IntImage _neighbors;
	IntImage _neighbors2;

    float _vis_weight;
    float _geom_weight;

};

#endif // _CPM_H_
