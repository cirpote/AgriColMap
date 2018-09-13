#include <uav_ugv_collaboration_module/utils.hpp>

using namespace std;
using namespace pcl_utils;

namespace evaluation_tool {

    class EvaluationTool{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      public:

			EvaluationTool();
			~EvaluationTool();


			void initFromYaml(const string&);

			void ReadAndComputeInitGuess();

			void duplicatePointCloud(const string&, 
									 const string&);

			void StorePointCloud(const string&, 
								 const string&, 
								 const string&);

			void visualize();
			void visualizeAndSpin();

			void removePointCloudtoViewer(const string&);
			void addPointCloudtoViewer(const string&, 
									   const string&);

			void storeFixedPts();
			void checkFixedCloudScaleError();
			void storeMovingPts();
			void checkMovingCloudScaleError();
			Transform computeGtTransform();
			void writeTransformYAML(const string&);
			void ExGConversion(const int&, const Vector3&, const int&, const Vector3&);
			bool inline checkForScaleError(){ return _check_for_scale_error; };
			void scalePointCloud(const string&, const float&);
			void TransformingPointCloud(const Transform&, const string&);


      private:
	  		// Unordered_Map storing cloud_name (KEY) and XYZRGB point-cloud 
			unordered_map< string, const boost::shared_ptr<pcl_utils::PointCloud> > pcl_map;

			// Unordered_Map storing cloud_name (KEY) and global UTM point-cloud coordinate 
			unordered_map< string, Vector3> pcl_map_params;

			// Point click event callback params
			struct callback_args cb_args;

			// Ros Package path
			const string package_path;

			// Point-cloud Viewer
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

			// Strings and types that encode the paths and the extensions for the Point-Clouds to measure
			string _fixed_pcl_path, _fixed_pcl, _moving_pcl_path, _moving_pcl;
			int _fixed_type, _moving_type;

			// Correspondences (Points) used for aligning the fixed and the moving cloud
			std::vector<Vector3> _fixed_pts, _moving_pts, _fixed_scale_error_pts, _moving_scale_error_pts;
			float _scale; bool _check_for_scale_error;
			float _moving_scale_error, _fixed_scale_error;

			// Ground Truth Transform
			Transform deltaT;

			// ICP algorithm params
			int _max_iter_num = 10;
			bool _verbosity = false;
    };

}