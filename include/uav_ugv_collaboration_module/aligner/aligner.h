#include <uav_ugv_collaboration_module/descriptor/descriptor.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl_utils;

namespace aligner {

    class PointCloudAligner{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      public:
		PointCloudAligner();
		~PointCloudAligner();
		void initFromYaml(const string& yaml_file);
		void ReadPointClouds(const string&, const string&);
		void FusePointClouds(const string&, const string&);
		void StorePointCloud(const string&, const string&, const string&, const int&);
		void ExGConversion(const string&, const int&, const Vector3&);
		void PlantFixedClustering(const string&);
		void PlantMovingClustering(const string&);
		void visualize();
		void visualizeAndSpin();
		void addPointCloudtoViewer(const string&, const string&);
		void filterPointCloud( vector<string>&, vector<float>&, vector<float>&, const string&, const string& );
		void addBiasToPointCloud( const Vector3&, const string& );
		void constructKDtree(const string&);
		void TransformingPointCloud(const Transform&, const string&);
		Transform computeGtTransform(const string&);
		void computeAndApplyInitialGuess(const string&, const string&);
		void scalePointCloud(const string&, const float&);
		void computeElevationMap(const string&, const string&);
		void duplicatePointCloud(const string&, const string&);
		void computeLocalElevationMap(const string&, const string&, const string&);
		void cutFixedCloudByInitialGuessAndRay(const Eigen::Vector2f&, const float&, const string&);
		void organizedFixedPCL(const string&, const Vector3&);
		void organizedMovingPCL(const string&);
		Vector3 normalizePointClouds(const string&, const string&);

	/*void constructKDtree(const string&);
	Transform computeTransform(const string&, const int&, const int&, const string&);
	
	Vector3 computeInitialGuess(const string& fixed_cloud, const string& moving_cloud)
	*/

      private:

		struct callback_args cb_args;
		unordered_map< string, const boost::shared_ptr<PointCloud> > pcl_map;
		unordered_map< string, Vector6> pcl_map_params;
		const string package_path;
		srrg_core::KDTree<float, 3>::VectorTDVector pcl_kdtree;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

		// Strings and types that encode the paths and the extensions for the Point-Clouds to measure
		string _fixed_pcl_path, _moving_pcl_path, _fixed_pcl, _moving_pcl;
		vector<string> _fixed_pcls, _moving_pcls;
		vector<string> _fixed_pcl_keys, _moving_pcl_keys;
		int _fixed_type, _moving_type;

		std::vector<Descriptor::CentroidDescriptor> fixed_descriptors, moving_descriptors;

		// ICP algorithm params
		int _max_iter_num = 10;
		bool _verbosity = false;

		Transform _T_init_guess, _T_final;

    };

}