#include <uav_ugv_collaboration_module/evaluation_tool/evaluation_tool.h>

using namespace pcl_utils;
using namespace std;

namespace evaluation_tool {

    EvaluationTool::EvaluationTool() : package_path(ros::package::getPath("uav_ugv_collaboration_module")),
					    			   viewer( new pcl::visualization::PCLVisualizer ("PointCloud Viewer") ) {
    
	   viewer->setBackgroundColor (0, 0, 0);
	   viewer->addCoordinateSystem (1.0);
	   viewer->setCameraPosition(0.f, 0.f, 100.f, 0.f, 1.f, 0.f);
	   viewer->setSize(1920, 1080);
	   viewer->spinOnce(5);
	   
	   PointCloud::Ptr clicked_points_3d (new PointCloud);
	   cb_args.clicked_points_3d = clicked_points_3d;
	   cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	   viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);

    }

    EvaluationTool::~EvaluationTool(){}

	void EvaluationTool::scalePointCloud(const string& cloud_to_scale, const float& scale_factor){

		for(size_t i = 0; i < pcl_map[cloud_to_scale]->size(); ++i){
			pcl_map[cloud_to_scale]->points[i].x *= scale_factor;
			pcl_map[cloud_to_scale]->points[i].y *= scale_factor;
			pcl_map[cloud_to_scale]->points[i].z *= scale_factor;
		}
	}

	void EvaluationTool::initFromYaml(const string& yaml_file){

		YAML::Node configuration = YAML::LoadFile(yaml_file);
 
		// Loading info for fixed input cloud from YAML
		_fixed_pcl_path = configuration["input_clouds"]["cloud_fixed_path"].as<string>();
		_fixed_pcl = configuration["input_clouds"]["cloud_fixed_name"].as<string>();
		string fixed_type = configuration["input_clouds"]["cloud_fixed_type"].as<string>();

		// check for the fixed cloud type
		strcmp( fixed_type.c_str(), "PCD" ) ? _fixed_type = 1 : _fixed_type = 0;

		// Check for the fixed cloud path
		std::ifstream in_fixed_pcl( package_path + "/maps/" + _fixed_pcl_path + "/" + _fixed_pcl );
		if(!in_fixed_pcl)
			throw std::runtime_error( package_path + "/maps/" + _fixed_pcl_path + "/" + _fixed_pcl + " file does not exist!" );

		// Loading info for moving input cloud from YAML
		_moving_pcl_path = configuration["input_clouds"]["cloud_moving_path"].as<string>();
		_moving_pcl = configuration["input_clouds"]["cloud_moving_name"].as<string>();
		string moving_type = configuration["input_clouds"]["cloud_moving_type"].as<string>();

		// check for the moving cloud type
		strcmp( moving_type.c_str(), "PCD" ) ? _moving_type = 1 : _moving_type = 0;

		// Check for the moving cloud path
		std::ifstream in_moving_pcl( package_path + "/maps/" + _moving_pcl_path + "/" + _moving_pcl );
		if(!in_moving_pcl)
			throw std::runtime_error( package_path + "/maps/" + _moving_pcl_path + "/" + _moving_pcl + " file does not exist!" );

		_max_iter_num = configuration["aligner_params"]["max_iter_number"].as<int>();
		_verbosity = configuration["aligner_params"]["verbosity"].as<bool>();
		_scale = configuration["input_clouds"]["scale"].as<float>();
		_check_for_scale_error = configuration["input_clouds"]["check_for_scale_error"].as<bool>();
	}

    void EvaluationTool::addPointCloudtoViewer(const string& cloud_key, 
											   const string& id) {

	  cb_args.pointCloud = pcl_map[cloud_key];
	  pcl::visualization::PointCloudColorHandlerRGBField<Point> RGB(pcl_map[cloud_key]);
	  viewer->addPointCloud<Point> (pcl_map[cloud_key], RGB, id);
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id);
    }

	void EvaluationTool::removePointCloudtoViewer(const string& id) {

		viewer->removePointCloud(id);
		viewer->removePointCloud("clicked_points");
	}

    void EvaluationTool::ReadAndComputeInitGuess() {

		// Reading the fixed PointCloud
		pcl_map.emplace( "fixed_cloud", boost::make_shared<PointCloud>() ); 
		cerr << "Reading fixed PCL from: " << package_path + "/maps/" + _fixed_pcl_path + "/" + _fixed_pcl << "\n";
		
		if( _fixed_type == TypePCL::PLY ){
			pcl::io::loadPLYFile<Point> ( package_path + "/maps/" + _fixed_pcl_path + "/" + _fixed_pcl, *pcl_map["fixed_cloud"]);
		} else if ( _fixed_type == TypePCL::PCD ) {
			pcl::io::loadPCDFile<Point> ( package_path + "/maps/" + _fixed_pcl_path + "/" + _fixed_pcl, *pcl_map["fixed_cloud"]);
		}
		
		cerr << "Point-Cloud " << "fixed_cloud" << " correctly red, N° of points: " << pcl_map["fixed_cloud"]->size() << "\n";
		cerr << "Reading Offset Params from: " << package_path + "/maps/" + _fixed_pcl_path + "/" + "offset.xyz" << '\n';
		
		// Reading the fixed Offset
		string fixed_offset;
		ifstream fixed_offset_file( package_path + "/maps/" + _fixed_pcl_path + "/" + "offset.xyz" );
		getline(fixed_offset_file, fixed_offset);
		
		vector<float> fixed_utm_vec = vectorFromString(fixed_offset);
		Vector3 fixed_utm(fixed_utm_vec[0], fixed_utm_vec[1], fixed_utm_vec[2]);
		cerr << "fixed cloud offset: " << fixed_utm.transpose() << "\n" << "\n";
		pcl_map_params.emplace( "fixed_cloud", fixed_utm ); 
	
		// Reading the moving PointCloud
		pcl_map.emplace( "moving_cloud", boost::make_shared<PointCloud>() ); 
		cerr << "Reading moving PCL from: " << package_path + "/maps/" + _moving_pcl_path + "/" + _moving_pcl << "\n";
		
		if( _moving_type == TypePCL::PLY ){
			pcl::io::loadPLYFile<Point> ( package_path + "/maps/" + _moving_pcl_path + "/" + _moving_pcl, *pcl_map["moving_cloud"]);
		} else if ( _moving_type == TypePCL::PCD ) {
			pcl::io::loadPCDFile<Point> ( package_path + "/maps/" + _moving_pcl_path + "/" + _moving_pcl, *pcl_map["moving_cloud"]);
		}
		
		cerr << "Point-Cloud " << "moving_cloud" << " correctly red, N° of points: " << pcl_map["moving_cloud"]->size() << "\n";
		cerr << "Reading Offset Params from: " << package_path + "/maps/" + _moving_pcl_path + "/" + "offset.xyz" << '\n';
		
		// Reading the moving Offset
		string moving_offset;
		ifstream moving_offset_file( package_path + "/maps/" + _moving_pcl_path + "/" + "offset.xyz" );
		getline(moving_offset_file, moving_offset);
		
		vector<float> moving_utm_vec = vectorFromString(moving_offset);
		Vector3 moving_utm(moving_utm_vec[0], moving_utm_vec[1], moving_utm_vec[2]);
		cerr << "moving cloud offset: " << moving_utm.transpose() << "\n" << "\n";
		pcl_map_params.emplace( "moving_cloud", moving_utm ); 



        Vector3 init_guess (pcl_map_params["moving_cloud"] - pcl_map_params["fixed_cloud"] );
		cerr << "Init alignment guess: " << init_guess.transpose() << "\n" << "\n";

		Transform T = Transform::Identity();
  		T.translation() << init_guess(0), init_guess(1), init_guess(2);
  		pcl::transformPointCloud(*pcl_map["moving_cloud"], *pcl_map["moving_cloud"], T);

    }

	void EvaluationTool::TransformingPointCloud(const Transform& transform, const string& cloud_to_transform){

		pcl::transformPointCloud(*pcl_map[cloud_to_transform], *pcl_map[cloud_to_transform], transform);
	}

    void EvaluationTool::ExGConversion(const int& type_fixed, const Vector3& color_fixed, const int& type_moving, const Vector3& color_moving) {
      
		std::cerr << "Converting " << "fixed_cloud" << " Point-Clouds by ExG index ...\n";
		int prev_size = pcl_map["fixed_cloud"]->size();
		std::cerr << "Converting Point-Cloud ...\n";
		ConvertToExG(type_fixed, pcl_map["fixed_cloud"], color_fixed);
		std::cerr << "Number of filtered points: " << prev_size - pcl_map["fixed_cloud"]->size() << "\n";
		std::cerr << "Converting " << "fixed_cloud" << " PCL by ExG index, process ended!!" << "\n" << "\n";

		std::cerr << "Converting " << "moving_cloud" << " Point-Clouds by ExG index ...\n";
		prev_size = pcl_map["moving_cloud"]->size();
		std::cerr << "Converting Point-Cloud ...\n";
		ConvertToExG(type_moving, pcl_map["moving_cloud"], color_moving);
		std::cerr << "Number of filtered points: " << prev_size - pcl_map["moving_cloud"]->size() << "\n";
		std::cerr << "Converting " << "moving_cloud" << " PCL by ExG index, process ended!!" << "\n" << "\n";
	}	

    void EvaluationTool::duplicatePointCloud(const string& cloud_to_copy_key, 
										     const string& new_cloud_key) {
	
		pcl_map.emplace( new_cloud_key, boost::make_shared<PointCloud>() ); 
		*pcl_map[new_cloud_key] = *pcl_map[cloud_to_copy_key];
    }

    
    void EvaluationTool::StorePointCloud(const std::__cxx11::string& cloud_key, 
					    				 const std::__cxx11::string& cloud_path,
					   					 const std::__cxx11::string& cloud_storing_name) {
      
      pcl::io::savePLYFile(package_path + "/maps/" + cloud_path + cloud_storing_name, *pcl_map[cloud_key]);
    }

    void EvaluationTool::visualize() {
		viewer->spinOnce(5);
    }

	void EvaluationTool::visualizeAndSpin(){

		while(!viewer->wasStopped())
			viewer->spin();
	}

	void EvaluationTool::storeFixedPts(){

		for(unsigned int i = 0; i < cb_args.clicked_points_3d->points.size(); i++){
			
			_fixed_pts.push_back( Vector3(	cb_args.clicked_points_3d->points[i].x,
								  			cb_args.clicked_points_3d->points[i].y,
											cb_args.clicked_points_3d->points[i].z) );

			cerr << "Fixed Point N°: " << i << " " << _fixed_pts[i].transpose() << "\n";
    	}

		cb_args.clicked_points_3d->points.clear();
		cerr << "\n";
	}

	void EvaluationTool::checkFixedCloudScaleError(){

		_fixed_scale_error_pts.push_back ( Vector3 ( cb_args.clicked_points_3d->points[0].x,
													 cb_args.clicked_points_3d->points[0].y,
													 cb_args.clicked_points_3d->points[0].z));

		_fixed_scale_error_pts.push_back ( Vector3 ( cb_args.clicked_points_3d->points[1].x,
													 cb_args.clicked_points_3d->points[1].y,
													 cb_args.clicked_points_3d->points[1].z));

		float real_scale = ( _fixed_scale_error_pts[0] - _fixed_scale_error_pts[1]).norm();
		_fixed_scale_error = _scale / real_scale;
		cerr << " Scale Error: " << _fixed_scale_error << "\n";
		for(size_t i = 0; i < pcl_map["fixed_cloud"]->size(); ++i){

			(*pcl_map["fixed_cloud"]).points[i].x *= _fixed_scale_error;
			(*pcl_map["fixed_cloud"]).points[i].y *= _fixed_scale_error;
			(*pcl_map["fixed_cloud"]).points[i].z *= _fixed_scale_error;
		}
		cb_args.clicked_points_3d->points.clear();
		cerr << "\n";
	}

	void EvaluationTool::storeMovingPts(){

		for(unsigned int i = 0; i < cb_args.clicked_points_3d->points.size(); i++){
			
			_moving_pts.push_back( Vector3(	cb_args.clicked_points_3d->points[i].x,
								  			cb_args.clicked_points_3d->points[i].y,
											cb_args.clicked_points_3d->points[i].z) );

			cerr << "Moving Point N°: " << i << " " << _moving_pts[i].transpose() << "\n";
    	}

		cb_args.clicked_points_3d->points.clear();
		cerr << "\n";

	}

	void EvaluationTool::checkMovingCloudScaleError(){

		_moving_scale_error_pts.push_back ( Vector3 ( cb_args.clicked_points_3d->points[0].x,
											cb_args.clicked_points_3d->points[0].y,
											cb_args.clicked_points_3d->points[0].z));

		_moving_scale_error_pts.push_back ( Vector3 ( cb_args.clicked_points_3d->points[1].x,
											cb_args.clicked_points_3d->points[1].y,
											cb_args.clicked_points_3d->points[1].z));

		float real_scale = ( _moving_scale_error_pts[0] - _moving_scale_error_pts[1]).norm();
		_moving_scale_error = _scale / real_scale;
		cerr << " Scale Error: " << _moving_scale_error << "\n";
		for(size_t i = 0; i < pcl_map["moving_cloud"]->size(); ++i){

			(*pcl_map["moving_cloud"]).points[i].x *= _moving_scale_error;
			(*pcl_map["moving_cloud"]).points[i].y *= _moving_scale_error;
			(*pcl_map["moving_cloud"]).points[i].z *= _moving_scale_error;
		}
		cb_args.clicked_points_3d->points.clear();
		cerr << "\n";
	}

	Transform EvaluationTool::computeGtTransform(){

		srrg_keypoint::Scene* _moving_scene = new srrg_keypoint::Scene();
      	srrg_keypoint::Scene* _fixed_scene = new srrg_keypoint::Scene();
      	srrg_jizz::Aligner aligner;
      	aligner.setMaxIterations(_max_iter_num);
      	aligner.setVerbosity(_verbosity);

		for( size_t i = 0; i < _fixed_pts.size(); ++i) { 
	  
			_fixed_scene->push_back( srrg_keypoint::KeypointPtr( new srrg_keypoint::Keypoint( _fixed_pts.at(i) ) ) );
			_moving_scene->push_back( srrg_keypoint::KeypointPtr( new srrg_keypoint::Keypoint( _moving_pts.at(i) ) ) );
	  	}
      
		aligner.nnCorrespondenceFinder().mutableConfig().max_distance = 50;
		aligner.setFixedScene(_fixed_scene);
		aligner.setMovingScene(_moving_scene);
		aligner.compute(Transform::Identity());
		const Transform& currTransform = aligner.T();
		deltaT = currTransform;

		cerr << "\n" << "Ground truth translation: \n";
		cerr <<  deltaT.translation().transpose() << "\n" << "\n";
		cerr << "Ground truth rotation: \n";
		cerr << deltaT.rotation() << "\n" << "\n";

		return deltaT;
	}

	void EvaluationTool::writeTransformYAML(const string& outut_file_name){

		YAML::Node yaml_output_file;
		cerr << "storing fixed cloud pts ... \n";

		if( this->checkForScaleError() ){

			YAML::Node scale_error_fixed_pts;
			for(size_t i = 0; i < _fixed_scale_error_pts.size(); ++i){

				stringstream ss;
				ss << i;
				string fixed_pt_num = ss.str();

				YAML::Node pt;
				pt.push_back( _fixed_scale_error_pts.at(i)(0) );
				pt.push_back( _fixed_scale_error_pts.at(i)(1) );
				pt.push_back( _fixed_scale_error_pts.at(i)(2) );
				YAML::Node current_pt;
				current_pt["pt_" + fixed_pt_num] = (pt);
				scale_error_fixed_pts["scale_error_fixed_pts"].push_back(current_pt);
			}

			yaml_output_file.push_back(scale_error_fixed_pts);

			YAML::Node fixed_scale_error;
			fixed_scale_error["scale_error"] = _fixed_scale_error;
			yaml_output_file.push_back(fixed_scale_error);

			YAML::Node scale_error_moving_pts;
			for(size_t i = 0; i < _fixed_scale_error_pts.size(); ++i){

				stringstream ss;
				ss << i;
				string fixed_pt_num = ss.str();

				YAML::Node pt;
				pt.push_back( _fixed_scale_error_pts.at(i)(0) );
				pt.push_back( _fixed_scale_error_pts.at(i)(1) );
				pt.push_back( _fixed_scale_error_pts.at(i)(2) );
				YAML::Node current_pt;
				current_pt["pt_" + fixed_pt_num] = (pt);
				scale_error_moving_pts["scale_error_moving_pts"].push_back(current_pt);
			}

			yaml_output_file.push_back(scale_error_moving_pts);
			
			YAML::Node moving_scale_error;
			moving_scale_error["scale_error"] = _moving_scale_error;
			yaml_output_file.push_back(moving_scale_error);
		}
 
		YAML::Node fixed_pts;
		for(size_t i = 0; i < _fixed_pts.size(); ++i){

			stringstream ss;
			ss << i;
			string fixed_pt_num = ss.str();

			YAML::Node pt;
			pt.push_back( _fixed_pts.at(i)(0) );
			pt.push_back( _fixed_pts.at(i)(1) );
			pt.push_back( _fixed_pts.at(i)(2) );
			YAML::Node current_pt;
			current_pt["pt_" + fixed_pt_num] = (pt);
			fixed_pts["fixed_pts"].push_back(current_pt);
		}

		yaml_output_file.push_back(fixed_pts);

		cerr << "storing moving cloud pts ... \n"; 
		YAML::Node moving_pts;
		for(size_t i = 0; i < _moving_pts.size(); ++i){

			stringstream ss;
			ss << i;
			string moving_pt_num = ss.str();

			YAML::Node pt;
			pt.push_back( _moving_pts.at(i)(0) );
			pt.push_back( _moving_pts.at(i)(1) );
			pt.push_back( _moving_pts.at(i)(2) );
			YAML::Node current_pt;
			current_pt["pt_" + moving_pt_num] = (pt);
			moving_pts["moving_pts"].push_back(current_pt);
		}

		yaml_output_file.push_back(moving_pts);

		cerr << "storing ground truth transform ... \n"; 
		YAML::Node gt_tf;
		YAML::Node t;
		t.push_back( deltaT.translation()(0) );
		t.push_back( deltaT.translation()(1) );
		t.push_back( deltaT.translation()(2) );
		YAML::Node current_t;
		current_t["translation"] = (t);
		gt_tf["ground_truth_transform"].push_back(current_t);

		Eigen::Quaternionf q( deltaT.rotation() );
		YAML::Node quat;
		quat.push_back( q.w() );
		quat.push_back( q.x() );
		quat.push_back( q.y() );
		quat.push_back( q.z() );
		YAML::Node current_q;
		current_q["translation"] = (quat);
		gt_tf["ground_truth_transform"].push_back(current_q);

		yaml_output_file.push_back(gt_tf);

		ofstream output_writer( package_path + "/params/output/" + outut_file_name );
  		output_writer << yaml_output_file << "\n";
		cerr << "Output Ground Truth Transform written in: " << package_path + "params/output/" + outut_file_name << "\n" << "\n";
	}

}

