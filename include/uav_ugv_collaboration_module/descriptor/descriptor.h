#pragma once
#include <uav_ugv_collaboration_module/utils.hpp>

using namespace std;
using namespace pcl_utils;

namespace Descriptor {

  
    class CentroidDescriptor{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      public:
		CentroidDescriptor(float& ray, float& x, float& y, float& z);
		CentroidDescriptor(float& ray, Vector3& centroid);
		~CentroidDescriptor();

		const static int size = 4;

		inline float getRay(){ return _ray;}
		inline Vector3 getCentroid(){ return _centroid;}
		
		float _ray;
		Vector3 _centroid;

      private:

	
    };

}