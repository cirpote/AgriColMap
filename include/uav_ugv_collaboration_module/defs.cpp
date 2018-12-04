#include "defs.h"

namespace instances {

    PCLPointCloudXYZRGB::Ptr pclxyzrgb_instance( new PCLPointCloudXYZRGB() );
    PCLPointCloudXYZ::Ptr pclxyz_instance( new PCLPointCloudXYZ() );
    PCLKDtreeXYZ kdtreexyz_instance();
    PCLsegmentationXYZ segmentation_xyz_instance();
    PCLsegmentationXYZRGB segmentation_xyzrgb_instance();
    PCLvoxelGridXYZRGB voxelgrid_xyzrgb_instance();
    GroundTruthUnorderedMap gt_un_map_instance();
    TransformUnorderedMap tf_un_map_instance();
    PCLXYZRGB_viz xyzrgb_viz_instance(pclxyzrgb_instance);
}
