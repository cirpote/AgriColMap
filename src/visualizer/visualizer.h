#pragma once
#include <uav_ugv_collaboration_module/utils.h>
 
class PointCloudViz{

    public:
        PointCloudViz(const std::string& win_name,
                      const int width = 640,
                      const int height = 480,
                      const int left = 50,
                      const int top = 50);
        ~PointCloudViz();

        void VisualizePointCloud(const std::shared_ptr<open3d::PointCloud> pcl);

        /*void inline spinOnce(){ viewer->spinOnce(); }

        void setViewerPosition(const float& x, const float& y, const float& z,
                               const float& view_x, const float& view_y, const float& view_z);

        void setViewerBackground(const float& r, const float& g, const float& b);

        void showTransparentCloud(const PCLPointCloudXYZRGB::Ptr cloud,
                       const std::string &cloud_to_show, const int& size = 1);*/

        void spingUntilDeath();

    //private:
        std::shared_ptr<open3d::Visualizer> visualizer;

        /*if (visualizer.CreateVisualizerWindow(window_name, width, height, left, top) ==
                false) {
            PrintWarning("[DrawGeometries] Failed creating OpenGL window.\n");
            return false;
        }
        for (const auto &geometry_ptr : geometry_ptrs) {
            if (visualizer.AddGeometry(geometry_ptr) == false) {
                PrintWarning("[DrawGeometries] Failed adding geometry.\n");
                PrintWarning("[DrawGeometries] Possibly due to bad geometry or wrong geometry type.\n");
                return false;
            }
        }
        visualizer.Run();
        visualizer.DestroyVisualizerWindow();*/

};
