#include "visualizer.h"

PointCloudViz::PointCloudViz(const std::string& win_name,
                             const int width,
                             const int height,
                             const int left,
                             const int top) {


    visualizer = std::shared_ptr<open3d::Visualizer>( new open3d::Visualizer() );

    if (!visualizer->CreateVisualizerWindow(win_name, width, height, left, top)){
        cerr << FRED("[DrawGeometries] Failed creating OpenGL window.\n");
        return;
    }

    visualizer->GetRenderOption().ChangePointSize(-5);
    visualizer->GetRenderOption().show_coordinate_frame_ = true;

}

PointCloudViz::~PointCloudViz(){
    std::cerr << FRED("[PointCloudViz] deleting\n");
    std::cerr << BOLD( FRED("[PointCloudViz] deleted\n") );
}

void PointCloudViz::VisualizePointCloud(const std::shared_ptr<open3d::PointCloud> pcl){

    for (const auto &geometry_ptr : {pcl}) {
        if (visualizer->AddGeometry(geometry_ptr) == false) {
            cerr << FRED("[DrawGeometries] Failed adding geometry.\n");
            cerr << FRED("[DrawGeometries] Possibly due to bad geometry or wrong geometry type.\n");
            return;
        }
    }

}

/*void PointCloudViz::setViewerPosition(const float &x, const float &y, const float &z,
                                      const float &view_x, const float &view_y, const float &view_z){
    viewer->setCameraPosition(x, y, z, view_x, view_y, view_z);
}

void PointCloudViz::setViewerBackground(const float &r, const float &g, const float &b){
    viewer->setBackgroundColor (r, g, b);
}

void PointCloudViz::removeCloud(const std::string &cloud_to_show){
    viewer->removePointCloud(cloud_to_show);
}

void PointCloudViz::showCloud(const PCLPointCloudXYZRGB::Ptr cloud,
                              const std::string& cloud_to_show , const int &size){

    PCLXYZRGB_viz RGB(cloud);
    viewer->addPointCloud<PCLptXYZRGB> (cloud, RGB, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cloud_to_show);

}

void PointCloudViz::showTransparentCloud(const PCLPointCloudXYZRGB::Ptr cloud,
                                         const std::string& cloud_to_show, const int& size ){

    PCLXYZRGB_viz RGB(cloud);
    viewer->addPointCloud<PCLptXYZRGB> (cloud, RGB, cloud_to_show);
    //viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cloud_to_show);

}*/


void PointCloudViz::spingUntilDeath(){

    visualizer->Run();
    visualizer->DestroyVisualizerWindow();
}
