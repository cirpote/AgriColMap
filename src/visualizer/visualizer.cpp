#include "visualizer.h"

PointCloudViz::PointCloudViz() {

    viewer = ( new pcl::visualization::PCLVisualizer ("PointCloud Viewer") );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(0.f, 0.f, 100.f, 0.f, 1.f, 0.f);
}

PointCloudViz::~PointCloudViz(){
    std::cerr << FRED("[PointCloudViz] deleting\n");
    delete[] viewer;
    std::cerr << BOLD( FRED("[PointCloudViz] deleted\n") );
}

void PointCloudViz::setViewerPosition(const float &x, const float &y, const float &z,
                                      const float &view_x, const float &view_y, const float &view_z){
    viewer->setCameraPosition(x, y, z, view_x, view_y, view_z);
}

void PointCloudViz::setViewerBackground(const float &r, const float &g, const float &b){
    viewer->setBackgroundColor (r, g, b);
}

void PointCloudViz::removeCloud(const std::string &cloud_to_show){
    viewer->removePointCloud(cloud_to_show);
}

void PointCloudViz::showCloud(const std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& points,
                               const std::string& cloud_to_show , const int &size){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
    for( pcl::PointXYZRGB pt : points)
        cloud->points.push_back( pt );

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> RGB(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, RGB, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cloud_to_show);

}

void PointCloudViz::showTransparentCloud(const std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& points,
                                         const std::string& cloud_to_show, const int& size ){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );

    for( pcl::PointXYZRGB pt : points)
        cloud->points.push_back( pt );

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> RGB(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, RGB, cloud_to_show);
    //viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, cloud_to_show);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cloud_to_show);

}


void PointCloudViz::spingUntilDeath(){

    viewer->setSize(1920, 1080);
    while(!viewer->wasStopped())
        viewer->spin();
}
