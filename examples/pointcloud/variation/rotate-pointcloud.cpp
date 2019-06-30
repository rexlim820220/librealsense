//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"          // Include short list of convenience functions for rendering

#include <iostream>
#include <algorithm>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

void transform(pcl::PointCloud <pcl::PointXYZ>::Ptr &);
void visualize(pcl::PointCloud <pcl::PointXYZ>::Ptr &);

int main(int argc, char * argv[])
{
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    char fname[FILENAME_MAX] = "/home/a/Desktop/e504.pcd";
    if(pcl::io::loadPCDFile(fname, *cloud) == -1){
        PCL_ERROR("Couldn't read pcd file.\n");
        return -1;
    }
    //transform(cloud);
    visualize(cloud);

    return EXIT_SUCCESS;
}

void transform(pcl::PointCloud <pcl::PointXYZ>::Ptr &cloud) {
    float theta;
    int a = 4 , b = 3;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ> ());
    theta = a * M_PI / b;
    //trans.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitY()));*/
    trans.translation() << 0.5, 0.0, 0.0;
    pcl::transformPointCloud ( *cloud , *transformed, trans);
    //visualize(transformed);
    pcl::io::savePCDFile("/home/a/Desktop/e504.pcd", *transformed, false);
}

void visualize(pcl::PointCloud <pcl::PointXYZ>::Ptr &cloud) {
    pcl::visualization::CloudViewer viewer("CLoud  Viewer");
    viewer.showCloud (cloud);
    while(!viewer.wasStopped()){

    }
}
