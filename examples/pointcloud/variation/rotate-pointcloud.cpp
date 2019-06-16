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

int main(int argc, char * argv[])
{

    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    char fname[FILENAME_MAX] = "/home/a/Desktop/655.pcd";
    if(pcl::io::loadPCDFile(fname, *cloud) == -1){
        PCL_ERROR("Couldn't read pcd file.\n");
        return -1;
    }
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid<pcl::PointXYZ> (*cloud, centroid);
    pcl::demeanPointCloud <pcl::PointXYZ, double> (*cloud, centroid, *cloud);

    pcl::visualization::CloudViewer viewer("CLoud  Viewer");

    float theta;
    int a = 0 , b = 1;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ> ());

    for(b = 1 ; ; b += 1) {
        for(a = 0; a/b < 2; a += 1) {
            theta = a * M_PI / b;
            std::cout<<a<<' '<<b<<' '<<theta<<'\n';
            trans.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
            pcl::transformPointCloud ( *cloud , *transformed, trans);
            viewer.showCloud (transformed);
            //boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
        }
    }

    return EXIT_SUCCESS;
}
