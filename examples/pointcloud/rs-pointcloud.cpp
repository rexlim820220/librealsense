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

    //filter out the background
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimitsNegative (1);
    pass.setFilterLimits(-1, 1e6);
    pass.filter(*cloud);

    pcl::visualization::CloudViewer viewer("CLoud  Viewer");
    viewer.showCloud (cloud);


    pcl::io::savePCDFile("/home/a/Desktop/d54.pcd", *cloud, false);

    return EXIT_SUCCESS;
}
