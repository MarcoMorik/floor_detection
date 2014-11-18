#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PCLPointCloud2 Cloud2;

class pcl_file_publisher {
public:
    pcl_file_publisher() {
        _str_sub = _nh.subscribe("/pcl_pub/file", 1, &pcl_file_publisher::publishFileCB, this);
        _pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("/filepub/pcl", 1);
    }

    void publishFileCB(const std_msgs::String& file) {
        Cloud::Ptr input(new Cloud);
        if(pcl::io::loadPCDFile(file.data, *input) == -1) {
            return;
        }

        sensor_msgs::PointCloud2 msgOut;
        input->header.frame_id = "floor";
        pcl::toROSMsg(*input, msgOut);
        _pcl_pub.publish(msgOut);
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _str_sub;
    ros::Publisher _pcl_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_file_publisher");
    pcl_file_publisher fp;
    ros::spin();
}
