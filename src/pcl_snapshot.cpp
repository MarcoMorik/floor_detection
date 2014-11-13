#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PCLPointCloud2 Cloud2;

class pclSnapshot {
public:
    pclSnapshot() :
        _createSnapshot(false)
    {
        _pclSub = _nh.subscribe("/camera/depth_registered/points", 1, &pclSnapshot::pclCB, this);
    }

    void pclCB(const sensor_msgs::PointCloud2ConstPtr& pclMsg) {
        if(_createSnapshot) {
            _createSnapshot = false;
            Cloud cTemp;
            Cloud2 c2Temp;
            pcl_conversions::toPCL(*pclMsg, c2Temp);
            pcl::fromPCLPointCloud2(c2Temp, cTemp);
            std::stringstream ss;
            ss << "pcl_" << cTemp.header.stamp;
            pcl::io::savePCDFileASCII(ss.str(), cTemp);
            std::cout << "Snapshot " << ss.str() << " created." << std::endl;
        }
    }

    void snap() {
        std::cout << "Taking snapshot" << std::endl;
        _createSnapshot = true;
    }

private:
    bool _createSnapshot;
    ros::NodeHandle _nh;
    ros::Subscriber _pclSub;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_snapshot");
    pclSnapshot pcls;
    while(ros::ok()) {
        std::cin.ignore();
        pcls.snap();
        ros::spinOnce();
    }
}
