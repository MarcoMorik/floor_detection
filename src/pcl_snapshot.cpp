#include <iostream>
#include <sstream>
#include <cstdio>
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
        action_(NONE)
    {
        pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/snapshot/pcl", 1);
        pclSub_ = nh_.subscribe("/camera/depth_registered/points", 1, &pclSnapshot::pclCB, this);
    }

    void pclCB(const sensor_msgs::PointCloud2ConstPtr& pclMsg) {
        if(action_ != NONE) {
            Cloud cTemp;
            Cloud2 c2Temp;
            pcl_conversions::toPCL(*pclMsg, c2Temp);
            pcl::fromPCLPointCloud2(c2Temp, cTemp);

            if(action_ == SAVE) {
                std::stringstream ss;
                ss << "pcl_" << cTemp.header.stamp;
                pcl::io::savePCDFileASCII(ss.str(), cTemp);
                std::cout << "Snapshot " << ss.str() << " created." << std::endl;
            }

            if(action_ == PUBLISH) {
                sensor_msgs::PointCloud2 msgOut;
                pcl::toROSMsg(cTemp, msgOut);
                pcl_pub_.publish(msgOut);
                std::cout << "Snapshot published" << std::endl;
            }
        }
        action_ = NONE;
    }

    void saveSnapshot() {
        std::cout << "Saving snapshot" << std::endl;
        action_ = SAVE;
        ros::spinOnce();
    }

    void publishSnapshot() {
        std::cout << "Publishing snapshot" << std::endl;
        action_ = PUBLISH;
        ros::spinOnce();
    }

    void publishFile(std::string file) {
        std::cout << "Reading file " << file << std::endl;
        Cloud input;
        if(pcl::io::loadPCDFile(file, input) == -1) {
            return;
        }
        sensor_msgs::PointCloud2 msgOut;
        input.header.frame_id = "floor";
        pcl::toROSMsg(input, msgOut);
        pcl_pub_.publish(msgOut);
        std::cout << "File published" << std::endl;
    }

private:
    enum {NONE, SAVE, PUBLISH};
    int action_;
    ros::NodeHandle nh_;
    ros::Subscriber pclSub_;
    ros::Publisher pcl_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_snapshot");
    pclSnapshot pcls;
    std::cout << "Type 'p' to take snapshot and publish.\nType 's' to take snapshot and save it to file.\nType path to pcl file to read and publish the file.\nPress 'ctrl+d' to exit." << std::endl << std::endl;
    std::string action;
    while(ros::ok() && std::cin >> action) {
        if(action == "s") {
            pcls.saveSnapshot();
        } else if(action == "p") {
            pcls.publishSnapshot();
        } else {
            pcls.publishFile(action);
        }
        std::cout << std::endl;
    }
}
