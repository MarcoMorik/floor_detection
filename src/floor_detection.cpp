#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PCLPointCloud2 Cloud2;

class floorDetection {
public:
    floorDetection() {
        _pcl_sub = _nh.subscribe("/filepub/pcl", 1, &floorDetection::detectFloor, this);
        _floor_pub = _nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor", 1);
        _all_pub = _nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/all", 1);
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    }

    void detectFloor(const sensor_msgs::PointCloud2ConstPtr& pclMsg) {
//        double r, p, y;
//        _nh.getParam("/floorDetection/r", r);
//        _nh.getParam("/floorDetection/p", p);
//        _nh.getParam("/floorDetection/y", y);
        q.setRPY(-1.9, 0, 0);
        transform.setRotation(q);
        _tb.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "floor"));

//        Cloud::Ptr input(new Cloud);
//        if(pcl::io::loadPCDFile("/home/haso/catkin_ws/src/floor_detection/test_clouds/" + file, *input) == -1) {
//            std::cout << "File not fond" << std::endl;
//            return;
//        }

        Cloud::Ptr input(new Cloud);
        Cloud2 cloud2In;
        pcl_conversions::toPCL(*pclMsg, cloud2In);
        pcl::fromPCLPointCloud2(cloud2In, *input);

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;
        pcl::SACSegmentation<Point> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(input);
        seg.segment(inliers, coefficients);

        Cloud result(*input, inliers.indices);
        for(Cloud::iterator it = result.begin(); it != result.end(); ++it) {
            it->r = 0;
            it->g = 255;
            it->b = 0;
        }
        std::cout << "floor plane: " << result.size() << std::endl;
        sensor_msgs::PointCloud2 msgOut;
        result.header.frame_id = "floor";
        pcl::toROSMsg(result, msgOut);
        _floor_pub.publish(msgOut);
        sensor_msgs::PointCloud2 allOut;
        input->header.frame_id = "floor";
        pcl::toROSMsg(*input, allOut);
        _all_pub.publish(allOut);
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _pcl_sub;
    ros::Publisher _floor_pub;
    ros::Publisher _all_pub;
    tf::TransformBroadcaster _tb;
    tf::Transform transform;
    tf::Quaternion q;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "floor_detection");
    floorDetection fd;
    ros::spin();
}
