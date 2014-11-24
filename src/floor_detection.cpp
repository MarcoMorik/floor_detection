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
        pcl_sub_ = nh_.subscribe("/snapshot/pcl", 1, &floorDetection::detectFloor, this);
        floor_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor", 1);
    }

    void detectFloor(const sensor_msgs::PointCloud2ConstPtr& pclMsg) {
        Cloud::Ptr input(new Cloud);
        Cloud2 cloud2In;
        pcl_conversions::toPCL(*pclMsg, cloud2In);
        pcl::fromPCLPointCloud2(cloud2In, *input);

        //coefficients on the form ax + by + cz + d = 0
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;
        pcl::SACSegmentation<Point> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(input);
        seg.segment(inliers, coefficients);

        tf::Vector3 floorNormal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
        floorNormal.normalize();
        double angle = std::acos(floorNormal.z());

        //Broadcast the tranform
        transform_.setOrigin(tf::Vector3(0.0, 0.0, coefficients.values[3]));
        q_.setRPY(-angle, 0, 0);
        transform_.setRotation(q_);
        tb_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "robot_center", "camera_rgb_optical"));

        //Publish pointcloud of floor, not really necessary other than for debugging
        Cloud result(*input, inliers.indices);
        for(Cloud::iterator it = result.begin(); it != result.end(); ++it) {
            it->r = 0;
            it->g = 255;
            it->b = 0;
        }
        sensor_msgs::PointCloud2 msgOut;
        result.header.frame_id = "floor";
        pcl::toROSMsg(result, msgOut);
        floor_pub_.publish(msgOut);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pcl_sub_;
    ros::Publisher floor_pub_;
    tf::TransformBroadcaster tb_;
    tf::Transform transform_;
    tf::Quaternion q_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "floor_detection");
    floorDetection fd;
    ros::spin();
}
