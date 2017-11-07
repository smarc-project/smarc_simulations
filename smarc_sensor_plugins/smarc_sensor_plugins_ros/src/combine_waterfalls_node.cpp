#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <smarc_gazebo_ros_plugins/SonarEntities.h>

#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

class CombineWaterfallImageNode {

public:

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    ros::NodeHandle n;
    ros::Publisher waterfall_pub;
    message_filters::Subscriber<sensor_msgs::Image>* waterfall_left_sub;
    message_filters::Subscriber<sensor_msgs::Image>* waterfall_right_sub;
    message_filters::Synchronizer<MySyncPolicy>* sync;

    string auv_namespace;

    CombineWaterfallImageNode()
    {
        ros::NodeHandle pn("~");
        string auv_name;
        pn.param<string>("auv_name", auv_name, "small_smarc_auv");
        auv_namespace = string("/") + auv_name;

        waterfall_left_sub = new message_filters::Subscriber<sensor_msgs::Image>(n, auv_namespace+"/sss_left_waterfall", 1);
        waterfall_right_sub = new message_filters::Subscriber<sensor_msgs::Image>(n, auv_namespace+"/sss_right_waterfall", 1);

        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *waterfall_left_sub, *waterfall_right_sub);
        sync->registerCallback(boost::bind(&CombineWaterfallImageNode::callback, this, _1, _2));

        waterfall_pub = n.advertise<sensor_msgs::Image>(auv_namespace+"/waterfall", 1);
    }

    void callback(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right)
    {
        cv_bridge::CvImagePtr cv_left_ptr;
        try {
            cv_left_ptr = cv_bridge::toCvCopy(image_left, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImagePtr cv_right_ptr;
        try {
            cv_right_ptr = cv_bridge::toCvCopy(image_right, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_left_ptr->image + cv_right_ptr->image;

        sensor_msgs::Image img_msg; // >> message to be sent
        std_msgs::Header header; // empty header
        //header.seq = counter; // user defined counter
        header.stamp = ros::Time::now(); // time
        cv_bridge::CvImage img_bridge;
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        
        waterfall_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "combine_waterfalls_node");

    CombineWaterfallImageNode node;

    ros::spin();

    return 0;
}
