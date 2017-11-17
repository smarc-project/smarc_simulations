#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <smarc_gazebo_ros_plugins/SonarEntities.h>
#include <tf/transform_listener.h>

#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>

using namespace std;

class WaterfallImageNode {

public:

    ros::NodeHandle n;
    ros::Subscriber sonar_sub;
    ros::Subscriber entity_sub;
    ros::Publisher waterfall_pub;
    tf::TransformListener listener;

    string auv_namespace;
    string auv_frame;
    cv::Mat waterfall_image;
    int timesteps;
    double start_angle;
    double fov;
    int rays;
    int samples;
    int offset;
    int width;
    bool angle_unknown;
    bool is_left;

    std::default_random_engine generator;

    //Eigen::VectorXi sample_indices;
    //Eigen::Vector3d last_pos;
	cv::Point3f last_pos;

	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WaterfallImageNode()
    {
        ros::NodeHandle pn("~");
        string auv_name;
        pn.param<string>("auv_name", auv_name, "small_smarc_auv");
        auv_namespace = string("/") + auv_name;
        auv_frame = auv_name+"/base_link";
        pn.param<int>("timesteps", timesteps, 200);
        pn.param<double>("start_angle", start_angle, 0.0);
        pn.param<double>("fov", fov, 60.0);
        fov *= M_PI/180.0;
        pn.param<int>("rays", rays, 120);
        pn.param<int>("samples", samples, 60);
        rays -= 1;
        pn.param<bool>("angle_unknown", angle_unknown, true);
        pn.param<bool>("is_left", is_left, true);
        string side_name;
        if (is_left) {
            side_name = "left";
        }
        else {
            side_name = "right";
        }

        offset = 5;
        width = 2*samples + 2*width;
        waterfall_image = cv::Mat::zeros(timesteps, width, CV_8UC1);

		ROS_INFO("Waiting for transform...");
		try {
		    listener.waitForTransform("/world", auv_frame,
		                              ros::Time(0), ros::Duration(60.0));
		}
		catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
		}
		ROS_INFO("Got transform...");

        waterfall_pub = n.advertise<sensor_msgs::Image>(auv_namespace+"/sss_"+side_name+"_waterfall", 1);
        sonar_sub = n.subscribe(auv_namespace+"/sss_"+side_name, 10, &WaterfallImageNode::sonar_callback, this);
        entity_sub = n.subscribe(auv_namespace+"/sss_"+side_name+"_entities", 10, &WaterfallImageNode::entities_callback, this);
    }

    void sonar_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/world", auv_frame,  
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        auto min_range_iter = std::min_element(scan->ranges.begin(), scan->ranges.end());
        double depth = double(*min_range_iter);
		cv::Point3f current_pos(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        double pixel_distance = tan(fov)*depth/double(samples);

        if (cv::norm(current_pos - last_pos) < pixel_distance) {
            return;
        }
        last_pos = current_pos;

        auto max_intensity_iter = std::max_element(scan->intensities.begin(), scan->intensities.end());
        auto max_range_iter = std::max_element(scan->ranges.begin(), scan->ranges.end());
        double min_range = sin(0.0)*scan->ranges[0];
        double max_range = sin(fov)*(*max_range_iter) - min_range;


        size_t max_index = std::distance(scan->intensities.begin(), max_intensity_iter);

        double scale = 1.0; //255.0/max_index;
        std::normal_distribution<double> noise_distribution(0.0, 10.0);

        // now we should move everything now one notch
        // let's just do the left for now, maybe synch later
        cv::Mat shifted = cv::Mat::zeros(timesteps, width, waterfall_image.type());
        waterfall_image(cv::Rect(0, 0, waterfall_image.cols, waterfall_image.rows-1)).copyTo(shifted(cv::Rect(0, 1, shifted.cols, shifted.rows-1)));
        shifted.copyTo(waterfall_image);

		std::normal_distribution<double> shift_distribution(0.0, 1.0);
		int noise_shift = int(shift_distribution(generator));

		//if (angle_unknown) {
        int ray = 0;
        for (int i = 0; i < samples && ray < rays; ) {
            int index = is_left? width/2-offset-1-i : width/2+offset+i;
			index += noise_shift;
            if ((sin(fov*ray/rays)*scan->ranges[ray] - min_range) > max_range*i/samples) {
                ++i;
            }
            else {
                double value = scale*scan->intensities[ray] + noise_distribution(generator);
				if (index >= 0 && index < waterfall_image.cols) {
				    waterfall_image.at<uchar>(0, index) = uchar(std::max(0.0, std::min(255.0, value)));
				}
                ++ray;
            }
        }

        sensor_msgs::Image img_msg; // >> message to be sent

        std_msgs::Header header; // empty header
        //header.seq = counter; // user defined counter
        header.stamp = ros::Time::now(); // time
        cv_bridge::CvImage img_bridge;
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, waterfall_image);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        
        waterfall_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
    }

    void entities_callback(const smarc_gazebo_ros_plugins::SonarEntities::ConstPtr& msg)
    {

    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sidescan_waterfall_image_node");

    WaterfallImageNode node;

    ros::spin();

    return 0;
}
