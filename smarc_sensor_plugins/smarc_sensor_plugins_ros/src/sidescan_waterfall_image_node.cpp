#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <smarc_gazebo_ros_plugins/SonarEntities.h>

#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <cmath>

using namespace std;

class WaterfallImageNode {

public:

    ros::NodeHandle n;
    ros::Subscriber sonar_left_sub;
    ros::Subscriber sonar_right_sub;
    ros::Subscriber entity_left_sub;
    ros::Subscriber entity_right_sub;
    ros::Publisher waterfall_pub;

    string auv_namespace;
    cv::Mat waterfall_image;
    int timesteps;
    double start_angle;
    double fov;
    int rays;
    int samples;
    int offset;
    int width;
    bool angle_unknown;

    Eigen::VectorXi sample_indices;

    WaterfallImageNode()
    {
        ros::NodeHandle pn("~");
        string auv_name;
        pn.param<string>("auv_name", auv_name, "small_smarc_auv");
        auv_namespace = string("/") + auv_name;
        pn.param<int>("timesteps", timesteps, 200);
        pn.param<double>("start_angle", start_angle, 0.0);
        pn.param<double>("fov", fov, 60.0);
        fov *= M_PI/180.0;
        pn.param<int>("rays", rays, 59);
        pn.param<int>("samples", samples, 40);
        pn.param<bool>("angle_unknown", angle_unknown, true);

        // with start angle and fov, we should be able to decide the index of the samples to keep
        sample_indices.resize(samples);
        double height = double(samples)/tan(fov);
        cout << "Height: " << height << endl;
        for (int i = 0; i < samples; ++i) {
            double dindex = atan(double(i+0.5)/height);
            int index = int(dindex*double(rays)/fov);
            sample_indices[i] = index;
        }

        cout << "Sample indices: " << sample_indices << endl;

        offset = 10;
        width = 2*samples+2*width;
        waterfall_image = cv::Mat::zeros(timesteps, width, CV_8UC1);

        waterfall_pub = n.advertise<sensor_msgs::Image>(auv_namespace+"/waterfall_image", 1);
        sonar_left_sub = n.subscribe(auv_namespace+"/sss_left", 10, &WaterfallImageNode::sonar_left_callback, this);
        sonar_right_sub = n.subscribe(auv_namespace+"/sss_right", 10, &WaterfallImageNode::sonar_right_callback, this);
        entity_left_sub = n.subscribe(auv_namespace+"/sss_left_entities", 10, &WaterfallImageNode::entities_left_callback, this);
        entity_right_sub = n.subscribe(auv_namespace+"/sss_right_entities", 10, &WaterfallImageNode::entities_right_callback, this);
    }

    void sonar_left_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        auto max_intensity_iter = std::max_element(scan->intensities.begin(), scan->intensities.end());
        auto max_range_iter = std::max_element(scan->ranges.begin(), scan->ranges.end());
        double min_range = scan->ranges[0];
        double max_range = *max_range_iter - min_range;

        cout << "=========" << endl;
        cout << max_range << endl;
        cout << min_range << endl;
        cout << *max_intensity_iter << endl;

        size_t max_index = std::distance(scan->intensities.begin(), max_intensity_iter);

        /*
        int max_index = 0;
        for (int i = 0; i < scan->intensities.size(); ++i) {
            max_index = scan->intensities[i] > scan->intensities[max_index]? i : max_index; 
        }
        */

        double scale = 1.0; //255.0/max_index;

        // now we should move everything now one notch
        // let's just do the left for now, maybe synch later
        cv::Mat shifted = cv::Mat::zeros(timesteps, width, waterfall_image.type());
        waterfall_image(cv::Rect(0, 0, waterfall_image.cols, waterfall_image.rows-1)).copyTo(shifted(cv::Rect(0, 1, shifted.cols, shifted.rows-1)));
        shifted.copyTo(waterfall_image);

		if (angle_unknown) {
            int ray = 0;
            for (int i = 0; i < samples && ray < rays; ) {
                if ((scan->ranges[ray] - min_range) > max_range*double(i)/double(samples)) {
                    ++i;
                }
                else {
                    waterfall_image.at<uchar>(0, width/2-offset-1-i) = uchar(scale*scan->intensities[ray]);
                    //++i;
                    ++ray;
                }
            }
		}
		else {
            for (int i = 0; i < samples; ++i) {
                waterfall_image.at<uchar>(0, width/2-offset-1-i) = uchar(scale*scan->intensities[sample_indices(i)]);
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
    
    void sonar_right_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // now we should move everything now one notch
    }

    void entities_left_callback(const smarc_gazebo_ros_plugins::SonarEntities::ConstPtr& msg)
    {

    }

    void entities_right_callback(const smarc_gazebo_ros_plugins::SonarEntities::ConstPtr& msg)
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
