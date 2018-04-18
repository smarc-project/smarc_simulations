/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <smarc_gazebo_ros_plugins/gazebo_ros_underwater_sonar_ray.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboUnderwaterSonarRay)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboUnderwaterSonarRay::GazeboUnderwaterSonarRay()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboUnderwaterSonarRay::~GazeboUnderwaterSonarRay()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboUnderwaterSonarRay::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboUnderwaterSonarRay controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");


  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_NAMED("laser", "Starting Laser Plugin (ns = %s)", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboUnderwaterSonarRay::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboUnderwaterSonarRay::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  if(this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
  }
  ROS_INFO_NAMED("laser", "Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 1,
      boost::bind(&GazeboUnderwaterSonarRay::LaserConnect, this),
      boost::bind(&GazeboUnderwaterSonarRay::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboUnderwaterSonarRay::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1) {
    this->laser_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                    &GazeboUnderwaterSonarRay::OnScan, this);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboUnderwaterSonarRay::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboUnderwaterSonarRay::OnScan(ConstLaserScanStampedPtr &_msg)
{
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min = _msg->scan().angle_min();
  laser_msg.angle_max = _msg->scan().angle_max();
  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment = 0;  // instantaneous simulator scan
  laser_msg.scan_time = 0;  // not sure whether this is correct
  laser_msg.range_min = _msg->scan().range_min();
  laser_msg.range_max = _msg->scan().range_max();
  
  double verticalAngleMin = _msg->scan().vertical_angle_min();
  double verticalAngleMax = _msg->scan().vertical_angle_max();
  double verticalAngleStep = _msg->scan().vertical_angle_step();
  
  unsigned int rangeCount = _msg->scan().count();
  laser_msg.ranges.resize(rangeCount);
  laser_msg.intensities.resize(rangeCount);
  
  unsigned int verticalRangeCount = _msg->scan().vertical_count();

  GZ_ASSERT(verticalRangeCount == 2, "Vertical ray count needs to be 2 for angle interpolation"); 

  unsigned int rayCount = rangeCount;
  //unsigned int verticalRayCount = vert;
  double rayAngle = M_PI/180.0*(_msg->scan().angle_max() - _msg->scan().angle_max()) / double(rangeCount-1);
  // interpolate in horizontal direction
  for (unsigned int i = 0; i < rangeCount-1; ++i) // TODO: fix the last one eventually
  {
    double angle, range, range_right, range_above, intensity;
    range = _msg->scan().ranges().Get(rayCount + i);
    range_right = _msg->scan().ranges().Get(rayCount + i + 1);
    range_above = _msg->scan().ranges().Get(i);

	ignition::math::Vector3d p_right(0.0, range_right*sin(rayAngle), range_right*cos(rayAngle)-range);
	ignition::math::Vector3d p_above(range_above*sin(rayAngle), 0.0, range_above*cos(rayAngle)-range); // not certain that rayAngle is correct here
	p_right.Normalize();
	p_above.Normalize();
	ignition::math::Vector3d normal = p_right.Cross(p_above);
	normal.Normalize();
	angle = acos(fabs(normal.Z()));

	intensity = _msg->scan().intensities().Get(rayCount + i);

    // Mask ranges outside of min/max to +/- inf, as per REP 117
    /*if (range >= this->RangeMax())
    {
      range = IGN_DBL_INF;
    }
    else if (range <= this->RangeMin())
    {
      range = -IGN_DBL_INF;
    }*/

	// TODO: make these into proper parameters
	double SL = 200.0; // source level
    double TS = double(intensity)*(0.5*M_PI-angle)/M_PI; // target strength, probably dir should be DI
	double TL = 0.5*range; // transmission loss
	double NL = 30; // noise level
	double DI = 0.0; // directivity index 
    double SNR = fmax(SL - 2.0*TL - (NL-DI) + TS, 0.0); // active sonar equation

	//intensity = intensity + 90.0 - 180.0/M_PI*angle;
	//intensity = 180.0/M_PI*angle;
    intensity = int(SNR);

	laser_msg.ranges[i] = range;
	laser_msg.intensities[i] = intensity;
  }
 
  std::cout << "Publihsed scansss" << std::endl;
  gzwarn << "Published scansss... ranges not constructed yet (zero sized)\n";

  this->pub_queue_->push(laser_msg, this->pub_);
}

}
