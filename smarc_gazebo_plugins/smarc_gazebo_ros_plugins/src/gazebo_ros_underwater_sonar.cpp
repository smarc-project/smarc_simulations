/*
 * This file was modified from the original version within Gazebo:
 *
 * Copyright (C) 2014 Open Source Robotics Foundation
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
 * Modifications:
 *
 * Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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

#include <smarc_gazebo_ros_plugins/gazebo_ros_underwater_sonar.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboUnderwaterSonar)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboUnderwaterSonar::GazeboUnderwaterSonar()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboUnderwaterSonar::~GazeboUnderwaterSonar()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboUnderwaterSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  UnderwaterSonarPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::UnderwaterSonarSensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboUnderwaterSonar controller requires a Ray Sensor as its parent");

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
  this->entities_connect_count_ = 0;

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
    boost::bind(&GazeboUnderwaterSonar::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboUnderwaterSonar::LoadThread()
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
      boost::bind(&GazeboUnderwaterSonar::LaserConnect, this),
      boost::bind(&GazeboUnderwaterSonar::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();
    ros::AdvertiseOptions eao =
      ros::AdvertiseOptions::create<smarc_gazebo_ros_plugins::SonarEntities>(
      this->topic_name_+"_entities", 1,
      boost::bind(&GazeboUnderwaterSonar::EntitiesConnect, this),
      boost::bind(&GazeboUnderwaterSonar::EntitiesDisconnect, this),
      ros::VoidPtr(), NULL);
	this->entities_pub_ = this->rosnode_->advertise(eao);
	this->entities_pub_queue_ = this->pmq.addPub<smarc_gazebo_ros_plugins::SonarEntities>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboUnderwaterSonar::EntitiesConnect()
{
  this->entities_connect_count_++;
  if (this->entities_connect_count_ == 1) {
    this->entities_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic() + "_entities",
                                    &GazeboUnderwaterSonar::OnEntities, this);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboUnderwaterSonar::EntitiesDisconnect()
{
  this->entities_connect_count_--;
  if (this->entities_connect_count_ == 0)
    this->entities_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboUnderwaterSonar::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1) {
    this->laser_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                    &GazeboUnderwaterSonar::OnScan, this);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboUnderwaterSonar::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboUnderwaterSonar::OnScan(ConstLaserScanStampedPtr &_msg)
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
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            laser_msg.intensities.begin());
  this->pub_queue_->push(laser_msg, this->pub_);
}

void GazeboUnderwaterSonar::OnEntities(ConstGzString_VPtr &_msg)
{
  smarc_gazebo_ros_plugins::SonarEntities entities_msg;
  for (const std::string& ent : _msg->data()) {
	entities_msg.entities.push_back(ent);
  }
  this->entities_pub_queue_->push(entities_msg, this->entities_pub_);
}

}
