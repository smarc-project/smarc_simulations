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

#ifndef GAZEBO_UNDERWATER_SONAR_RAY_HH
#define GAZEBO_UNDERWATER_SONAR_RAY_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <smarc_gazebo_plugins/UnderwaterSonarPlugin.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class GazeboUnderwaterSonarRay : public RayPlugin
  {
    /// \brief Constructor
    public: GazeboUnderwaterSonarRay();

    /// \brief Destructor
    public: ~GazeboUnderwaterSonarRay();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    // Pointer to the model
    GazeboRosPtr gazebo_ros_;
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    /// \brief The parent sensor
	private: std::shared_ptr<sensors::RaySensor> parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief tf prefix
    private: std::string tf_prefix_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: void OnScan(ConstLaserScanStampedPtr &_msg);

    /// \brief prevents blocking
    private: PubMultiQueue pmq;
  };
}
#endif // GAZEBO_UNDERWATER_SONAR_RAY_HH
