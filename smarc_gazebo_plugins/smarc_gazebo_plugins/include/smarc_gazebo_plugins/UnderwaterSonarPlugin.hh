/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
 * Desc: Underwater Sonar Plugin
 * Author: Nate Koenig mod by John Hsu and Nils Bore
 */

#ifndef _GAZEBO_UNDERWATER_SONAR_PLUGIN_HH_
#define _GAZEBO_UNDERWATER_SONAR_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <smarc_gazebo_plugins/UnderwaterSonarSensor.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  /// \brief A Ray Sensor Plugin
  class GAZEBO_VISIBLE UnderwaterSonarPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: UnderwaterSonarPlugin();

    /// \brief Destructor
    public: virtual ~UnderwaterSonarPlugin();

    /// \brief Update callback
    public: virtual void OnNewLaserScans();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world;

    /// \brief The parent sensor
	private: std::shared_ptr<sensors::UnderwaterSonarSensor> parentSensor;

    /// \brief The connection tied to UnderwaterSonarPlugin::OnNewLaserScans()
    private: event::ConnectionPtr newLaserScansConnection;
  };
}
#endif // _GAZEBO_UNDERWATER_SONAR_PLUGIN_HH_
