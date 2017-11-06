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
#include <functional>

#include <gazebo/physics/physics.hh>
#include <smarc_gazebo_plugins/UnderwaterSonarPlugin.hh>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(UnderwaterSonarPlugin)

/////////////////////////////////////////////////
UnderwaterSonarPlugin::UnderwaterSonarPlugin()
{
}

/////////////////////////////////////////////////
UnderwaterSonarPlugin::~UnderwaterSonarPlugin()
{
  this->parentSensor->LaserShape()->DisconnectNewLaserScans(
      this->newLaserScansConnection);
  this->newLaserScansConnection.reset();

  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void UnderwaterSonarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Get then name of the parent sensor
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::UnderwaterSonarSensor>(_parent);

  if (!this->parentSensor)
    gzthrow("UnderwaterSonarPlugin requires a Ray Sensor as its parent");

  this->world = physics::get_world(this->parentSensor->WorldName());

  this->newLaserScansConnection =
    this->parentSensor->LaserShape()->ConnectNewLaserScans(
      std::bind(&UnderwaterSonarPlugin::OnNewLaserScans, this));
}

/////////////////////////////////////////////////
void UnderwaterSonarPlugin::OnNewLaserScans()
{
  /* overload with useful callback here */
}
