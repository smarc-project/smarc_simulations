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
  this->parentSensor->LaserShape().reset();
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
