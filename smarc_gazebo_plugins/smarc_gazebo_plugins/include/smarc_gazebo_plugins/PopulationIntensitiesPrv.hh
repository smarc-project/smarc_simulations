/*
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
*/

#ifndef _GAZEBO_POPULATIONINTENSITIESPRV_HH_
#define _GAZEBO_POPULATIONINTENSITIESPRV_HH_

#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include "gazebo/physics/World.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Private data for the Population class
    class PopulationPrivateIntensities
    {
      /// \brief The Population's SDF values.
      public: sdf::ElementPtr populationElem;

      /// \brief Pointer to the world.
      public: boost::shared_ptr<World> world;
    };
  }
}
#endif
