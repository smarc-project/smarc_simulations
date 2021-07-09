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

#ifndef _GAZEBO_POPULATION_HH_
#define _GAZEBO_POPULATION_HH_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <sdf/sdf.hh>
#include "gazebo/common/Console.hh"
#include "ignition/math/Pose3.hh"
#include "ignition/math/Vector3.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Forward declaration of the private data class.
    class PopulationPrivateIntensities;

    /// \brief Stores all the posible parameters that define a population.
    class GAZEBO_VISIBLE PopulationParamsIntensities
    {
      /// \brief The three side lengths of the box.
      public: ignition::math::Vector3d size;

      /// \brief Number of rows used when models are distributed as a grid.
      public: int rows;

      /// \brief Number of columns used when models are distributed as a grid.
      public: int cols;

      /// \brief Distance between models when they are distributed as a grid.
      public: ignition::math::Vector3d step;

      /// The reference frame of the population's region.
      public: ignition::math::Pose3d pose;

      /// \brief Radius of the cylinder's base containing the models.
      public: double radius;

      /// \brief Length of the cylinder containing the models.
      public: double length;

      /// \brief Name of the model.
      public: std::string modelName;

      /// \brief Contains the sdf representation of the model.
      public: std::string modelSdf;

      /// \brief Number of models to spawn.
      public: int modelCount;

      /// \brief Object distribution. E.g.: random, grid.
      public: std::string distribution;

      /// \brief Type region in which the objects will be spawned. E.g.: box.
      public: std::string region;

      /// \brief The intensity of the next new rock.
      public: int intensity;
    };

    /// \class Population Population.hh physics/physics.hh
    /// \brief Class that automatically populates an environment with multiple
    /// objects based on several parameters to define the number of objects,
    /// shape of the object distribution or type of distribution.
    class GAZEBO_VISIBLE PopulationIntensities
    {
      /// \brief Constructor. Load an sdf file containing a population element.
      /// \param[in] _sdf SDF parameters.
      /// \param[in] _world Pointer to the world.
      public: PopulationIntensities(sdf::ElementPtr _sdf, boost::shared_ptr<World> _world);

      /// \brief Destructor.
      public: virtual ~PopulationIntensities();

      /// \brief Generate and spawn multiple populations into the world.
      /// \return True when the populations were successfully spawned or false
      /// otherwise.
      public: bool PopulateAll();

      /// \brief Generate and spawn one model population into the world.
      /// \param[in] _population SDF parameter containing the population details
      /// \return True when the population was successfully spawned or false
      /// otherwise.
      private: bool PopulateOne(const sdf::ElementPtr _population);

      /// \brief Read a value from an SDF element. Before reading the value, it
      /// checks if the element exists and print an error message if not found.
      /// \param[in] _sdfElement SDF element containing the value to read.
      /// \param[in] _element SDF label to read. Ex: "model_count", "min".
      /// \param[out] _value Requested value.
      /// \return True if the element was found or false otherwise.
      private: template<typename T> bool ValueFromSdf(
        const sdf::ElementPtr &_sdfElement, const std::string &_element,
        T &_value)
      {
        if (_sdfElement->HasElement(_element))
        {
          _value = _sdfElement->Get<T>(_element);
          return true;
        }
        gzerr << "Unable to find <" << _element << "> inside the population tag"
              << std::endl;
        return false;
      }

      /// \brief Get a requested SDF element from a SDF. Before returning, it
      /// checks if the element exists and print an error message if not found.
      /// \param[in] _sdfElement SDF element containing the requested SDF.
      /// \param[in] _element SDF label to read. Ex: "model", "box".
      /// \param[out] _value Requested SDF element.
      /// \return True if the element was found or false otherwise.
      private: bool ElementFromSdf(const sdf::ElementPtr &_sdfElement,
        const std::string &_element, sdf::ElementPtr &_value);

      /// \brief Parse the sdf file. Some of the output parameters should be
      /// ignored depending on the region's population. For example, if the
      /// region is a box, the parameters '_center', 'radius', and 'height'
      /// should not be used.
      /// \param[in] _population SDF element containing the Population
      /// description.
      /// \param[out] _params Population parameters parsed from the SDF.
      /// \return True when the function succeed or false otherwise.
      private: bool ParseSdf(sdf::ElementPtr _population,
        PopulationParamsIntensities &_params);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// randomly distributed within a box.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxRandom(const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// uniformly distributed within a box. We use k-means to split the
      /// box in similar subregions.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxUniform(const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \brief Populate a vector of poses evenly placed in a 2D grid pattern.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _rows Number of rows used when the models are
      /// distributed as a 2D grid.
      /// \param[in] _cols Number of columns used when the models are
      /// distributed as a 2D grid.
      /// \param[in] _step Distance between the models when the objects are
      /// distributed as a 2D grid.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxGrid(const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global x-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxLinearX(const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global y-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxLinearY(const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global z-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxLinearZ(const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// randomly distributed within a cylinder.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _center Center of the cylinder's base containing
      /// the models.
      /// \param[in] _radius Radius of the cylinder's base containing
      /// the models
      /// \param[in] _height Height of the cylinder containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesCylinderRandom(
        const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// uniformly distributed within a cylinder. We use k-means to split the
      /// cylinder in similar subregions.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _center Center of the cylinder's base containing
      /// the models.
      /// \param[in] _radius Radius of the cylinder's base containing
      /// the models
      /// \param[in] _height Height of the cylinder containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesCylinderUniform(
        const PopulationParamsIntensities &_populParams,
        std::vector<ignition::math::Vector3d> &_poses);

      /// \internal
      /// \brief Pointer to private data.
      private: boost::scoped_ptr<PopulationPrivateIntensities> dataPtr;
    };
    /// \}
  }
}
#endif
