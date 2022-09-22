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

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
//#include <math/gzmath.hh>
//#include "gazebo/math/Kmeans.hh"
#include "ignition/math/Kmeans.hh"
#include "ignition/math/Pose3.hh"
#include "ignition/math/Rand.hh"
#include "gazebo/physics/World.hh"
#include "smarc_gazebo_plugins/PopulationIntensities.hh"
#include "smarc_gazebo_plugins/PopulationIntensitiesPrv.hh"

#include "boost/lexical_cast.hpp"

using namespace gazebo;
using namespace common;
using namespace physics;
using namespace  std;

//////////////////////////////////////////////////
PopulationIntensities::PopulationIntensities(sdf::ElementPtr _sdf, boost::shared_ptr<World> _world)
  : dataPtr(new PopulationPrivateIntensities)
{
  this->dataPtr->world = _world;
  if (_sdf->HasElement("population"))
    this->dataPtr->populationElem = _sdf->GetElement("population");
}

//////////////////////////////////////////////////
PopulationIntensities::~PopulationIntensities()
{
}

//////////////////////////////////////////////////
bool PopulationIntensities::PopulateAll()
{
  GZ_ASSERT(this->dataPtr->populationElem, "<population> SDF element is NULL");

  sdf::ElementPtr popElem = this->dataPtr->populationElem;
  bool result = true;

  // Iterate through all the population elements in the sdf.
  while (popElem)
  {
    if (!this->PopulateOne(popElem))
      result = false;
    popElem = popElem->GetNextElement("population");
  }

  return result;
}

//////////////////////////////////////////////////
bool PopulationIntensities::PopulateOne(const sdf::ElementPtr _population)
{
  std::vector<ignition::math::Vector3d> objects;
  PopulationParamsIntensities params;

  GZ_ASSERT(_population, "'_population' parameter is NULL");

  if (!this->ParseSdf(_population, params))
    return false;

  // Generate the set of poses based on the region and distribution.
  if (params.region == "box" && params.distribution == "random")
    this->CreatePosesBoxRandom(params, objects);
  else if (params.region == "box" && params.distribution == "uniform")
    this->CreatePosesBoxUniform(params, objects);
  else if (params.distribution == "grid")
    this->CreatePosesBoxGrid(params, objects);
  else if (params.region == "box" && params.distribution == "linear-x")
    this->CreatePosesBoxLinearX(params, objects);
  else if (params.region == "box" && params.distribution == "linear-y")
    this->CreatePosesBoxLinearY(params, objects);
  else if (params.region == "box" && params.distribution == "linear-z")
    this->CreatePosesBoxLinearZ(params, objects);
  else if (params.region == "cylinder" && params.distribution == "random")
    this->CreatePosesCylinderRandom(params, objects);
  else if (params.region == "cylinder" && params.distribution == "uniform")
    this->CreatePosesCylinderUniform(params, objects);
  else
  {
    gzerr << "Unrecognized combination of region [" << params.region << "] and "
          << "distribution [" << params.distribution << "]" << std::endl;
    return false;
  }

  // Create an sdf containing the model description.
  sdf::SDF sdf;
  sdf.SetFromString("<sdf version ='1.5'>" + params.modelSdf + "</sdf>");

  for (size_t i = 0; i < objects.size(); ++i)
  {
    ignition::math::Vector3d p(objects[i].X(), objects[i].Y(), objects[i].Z());

    // Create a unique model for each clone.
    std::string cloneSdf = sdf.ToString();
    std::string delim = "model name='";
    std::string intensity = "laser_retro='";
    size_t first = cloneSdf.find(delim) + delim.size();
    size_t last = cloneSdf.find("'", first);
    std::string newName = params.modelName + std::string("_clone_") +
      boost::lexical_cast<std::string>(i);
    cloneSdf.replace(first, last - first, newName);

    // Insert the <pose> element.
    std::string endDelim = "'>";
    first = cloneSdf.find(delim) + delim.size();
    last = cloneSdf.find(endDelim, first);
    std::string pose = "\n    <pose>" +
      boost::lexical_cast<std::string>(p.X()) + " " +
      boost::lexical_cast<std::string>(p.Y()) + " " +
      boost::lexical_cast<std::string>(p.Z()) + " 0 0 0</pose>";
    cloneSdf.insert(last + endDelim.size(), pose);

    this->dataPtr->world->InsertModelString(cloneSdf);
  }

  return true;
}

/////////////////////////////////////////////////
bool PopulationIntensities::ElementFromSdf(const sdf::ElementPtr &_sdfElement,
  const std::string &_element, sdf::ElementPtr &_value)
{
  if (_sdfElement->HasElement(_element))
  {
    _value = _sdfElement->GetElement(_element);
    return true;
  }
  gzerr << "Unable to find <" << _element << "> inside the population tag"
        << std::endl;
  return false;
}

/////////////////////////////////////////////////
bool PopulationIntensities::ParseSdf(sdf::ElementPtr _population,
  PopulationParamsIntensities &_params)
{
  GZ_ASSERT(_population, "'_population' parameter is NULL");

  // Read the model element.
  sdf::ElementPtr model;
  if (!this->ElementFromSdf(_population, "model", model))
    return false;

  _params.modelSdf = model->ToString("");
  _params.modelName = model->Get<std::string>("name");

  // Read the pose.
  ignition::math::Pose3d pose;
  if (!this->ValueFromSdf(_population, "pose", _params.pose))
    return false;

  // Read the initial intensity
  if (!this->ValueFromSdf(_population, "initial_intensity", _params.intensity))
    return false;

  // Read the distribution element.
  sdf::ElementPtr distribution;
  if (!this->ElementFromSdf(_population, "distribution", distribution))
    return false;

  // Read the distribution type.
  if (!this->ValueFromSdf<std::string>(distribution, "type",
    _params.distribution))
  {
    return false;
  }

  if ((_params.distribution != "random")   &&
      (_params.distribution != "uniform")  &&
      (_params.distribution != "grid")     &&
      (_params.distribution != "linear-x") &&
      (_params.distribution != "linear-y") &&
      (_params.distribution != "linear-z"))
  {
    gzerr << "Unknown distribution type [" << _params.distribution << "]"
          << std::endl;
    return false;
  }

  // Models evenly distributed in a 2D grid pattern.
  if (_params.distribution == "grid")
  {
    // Read the number of rows.
    if (!this->ValueFromSdf<int>(distribution, "rows", _params.rows))
      return false;

    // Sanity check.
    if (_params.rows <= 0)
    {
      gzwarn << "Incorrect number of rows while populating objects ["
             << _params.rows << "]. Population ignored." << std::endl;
      return false;
    }

    // Read the number of columns.
    if (!this->ValueFromSdf<int>(distribution, "cols", _params.cols))
      return false;

    // Sanity check.
    if (_params.cols <= 0)
    {
      gzwarn << "Incorrect number of columns while populating objects ["
             << _params.cols << "]. Population ignored." << std::endl;
      return false;
    }

    // Read the <step> value used to separate each model in the grid.
    if (!this->ValueFromSdf<ignition::math::Vector3d>(distribution, "step", _params.step))
      return false;

    // Align the origin of the grid with 'pose'.
    if (_params.cols % 2 == 0)
    {
      _params.pose.Pos().X() -=
        (_params.step.X() * (_params.cols - 2) / 2.0) + (_params.step.X() / 2.0);
    }
    else
      _params.pose.Pos().X() -= _params.step.X() * (_params.cols - 1) / 2.0;

    if (_params.rows % 2 == 0)
    {
      _params.pose.Pos().Y() -=
        (_params.step.Y() * (_params.rows - 2) / 2.0) + (_params.step.Y() / 2.0);
    }
    else
      _params.pose.Pos().Y() -= _params.step.Y() * (_params.rows - 1) / 2.0;
  }
  else
  {
    // Read the model_count element.
    if (!this->ValueFromSdf<int>(_population, "model_count",
      _params.modelCount))
    {
      return false;
    }

    // Sanity check.
    if (_params.modelCount <= 0)
    {
      gzwarn << "Trying to populate a non positive number of models ["
             << _params.modelCount << "]. Population ignored." << std::endl;
      return false;
    }

    // Read the region element.
    if (_population->HasElement("box"))
    {
      sdf::ElementPtr box = _population->GetElement("box");
      _params.region = "box";

      // Read the size of the bounding box.
      if (!this->ValueFromSdf<ignition::math::Vector3d>(box, "size", _params.size))
        return false;

      // Sanity check.
      if (_params.size.X() <= 0 || _params.size.Y() <= 0 || _params.size.Z() <= 0)
      {
        gzwarn << "Incorrect box size while populating objects ["
               << _params.size << "]. Population ignored." << std::endl;
        return false;
      }

      // Align the origin of the box with 'pose'.
      _params.pose.Pos() -= _params.size / 2.0;
    }
    else if (_population->HasElement("cylinder"))
    {
      sdf::ElementPtr cylinder = _population->GetElement("cylinder");
      _params.region = "cylinder";

      // Read the radius of the cylinder's base.
      if (!this->ValueFromSdf<double>(cylinder, "radius", _params.radius))
        return false;

      // Sanity check.
      if (_params.radius <= 0)
      {
        gzwarn << "Incorrect radius value while populating objects ["
               << _params.radius << "]. Population ignored." << std::endl;
        return false;
      }

      // Read the cylinder's length.
      if (!this->ValueFromSdf<double>(cylinder, "length", _params.length))
        return false;

      // Sanity check.
      if (_params.length <= 0)
      {
        gzwarn << "Incorrect length value while populating objects ["
               << _params.length << "]. Population ignored." << std::endl;
        return false;
      }
    }
    else
    {
      gzerr << "I have not found a valid region. 'box' or 'cylinder' are"
            << " the valid region types" << std::endl;
      return false;
    }
  }

  return true;
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesBoxRandom(const PopulationParamsIntensities &_populParams,
  std::vector<ignition::math::Vector3d> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  _poses.clear();
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    ignition::math::Pose3d offset(ignition::math::Rand::DblUniform(0, _populParams.size.X()),
                      ignition::math::Rand::DblUniform(0, _populParams.size.Y()),
                      ignition::math::Rand::DblUniform(0, _populParams.size.Z()),
                      0, 0, 0);

    _poses.push_back((offset + _populParams.pose).Pos());
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesBoxUniform(const PopulationParamsIntensities &_populParams,
  std::vector<ignition::math::Vector3d> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  std::vector<ignition::math::Vector3d> obs;

  // Step1: Sample points in a box.
  double x = 0.0;
  double y = 0.0;
  while (y < _populParams.size.Y())
  {
    while (x < _populParams.size.X())
    {
      ignition::math::Vector3d p (x,y, ignition::math::Rand::DblUniform(0, _populParams.size.Z()));
      //p.X() = x;
      //p.Y() = y;
      //p.Z() = ignition::math::Rand::DblUniform(0, _populParams.size.Z());
      obs.push_back(p);
      x += .1;
    }
    x = 0.0;
    y += .1;
  }

  // Step2: Cluster the sampled points in 'modelCount' clusters.
  std::vector<ignition::math::Vector3d> centroids;
  std::vector<unsigned int> labels;
  ignition::math::Kmeans kmeans(obs);
  kmeans.Cluster(_populParams.modelCount, centroids, labels);

  // Step3: Create the list of object positions.
  _poses.clear();
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    ignition::math::Pose3d p(centroids[i], ignition::math::Quaterniond(0, 0, 0));
    _poses.push_back((p + _populParams.pose).Pos());
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesBoxGrid(const PopulationParamsIntensities &_populParams,
  std::vector<ignition::math::Vector3d> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  _poses.clear();
  ignition::math::Pose3d offset = ignition::math::Pose3d::Zero;
  for (int i = 0; i < _populParams.rows; ++i)
  {
    for (int j = 0; j < _populParams.cols; ++j)
    {
      _poses.push_back((offset + _populParams.pose).Pos());
      offset.Pos().X() += _populParams.step.X();
    }
    offset.Pos().X() = 0;
    offset.Pos().Y() += _populParams.step.Y();
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.rows * _populParams.cols ==
    static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesBoxLinearX(const PopulationParamsIntensities &_populParams,
  std::vector<ignition::math::Vector3d> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  // Evenly placed in a row along the global x-axis.
  _poses.clear();
 ignition::math::Pose3d offset = ignition::math::Pose3d::Zero;
  offset.Pos().Y() = _populParams.size.Y() / 2.0;
  offset.Pos().Z() = _populParams.size.Z() / 2.0;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.Pos().X() =
      _populParams.size.X() * i / static_cast<double>(_populParams.modelCount);
    _poses.push_back((offset + _populParams.pose).Pos());
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesBoxLinearY(const PopulationParamsIntensities &_populParams,
  std::vector<ignition::math::Vector3d> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  // Evenly placed in a row along the global y-axis.
  _poses.clear();
  ignition::math::Pose3d offset = ignition::math::Pose3d::Zero;
  offset.Pos().X() = _populParams.size.X() / 2.0;
  offset.Pos().Z() = _populParams.size.Z() / 2.0;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.Pos().Y() =
      _populParams.size.Y() * i / static_cast<double>(_populParams.modelCount);
    _poses.push_back((offset + _populParams.pose).Pos());
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesBoxLinearZ(const PopulationParamsIntensities &_populParams,
  std::vector<ignition::math::Vector3d> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  // Evenly placed in a row along the global z-axis.
  _poses.clear();
  ignition::math::Pose3d offset = ignition::math::Pose3d::Zero;
  offset.Pos().X() = _populParams.size.X() / 2.0;
  offset.Pos().Y() = _populParams.size.Y() / 2.0;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.Pos().Z() =
      _populParams.size.Z() * i / static_cast<double>(_populParams.modelCount);
    _poses.push_back((offset + _populParams.pose).Pos());
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesCylinderRandom(const PopulationParamsIntensities &_populParams,
  std::vector<ignition::math::Vector3d> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  _poses.clear();
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    double ang = ignition::math::Rand::DblUniform(0, 2 * M_PI);
    double r = ignition::math::Rand::DblUniform(0, _populParams.radius);
    ignition::math::Pose3d offset = ignition::math::Pose3d::Zero;
    offset.Pos().X() = r * cos(ang);
    offset.Pos().Y() = r * sin(ang);
    offset.Pos().Z() = ignition::math::Rand::DblUniform(0, _populParams.length);
    _poses.push_back((offset + _populParams.pose).Pos());
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void PopulationIntensities::CreatePosesCylinderUniform(
  const PopulationParamsIntensities &_populParams, std::vector<ignition::math::Vector3d>  &_poses){
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  std::vector<ignition::math::Vector3d> obs;

  // Step1: Sample points in the cylinder.
  unsigned int points = 10000;
  
  for (size_t i = 0; i < points; ++i)
  {
    double ang = ignition::math::Rand::DblUniform(0, 2 * M_PI);
    double r = ignition::math::Rand::DblUniform(0, _populParams.radius);
    ignition::math::Vector3d p (r * cos(ang), r * sin(ang), ignition::math::Rand::DblUniform(0, _populParams.length));
    //p.x = r * cos(ang);
    //p.y = r * sin(ang);
    //p.z = ignition::math::Rand::DblUniform(0, _populParams.length);
    obs.push_back(p);
  }

  // Step2: Cluster the sampled points in 'modelCount' clusters.
  std::vector<ignition::math::Vector3d> centroids;
  std::vector<unsigned int> labels;
  ignition::math::Kmeans kmeans(obs);
  kmeans.Cluster(_populParams.modelCount, centroids, labels);

  // Step3: Create the list of object positions.
  _poses.clear();
  ignition::math::Pose3d offset = ignition::math::Pose3d::Zero;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.Pos() = centroids[i];
    _poses.push_back((offset + _populParams.pose).Pos());
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}
