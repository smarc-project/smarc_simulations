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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/algorithm/string.hpp>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Collision.hh>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Exception.hh>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/sensors/SensorFactory.hh>
#include <smarc_gazebo_plugins/UnderwaterSonarSensorPrivate.hh>
#include <smarc_gazebo_plugins/UnderwaterSonarSensor.hh>
#include <smarc_gazebo_plugins/SemanticMultiRayShape.hh>
#include <gazebo/sensors/Noise.hh>
#include <math.h>


using namespace gazebo;
using namespace sensors;

//lofu start
//SONAR PARAMETER SETTINGS
double freq = 340000; //sonar frequency Hz
double SL = 200.0; // source level
double soundVel = 1500;
double lamda = soundVel/freq;
double k = 2*M_PI/lamda ;//wavenumber

//ENVIRONMENTAL PARAMETER SETTINGS
//TL,seastate... etc
double TL_per_km = 90;//90dB/km (from table pg ... TransmissionLoss.pdf
double NL = 40; // noise level seastate0 for f > 2000 Hz from NoiseLevel.pdf (graph pg3)

//default


//lofu end

GZ_REGISTER_STATIC_SENSOR("underwater_sonar", UnderwaterSonarSensor)

//////////////////////////////////////////////////
UnderwaterSonarSensor::UnderwaterSonarSensor()
: Sensor(sensors::RAY),
  dataPtr(new UnderwaterSonarSensorPrivate)
{
}

//////////////////////////////////////////////////
UnderwaterSonarSensor::~UnderwaterSonarSensor()
{
}

//////////////////////////////////////////////////
std::string UnderwaterSonarSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/scan";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void UnderwaterSonarSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  this->dataPtr->scanPub =
    this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);
  this->dataPtr->entityPub =
    this->node->Advertise<msgs::GzString_V>(this->Topic()+"_entities", 50);

  GZ_ASSERT(this->world != NULL,
      "UnderwaterSonarSensor did not get a valid World pointer");

  physics::PhysicsEnginePtr physicsEngine =
    this->world->GetPhysicsEngine();

  GZ_ASSERT(physicsEngine != NULL,
      "Unable to get a pointer to the physics engine");

  this->dataPtr->laserCollision = physicsEngine->CreateCollision("multiray",
      this->ParentName());

  GZ_ASSERT(this->dataPtr->laserCollision != NULL,
      "Unable to create a multiray collision using the physics engine.");

  this->dataPtr->laserCollision->SetName("ray_sensor_collision");
  this->dataPtr->laserCollision->SetRelativePose(this->pose);
  this->dataPtr->laserCollision->SetInitialRelativePose(this->pose);

  this->dataPtr->laserShape =
    boost::dynamic_pointer_cast<physics::MultiRayShape>(
        this->dataPtr->laserCollision->GetShape());

  GZ_ASSERT(this->dataPtr->laserShape != NULL,
      "Unable to get the laser shape from the multi-ray collision.");

  this->dataPtr->laserShape->Load(this->sdf);
  this->dataPtr->laserShape->Init();

  // Handle noise model settings.
  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  if (rayElem->HasElement("noise"))
  {
    this->noises[RAY_NOISE] =
        NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
        this->Type());
  }

  this->dataPtr->parentEntity =
    this->world->GetEntity(this->ParentName());

  GZ_ASSERT(this->dataPtr->parentEntity != NULL,
      "Unable to get the parent entity.");
}

//////////////////////////////////////////////////
void UnderwaterSonarSensor::Init()
{
  Sensor::Init();
  this->dataPtr->laserMsg.mutable_scan()->set_frame(this->ParentName());
}

//////////////////////////////////////////////////
void UnderwaterSonarSensor::Fini()
{
  Sensor::Fini();

  this->dataPtr->scanPub.reset();

  if (this->dataPtr->laserCollision)
  {
    this->dataPtr->laserCollision->Fini();
    this->dataPtr->laserCollision.reset();
  }

  if (this->dataPtr->laserShape)
  {
    this->dataPtr->laserShape->Fini();
    this->dataPtr->laserShape.reset();
  }
}

//////////////////////////////////////////////////
ignition::math::Angle UnderwaterSonarSensor::AngleMin() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetMinAngle().Ign();
  else
    return -1;
}

//////////////////////////////////////////////////
ignition::math::Angle UnderwaterSonarSensor::AngleMax() const
{
  if (this->dataPtr->laserShape)
  {
    return ignition::math::Angle(
        this->dataPtr->laserShape->GetMaxAngle().Radian());
  }
  else
    return -1;
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::GetRangeMin() const
{
  return this->RangeMin();
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::RangeMin() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetMinRange();
  else
    return -1;
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::GetRangeMax() const
{
  return this->RangeMax();
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::RangeMax() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetMaxRange();
  else
    return -1;
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::GetAngleResolution() const
{
  return this->AngleResolution();
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::AngleResolution() const
{
  return (this->AngleMax() - this->AngleMin()).Radian() /
    (this->RangeCount()-1);
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::GetRangeResolution() const
{
  return this->RangeResolution();
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::RangeResolution() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetResRange();
  else
    return -1;
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::GetRayCount() const
{
  return this->RayCount();
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::RayCount() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetSampleCount();
  else
    return -1;
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::GetRangeCount() const
{
  return this->RangeCount();
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::RangeCount() const
{
  // TODO: maybe should check against this->dataPtr->laserMsg.ranges_size()
  //       as users use this to loop through GetRange() calls
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetSampleCount() *
      this->dataPtr->laserShape->GetScanResolution();
  else
    return -1;
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::GetVerticalRayCount() const
{
  return this->VerticalRayCount();
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::VerticalRayCount() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetVerticalSampleCount();
  else
    return -1;
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::GetVerticalRangeCount() const
{
  return this->VerticalRangeCount();
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::VerticalRangeCount() const
{
  if (this->dataPtr->laserShape)
    return this->dataPtr->laserShape->GetVerticalSampleCount() *
      this->dataPtr->laserShape->GetVerticalScanResolution();
  else
    return -1;
}

//////////////////////////////////////////////////
ignition::math::Angle UnderwaterSonarSensor::VerticalAngleMin() const
{
  if (this->dataPtr->laserShape)
  {
    return ignition::math::Angle(
        this->dataPtr->laserShape->GetVerticalMinAngle().Radian());
  }
  else
    return -1;
}

//////////////////////////////////////////////////
ignition::math::Angle UnderwaterSonarSensor::VerticalAngleMax() const
{
  if (this->dataPtr->laserShape)
  {
    return ignition::math::Angle(
        this->dataPtr->laserShape->GetVerticalMaxAngle().Radian());
  }
  else
    return -1;
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::GetVerticalAngleResolution() const
{
  return this->VerticalAngleResolution();
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::VerticalAngleResolution() const
{
  return (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian() /
    (this->VerticalRangeCount()-1);
}

//////////////////////////////////////////////////
void UnderwaterSonarSensor::GetRanges(std::vector<double> &_ranges)
{
  this->Ranges(_ranges);
}

//////////////////////////////////////////////////
void UnderwaterSonarSensor::Ranges(std::vector<double> &_ranges) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  _ranges.resize(this->dataPtr->laserMsg.scan().ranges_size());
  memcpy(&_ranges[0], this->dataPtr->laserMsg.scan().ranges().data(),
         sizeof(_ranges[0]) * this->dataPtr->laserMsg.scan().ranges_size());
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::GetRange(unsigned int _index)
{
  return this->Range(_index);
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::Range(const unsigned int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->laserMsg.scan().ranges_size() == 0)
  {
    gzwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (static_cast<int>(_index) >= this->dataPtr->laserMsg.scan().ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->dataPtr->laserMsg.scan().ranges(_index);
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::GetRetro(unsigned int _index)
{
  return this->Retro(_index);
}

//////////////////////////////////////////////////
double UnderwaterSonarSensor::Retro(const unsigned int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->laserMsg.scan().intensities_size() == 0)
  {
    gzwarn << "Intensities not constructed yet (zero size)\n";
    return 0.0;
  }
  if (static_cast<int>(_index) >=
      this->dataPtr->laserMsg.scan().intensities_size())
  {
    gzerr << "Invalid intensity index[" << _index << "]\n";
    return 0.0;
  }

  return this->dataPtr->laserMsg.scan().intensities(_index);
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::GetFiducial(unsigned int _index)
{
  return this->Fiducial(_index);
}

//////////////////////////////////////////////////
int UnderwaterSonarSensor::Fiducial(const unsigned int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Convert range index to ray index.
  // Find vertical/horizontal range indices (vIdx, hIdx) and mulitply
  // by the ratio of ray count to range count to get the vertical/horizontal
  // ray indices, which are then used to compute the final index into ray array.
  int vIdx = _index / this->RangeCount();
  vIdx = vIdx * this->VerticalRayCount() / this->VerticalRangeCount();
  int hIdx = _index % this->RangeCount();
  hIdx = hIdx * this->RayCount() / this->RangeCount();
  int idx = vIdx * this->RayCount()  + hIdx;

  if (idx >= this->RayCount() * this->VerticalRayCount())
  {
    gzerr << "Invalid fiducial index[" << _index << "]\n";
    return 0.0;
  }
  return this->dataPtr->laserShape->GetFiducial(idx);
}

//////////////////////////////////////////////////
bool UnderwaterSonarSensor::UpdateImpl(const bool /*_force*/)
{
  // do the collision checks
  // this eventually call OnNewScans, so move mutex lock behind it in case
  // need to move mutex lock after this? or make the OnNewLaserScan connection
  // call somewhere else?
  this->dataPtr->laserShape->Update();
  this->lastMeasurementTime = this->world->GetSimTime();

  // moving this behind laserShape update
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  msgs::Set(this->dataPtr->laserMsg.mutable_time(),
            this->lastMeasurementTime);
  //msgs::Set(this->dataPtr->entityMsg.mutable_time(),
  //          this->lastMeasurementTime);

  msgs::LaserScan *scan = this->dataPtr->laserMsg.mutable_scan();
  msgs::GzString_V *entities = &this->dataPtr->entityMsg;

  // Store the latest laser scans into laserMsg
  msgs::Set(scan->mutable_world_pose(),
      this->pose + this->dataPtr->parentEntity->GetWorldPose().Ign());
  scan->set_angle_min(this->AngleMin().Radian());
  scan->set_angle_max(this->AngleMax().Radian());
  scan->set_angle_step(this->AngleResolution());
  scan->set_count(this->RangeCount());

  scan->set_vertical_angle_min(this->VerticalAngleMin().Radian());
  scan->set_vertical_angle_max(this->VerticalAngleMax().Radian());
  if (this->VerticalRangeCount() == 1 && this->VerticalRayCount() > 1) {
    scan->set_vertical_angle_step((this->VerticalAngleMax() - this->VerticalAngleMin()).Radian());
  }
  else {
    scan->set_vertical_angle_step(this->VerticalAngleResolution());
  }
  scan->set_vertical_count(this->VerticalRangeCount());

  scan->set_range_min(this->RangeMin());
  scan->set_range_max(this->RangeMax());

  scan->clear_ranges();
  scan->clear_intensities();
  entities->clear_data();

  unsigned int rayCount = this->RayCount();
  unsigned int rangeCount = this->RangeCount();
  unsigned int verticalRayCount = this->VerticalRayCount();
  unsigned int verticalRangeCount = this->VerticalRangeCount();
  double rayAngle = this->AngleResolution();

  /*
  printf("Vertical rays: %u", verticalRayCount);
  printf("Vertical ranges: %u", verticalRangeCount);
  printf("Vertical min angle: %f", this->VerticalAngleMin().Radian());
  printf("Vertical max angle: %f", this->VerticalAngleMax().Radian());
  printf("Vertical angle step: %f", this->VerticalAngleResolution());
  printf("Horizontal rays: %u", rayCount);
  printf("Horizontal ranges: %u", rangeCount);
  printf("Horizontal angle step: %f", this->AngleResolution());
  */
  GZ_ASSERT(verticalRayCount == 2 && verticalRangeCount == 1,
          "Vertical ray count needs to be 2 for angle interpolation");
  GZ_ASSERT(rayCount == rangeCount,
          "Horizontal ray count needs to be the same as ranges");

  // interpolate in horizontal direction
  for (unsigned int i = 0; i < rangeCount-1; ++i) // TODO: fix the last one eventually
  {
    double angle, range, range_right, range_above, intensity;
    range = this->dataPtr->laserShape->GetRange(this->RayCount() + i);
    range_right = this->dataPtr->laserShape->GetRange(this->RayCount() + i + 1);
    range_above = this->dataPtr->laserShape->GetRange(i);

	ignition::math::Vector3d p_right(0.0, range_right*sin(rayAngle), range_right*cos(rayAngle)-range);
	ignition::math::Vector3d p_above(range_above*sin(rayAngle), 0.0, range_above*cos(rayAngle)-range); // not certain that rayAngle is correct here
	p_right.Normalize();
	p_above.Normalize();
	ignition::math::Vector3d normal = p_right.Cross(p_above);
	normal.Normalize();
	angle = acos(fabs(normal.Z()));

	intensity = this->dataPtr->laserShape->GetRetro(this->RayCount() + i);
    physics::RayShapePtr ray = physics::SemanticMultiRayShape::StaticGetRay(this->dataPtr->laserShape, this->RayCount() + i);
    double _dist;
    std::string _entity;
    ray->GetIntersection(_dist, _entity);
    //printf("%s", _entity.c_str());




    // Mask ranges outside of min/max to +/- inf, as per REP 117
    if (range >= this->RangeMax())
    {
      range = IGN_DBL_INF;
    }
    else if (range <= this->RangeMin())
    {
      range = -IGN_DBL_INF;
    }
    else if (this->noises.find(RAY_NOISE) !=
             this->noises.end())
    {
      // currently supports only one noise model per laser sensor
	  range = this->noises[RAY_NOISE]->Apply(range);
      range = ignition::math::clamp(range,
              this->RangeMin(), this->RangeMax());
    }

	// TODO: make these into proper parameters
        //double SL = 200.0; // source level
        double TS = 0.0; //double(intensity)*(0.5*M_PI-angle)/M_PI; // target strength, probably dir should be DI
        double TL = TL_per_km*range/1000; // 0.5*range; // transmission loss
	double DI = 0.0; // directivity index 
        double SNR = fmax(SL - 2.0*TL - (NL-DI) + TS, 0.0); // active sonar equation

        //lofu start test

        //std::cout<< _entity<< std::endl;
        //std::cout << "..." << std::endl;
       // printf("%s", _entity.c_str());


        printf("%s\n", _entity.c_str());

        //See if entities contain .. unit_cylinder or large_rock then set TS otherwise...
        std::string s1 = _entity.c_str();
        std::string s2 = "large_rock";
        std::string s3 = "cylinder";
        if (s1.find(s2) != std::string::npos) {
                std::cout << "large rock found!" << '\n';
                double SNR = fmax(SL - 2.0*TL - (NL-DI) + TS, 0.0); // active sonar equation

            }
        if (s1.find(s3) != std::string::npos) {
                std::cout << "cylinder found!" << '\n';
                double a = 1; // unit cylider radius
                double L = 1; //length cylinder
                double beta = k*L*sin(angle);
                double TS = (a*pow(L,2)*pow(sin(beta)/beta, 2)*pow(cos(angle),2))/(2*lamda); //formula for finite cylinder with ka >> 1
                double SNR = fmax(SL - 2.0*TL - (NL-DI) + TS, 0.0); // active sonar equation
            }


        //lofu end test

	//intensity = intensity + 90.0 - 180.0/M_PI*angle;
	//intensity = 180.0/M_PI*angle;
    intensity = int(SNR);

    scan->add_ranges(range);
    scan->add_intensities(intensity);
	entities->add_data(_entity);
  }

  this->dataPtr->scanPub->Publish(this->dataPtr->laserMsg);
if (this->dataPtr->entityPub && this->dataPtr->entityPub->HasConnections())
  if (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections())
    this->dataPtr->entityPub->Publish(this->dataPtr->entityMsg);

  return true;
}

//////////////////////////////////////////////////
bool UnderwaterSonarSensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections());
}

//////////////////////////////////////////////////
physics::MultiRayShapePtr UnderwaterSonarSensor::GetLaserShape() const
{
  return this->LaserShape();
}

//////////////////////////////////////////////////
physics::MultiRayShapePtr UnderwaterSonarSensor::LaserShape() const
{
  return this->dataPtr->laserShape;
}
