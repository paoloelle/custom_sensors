/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <math.h>

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/sensors/Noise.hh>
#include <ignition/sensors/Util.hh>

#include "LightSensor.hh"

using namespace custom;

  ignition::math::v6::Pose3d sensorPose;

  ignition::math::v6::Pose3d lightPose;

  std::double_t lightValue;

//////////////////////////////////////////////////
bool LightSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = ignition::sensors::customType(_sdf);
  if ("light_sensor" != type)
  {
    ignerr << "Trying to load [light_sensor] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  ignition::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<ignition::msgs::Double>(this->Topic());

  if (!_sdf.Element()->HasElement("ignition:light_sensor"))
  {
    igndbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("ignition:light_sensor");

  if (!customElem->HasElement("noise"))
  {
    igndbg << "No noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise noiseSdf;
  noiseSdf.Load(customElem->GetElement("noise"));
  this->noise = ignition::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    ignerr << "Failed to load noise." << std::endl;
    return false;
  }

  // get sensor's position
  sensorPose = _sdf.RawPose(); 

  // get light position
  // TODO put a while cycle to go to the world entity (there is a method to check the name of the entity)
  // so when the name of the entity is "world" stop while cycle
  auto parentElement = _sdf.Element()->GetParent()->GetParent()->GetParent(); // get world pointer
  auto lightElement = parentElement->FindElement("light");
  lightPose = lightElement->Get<ignition::math::Pose3d>("pose");
  
  //std::cout << lightPose << std::endl; 


  return true;

}

//////////////////////////////////////////////////
bool LightSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  ignition::msgs::Double msg;
  *msg.mutable_header()->mutable_stamp() = ignition::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  //this->totalDistance = this->noise->Apply(this->totalDistance);

 // msg.set_data(this->totalDistance);

  msg.set_data(lightValue);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  

  return true;
}

//////////////////////////////////////////////////
void LightSensor::NewPosition(const ignition::math::Vector3d &_pos)
{
  //if (!isnan(this->prevPos.X()))
  //{
    lightValue = lightPose.Pos().Distance(_pos);
    lightValue = (lightValue);
    //this->totalDistance += this->prevPos.Distance(_pos);
    //std::cout << lightValue << std::endl;
  //}
  //this->prevPos = _pos;
}

//////////////////////////////////////////////////
const ignition::math::Vector3d &LightSensor::Position() const
{
  return this->prevPos;
}
