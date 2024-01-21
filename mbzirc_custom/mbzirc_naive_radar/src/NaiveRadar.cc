/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>

#include <gz/sensors/Noise.hh>

#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include "NaiveRadar.hh"

using namespace mbzirc;

/////////////////////////////////////////////////
NaiveRadar::NaiveRadar()
{
}

/////////////////////////////////////////////////
NaiveRadar::~NaiveRadar() = default;

//////////////////////////////////////////////////
void NaiveRadar::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  // parse configuration parameters from SDF
  auto sdf = const_cast<sdf::Element *>(_sdf.get());
  this->updateRate = sdf->Get("update_rate", this->updateRate).first;
  if (sdf->HasElement("scan"))
  {
    sdf::ElementPtr scanElem = sdf->GetElement("scan");
    sdf::ElementPtr horElem = scanElem->GetElement("horizontal");
    this->minAngle = horElem->Get("min_angle", this->minAngle).first;
    this->maxAngle = horElem->Get("max_angle", this->maxAngle).first;
    sdf::ElementPtr vertElem = scanElem->GetElement("vertical");
    this->minVerticalAngle =
        vertElem->Get("min_angle", this->minVerticalAngle).first;
    this->maxVerticalAngle =
        vertElem->Get("max_angle", this->maxVerticalAngle).first;
    sdf::ElementPtr rangeElem = scanElem->GetElement("range");
    this->minRange = rangeElem->Get("min", this->minRange).first;
    this->maxRange = rangeElem->Get("max", this->maxRange).first;
    if (scanElem->HasElement("noise"))
    {
      sdf::ElementPtr noiseElem = scanElem->GetElement("noise");
      this->noise = gz::sensors::NoiseFactory::NewNoiseModel(noiseElem);
    }
  }

  // Get top level model this entity belongs to
  // this system is attached to a sensor model but we are only interested
  // in the top level vehicle model pose - this is later used for
  // computing distance to other models in the environment
  auto parent = _ecm.Component<gz::sim::components::ParentEntity>(
      _entity);
  this->entity = _entity;
  this->modelEntity = _entity;
  while (parent && _ecm.Component<gz::sim::components::Model>(
         parent->Data()))
  {
    this->modelEntity = parent->Data();
    parent = _ecm.Component<gz::sim::components::ParentEntity>(
        parent->Data());
  }

  // Get the name of the top-level model
  //  auto modelNameComp = _ecm.Component<gz::sim::components::Name>(this->modelEntity);
    //std::string modelName = "model"; // default name

    //if (modelNameComp)
   // {
      // If the model has a name, use it
      //modelName = modelNameComp->Data();
    //}

    // set topic to publish sensor data to
    std::string topic = "/radar/scan";
    topic = gz::transport::TopicUtils::AsValidTopic(topic);

  // create the publisher
  this->publisher =
    this->node.Advertise<gz::msgs::Float_V>(topic);
}

//////////////////////////////////////////////////
void NaiveRadar::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &_ecm)
{

  // Throttle the sensor updates using sim time
  // If update_rate is set to 0, it means unthrottled
  if (_info.simTime < this->nextUpdateTime && this->updateRate > 0)
    return;

  if (this->updateRate > 0.0)
  {
    // Update the time the plugin should be loaded
    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::duration<double>(1.0 / this->updateRate));
    this->nextUpdateTime += delta;
  }

  // do not bother generating data if there are no subscibers
  if (!this->publisher.HasConnections())
    return;

  // get the pose of the model
  const gz::sim::components::Pose *poseComp =
      _ecm.Component<gz::sim::components::Pose>(this->modelEntity);
  gz::math::Pose3d entityPose = poseComp->Data();

  // compute the inverse of the pose
  // this is used later to convert pose of other entites into this model frame
  auto inversePose = entityPose.Inverse();

  // create vectors to store data that will be published by the sensor
  std::vector<double> ranges;
  std::vector<double> azimuths;
  std::vector<double> elevations;
  std::vector<double> doppler_velocities;

  // Loop through all the models in simulation
  _ecm.Each<gz::sim::components::Model,
            gz::sim::components::Pose,
            gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Model *,
          const gz::sim::components::Pose *_pose,
          const gz::sim::components::ParentEntity *_parent) -> bool
      {
        // skip self
        if (_entity == this->modelEntity)
          return true;
        
       // Get the name of the model
        const gz::sim::components::Name *nameComp =
            _ecm.Component<gz::sim::components::Name>(_entity);

        // If the model has a name
          if (nameComp)
          {
            // Print the name of the model for debugging
            //std::cout << "Model name: " << nameComp->Data() << std::endl;

            // Convert both strings to lower case for case-insensitive comparison
            std::string entityName = nameComp->Data();
            std::transform(entityName.begin(), entityName.end(), entityName.begin(), ::tolower);
            std::vector<std::string> keywords = {"vessel", "car", "vehicle", "human", "person"};
            
            // Flag to indicate if any keyword is found in the entity name
            bool keywordFound = false;

            // Check if the name contains any of the keywords
            for (const auto& keyword : keywords)
            {
              if (entityName.find(keyword) != std::string::npos)
              {
                keywordFound = true;
                break;
              }
            }

            if (keywordFound)
            {
              // The name contains one of the keywords, continue processing
              //std::cout << "Found object of interest" << std::endl;
              ;
            }
            else
            {
              // The name does not contain any of the keywords, skip this model
              return true;
            }
          }
          else
          {
            // The model does not have a name, skip this model
            return true;
          }
                  // ignore nested models by checking to see if it has a parent entity
        // that is also a model
        const gz::sim::components::ParentEntity *parentComp =
            _ecm.Component<gz::sim::components::ParentEntity>(
            _parent->Data());
        if (parentComp &&
            _ecm.Component<gz::sim::components::Model>(
            parentComp->Data()))
          return true;

        // get the model pose
        gz::math::Pose3d pose = _pose->Data();

        // compute range
        double range = entityPose.Pos().Distance(pose.Pos());

        // discard data that are out of range
        if (range > this->maxRange || range < this->minRange)
          return true;

        // rotate entity pose to model frame
        // The position is now also the direction
        gz::math::Vector3d dir = (inversePose * pose).Pos();
        dir.Normalize();

        // compute azimuth and elevation angles
        gz::math::Vector3d xy(dir.X(), dir.Y(), 0.0);
        xy.Normalize();
        double azimuth = std::acos(gz::math::Vector3d::UnitX.Dot(xy));
        azimuth = (dir.Y() < 0) ? -azimuth : azimuth;

        // filter out models that are outside the min/max angles
        if (azimuth > this->maxAngle || azimuth < this->minAngle)
          return true;

        double elevation = std::acos(xy.Dot(dir));
        elevation = (dir.Z() < 0) ? -elevation : elevation;

        // filter out models that are outside the min/max vertical angles
        if (elevation > maxVerticalAngle || elevation < minVerticalAngle)
          return true;


         // get the model pose
        gz::math::Pose3d currentPose = _pose->Data();
        double linearVelocityX = 0.0;
        // compute velocity if previous pose exists for this entity
        if (previousPoses.count(_entity) > 0 && previousTimes.count(_entity) > 0)
        {
          auto currentTime = std::chrono::steady_clock::now();
          auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - previousTimes[_entity]).count() / 1000.0; // convert to seconds

          gz::math::Vector3d velocity = (currentPose.Pos() - previousPoses[_entity].Pos()) / elapsedTime;

          // Print velocity to console
          //std::cout << "Velocity for entity " << _entity << ": " << velocity << std::endl;
          linearVelocityX = velocity.X();
        }
        else
        {
          // If no previous data exists, set velocity to zero
          gz::math::Vector3d velocity(0, 0, 0);
          //std::cout << "Velocity for entity " << _entity << ": " << velocity << std::endl;
          linearVelocityX = velocity.X();

        }
          // store current pose and time for next update
        previousPoses[_entity] = currentPose;
        previousTimes[_entity] = std::chrono::steady_clock::now();

          // apply noise
        if (this->noise)
        {
          range = this->noise->Apply(range);
          azimuth = this->noise->Apply(azimuth);
          elevation = this->noise->Apply(elevation);
          //This noise is too much, add better noise
         // linearVelocityX = this->noise->Apply(linearVelocityX); 
          
        }

        // store results to be published
        ranges.push_back(range);
        azimuths.push_back(azimuth);
        elevations.push_back(elevation);
        doppler_velocities.push_back(linearVelocityX);

        return true;
      });

  // populate and publish the message

  // time stamp the message with sim time
  gz::msgs::Float_V msg;
  *msg.mutable_header()->mutable_stamp() =
      gz::msgs::Convert(_info.simTime);
  auto frame = msg.mutable_header()->add_data();

  // set frame id to scoped name of this sensor
  frame->set_key("frame_id");
  std::string scopedName =
      gz::sim::removeParentScope(
      gz::sim::scopedName(this->entity, _ecm, "::", false), "::");
  frame->add_value(scopedName);

  // populate sensor data
  for (unsigned int i = 0; i < ranges.size(); ++i)
  {
    msg.add_data(ranges[i]);
    msg.add_data(azimuths[i]);
    msg.add_data(elevations[i]);
    msg.add_data(doppler_velocities[i]);
  }
  this->publisher.Publish(msg);
}

// Register the plugin
GZ_ADD_PLUGIN(mbzirc::NaiveRadar,
                    gz::sim::System,
                    NaiveRadar::ISystemConfigure,
                    NaiveRadar::ISystemPostUpdate)
