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
#ifndef MBZIRC_CUSTOMIZATIONS_NAIVERADAR_HH_
#define MBZIRC_CUSTOMIZATIONS_NAIVERADAR_HH_

#include <memory>

#include <sdf/sdf.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/components/Name.hh>
namespace mbzirc
{
  /// \brief A example class to simulate a radar that generates range
  /// and bearing data
  class NaiveRadar:
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: NaiveRadar();

    /// \brief Destructor
    public: ~NaiveRadar() override;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Minimum range (m)
    public: double minRange{1.0};

    /// \brief Maximum range (m)
    public: double maxRange{1000.0};

    /// \brief Minimum angle (rad)
    public: double minAngle{-GZ_PI};

    /// \brief Maximum angle (rad)
    public: double maxAngle{GZ_PI};

    /// \brief Minimum vertical angle (rad)
    public: double minVerticalAngle{-0.1745};

    /// \brief Maximum vertical angle (rad)
    public: double maxVerticalAngle{0.1745};

    /// \brief Sensor update rate
    public: double updateRate{1.0};

    /// \brief Noise to be applied to radar data
    public: gz::sensors::NoisePtr noise;

    /// \brief Entity ID of the sensor
    public: gz::sim::Entity entity{gz::sim::kNullEntity};

    /// \brief Entity ID of the parent model
    public: gz::sim::Entity modelEntity{gz::sim::kNullEntity};

    /// \brief gz tranport node
    public: gz::transport::Node node;

    /// \brief gz transport publisher for publishing sensor data
    public: gz::transport::Node::Publisher publisher;

    /// \brief Sim time when next update should occur
    public: std::chrono::steady_clock::duration nextUpdateTime
        {std::chrono::steady_clock::duration::zero()};
    
    private:
      std::unordered_map<gz::sim::Entity, gz::math::Pose3d> previousPoses;
      std::unordered_map<gz::sim::Entity, std::chrono::steady_clock::time_point> previousTimes;

  };
}

#endif
