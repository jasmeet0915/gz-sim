/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GZ_SIM_BLINKVISUAL_SYSTEM_HH_
#define GZ_SIM_BLINKVISUAL_SYSTEM_HH_

#include <memory>

#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief Forward declare private data class.
  class BlinkVisualPrivate;

  /** \class BlinkVisual BlinkVisual.hh \
   * gz/sim/systems/BlinkVisual.hh
  **/
  /// \brief Plugin that makes a visual blink between two colors. This can be used to simulate
  /// LEDs .See the example usage below:
  ///
  /// ## System Parameters:
  ///
  ///    <plugin name="blink" filename="libBlinkVisualPlugin.so">
  ///
  ///      <!-- First RGBA color, each number from 0 to 1. Defaults to red. -->
  ///      <color_a>1 0 0 1</color_a>
  ///
  ///      <!-- Second RGBA color. Defaults to black. -->
  ///      <color_a>0 0 0 1</color_a>
  ///
  ///      <!-- Period in seconds. Defaults to 1 s. -->
  ///      <period>1</period>
  ///
  ///    </plugin>
  ///
  class BlinkVisual:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    /// \brief Constructor.
    public: BlinkVisual();

    /// \brief Destructor.
    public: ~BlinkVisual() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<BlinkVisualPrivate> dataPtr;
  };
}
}
}
}
#endif