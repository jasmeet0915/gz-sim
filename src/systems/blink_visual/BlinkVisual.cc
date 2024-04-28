/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include <mutex>
#include <chrono>

#include <gz/plugin/Register.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/RenderingIface.hh>

#include <gz/math/Color.hh>

#include <gz/transport/Node.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/rendering/Events.hh>

#include "BlinkVisual.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace std::chrono_literals;

// Private data class
class gz::sim::systems::BlinkVisualPrivate
{
  /// \brief Callback invoked by the rendering thread before a render update
  public: void OnPreRender();

  /// \brief Function used to connect to the pre render event
  public: void ConnectToPreRender();

  /// \brief Connection to the pre-render event
  public: common::ConnectionPtr preRenderConn;

  /// \brief Pointer to EventManager
  public: EventManager *eventMgr{nullptr};

  /// \brief Entity of the model to blink
  public: Entity visualEntity;

  /// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene{nullptr};

  /// \brief Visual whose color will be changed.
  public: rendering::VisualPtr visual{nullptr};

  /// \brief Pointer to the Material of the visual
  public: rendering::MaterialPtr material{nullptr};

  /// \brief First color.
  public: math::Color colorA{math::Color(1, 0, 0, 1)};

  /// \brief Second color.
  public: math::Color colorB{math::Color(1, 1, 1, 1)};

  /// \brief Color according the current elapsed time
  public: math::Color currentColor{math::Color(1, 1, 1, 1)};

  /// \brief Time duration for which color A is on in seconds
  public: std::chrono::duration<double> aOnTime{1s};

  /// \brief Time Duration for which color B is on in seconds
  public: std::chrono::duration<double> bOnTime{1s};

  /// \brief Time period of the LED blink sequence which A on time + B on time
  public: std::chrono::duration<double> period{0s};

  /// \brief Time the current cycle started.
  public: std::chrono::duration<double> cycleStartTime{0s};

  /// \brief The current simulation time.
  public: std::chrono::duration<double> currentSimTime{0s};

  /// \brief Mutex to protect sim time updates
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
BlinkVisual::BlinkVisual()
    : System(), dataPtr(std::make_unique<BlinkVisualPrivate>())
{
}

/////////////////////////////////////////////////
BlinkVisual::~BlinkVisual()
{
}

/////////////////////////////////////////////////
void BlinkVisual::Configure(
  const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &, gz::sim::EventManager &_eventMgr)
{
  this->dataPtr->visualEntity = _entity;
  this->dataPtr->eventMgr = &_eventMgr;

  // Get color A
  if (_sdf->HasElement("color_a"))
  {
    this->dataPtr->colorA = _sdf->Get<gz::math::Color>("color_a");
  }

  // Get color B
  if (_sdf->HasElement("color_b"))
  {
    this->dataPtr->colorB = _sdf->Get<gz::math::Color>("color_b");
  }

  // Get the On time for color A
  if (_sdf->HasElement("a_on_time"))
  {
    this->dataPtr->aOnTime = std::chrono::duration<double>(_sdf->Get<double>("a_on_time"));
  }

  // Get the On time for color B
  if (_sdf->HasElement("b_on_time"))
  {
    this->dataPtr->bOnTime = std::chrono::duration<double>(_sdf->Get<double>("b_on_time"));
  }

  if (this->dataPtr->aOnTime.count() <= 0 || this->dataPtr->bOnTime.count() <= 0)
  {
    gzerr << "On time for any of the Colors cannot be 0 or negative" << std::endl;
    return;
  }

  // Set the period as the total of A on Time and B on Time
  this->dataPtr->period = this->dataPtr->aOnTime + this->dataPtr->bOnTime;

  // Connect to the pre render event
  this->dataPtr->preRenderConn = this->dataPtr->eventMgr->Connect<events::PreRender>(
      std::bind(&BlinkVisualPrivate::OnPreRender, this->dataPtr.get()));
}

/////////////////////////////////////////////////
void BlinkVisual::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &)
{
  // Set the current sim time in the PreUpdate function
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime =
    std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(_info.simTime);
}

/////////////////////////////////////////////////
void BlinkVisualPrivate::OnPreRender()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Get a pointer to the rendering scene
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene->IsInitialized())
  {
    gzmsg << "Scene is not initialized" << std::endl;
    return;
  }

  // Get the pointer to the visual using the Enitity ID and its material if not already set
  if (!this->visual)
  {
    this->visual = this->scene->VisualById(this->visualEntity);
    if (!this->visual)
    {
      gzerr << "Visual with the id: " << this->visualEntity <<
        " was not found. Maybe entity for plugin is not a Visual?" << std::endl;
      return;
    }
    this->material = this->visual->Material();
  }

  // Set the cycle start time. This is used to calculate the elapsed time in every PreRender event
  if (this->cycleStartTime == std::chrono::duration<double>::zero() ||
      this->cycleStartTime > currentSimTime)
  {
    this->cycleStartTime = currentSimTime;
  }
  std::chrono::duration<double> elapsed = currentSimTime - this->cycleStartTime;

  // Restart cycle
  if (elapsed >= this->period)
  {
    this->cycleStartTime = currentSimTime;
  }

  // Set the current color based on the time elapsed in the current cycle
  if (elapsed <= this->aOnTime)
  {
    this->currentColor = this->colorA;
  }
  else
  {
    this->currentColor = this->colorB;
  }

  // Set the material of the visual with the current color
  this->material->SetDiffuse(this->currentColor);
  this->material->SetAmbient(this->currentColor);
  this->material->SetEmissive(this->currentColor);
  this->material->SetSpecular(this->currentColor);
}
