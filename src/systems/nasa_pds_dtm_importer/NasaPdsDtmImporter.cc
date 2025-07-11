/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <string>

#include <gz/plugin/Register.hh>

#include <gz/common/geospatial/Dem.hh>

#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Heightmap.hh>
#include <gz/rendering/HeightmapDescriptor.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/rendering/Events.hh"

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/Util.hh"

#include "NasaPdsDtmImporter.hh"

using namespace gz;
using namespace sim;
using namespace systems;

// Private data class
class gz::sim::systems::NasaPdsDtmImporterPrivate
{
  /// \brief Pointer to the event manager
  public: EventManager *eventMgr = nullptr;

  /// \brief Connection to pre-render event callback.
  public: gz::common::ConnectionPtr preRenderConnection{nullptr};

  /// \brief Entity
  public: Entity entity;

  /// \brief Name of the Terrain Model
  public: std::string terrainName;

  /// \brief URL of the DTM in NASA PDS (Planetary Data Systems)
  public: std::string dtmPdsUrl;

  /// \brief X size limit to be used for the imported raster
  public: double rasterXSizeLimit{100.0};

  /// \brief X size limit to be used for the imported raster
  public: double rasterYSizeLimit{50.0};

  /// \brief Color for the terrain if no texture is available for the DTM
  public: bool useColor{false};

  /// \brief Color for the terrain if no texture is available for the DTM
  public: math::Vector3d color = math::Vector3d{1.4, 1.2, 1.0};

  public: double equatorialAxisRadius{0.0};


  public: double polarAxisRadius{0.0};

  /// \brief The spherical coordinates object for the terrain
  public: math::SphericalCoordinates terrainSphericalCoordinates =
    math::SphericalCoordinates();

  /// \brief Dem Class instance from gz common for loading DEMs
  public: std::shared_ptr<gz::common::Dem> demImporter = std::make_shared<gz::common::Dem>();

  public: gz::rendering::HeightmapDescriptor heightmapDescriptor;
  public: gz::rendering::HeightmapTexture heightmapTexture;
  public: gz::rendering::VisualPtr heightmapVisual;
  public: gz::rendering::GeometryPtr heightmapGeometry{nullptr};
  public: bool demLoadedCorrectly{false};
  public: bool addedHeightmapVisualToScene{false};

  public: void AddHeightmapToScene();
};

//////////////////////////////////////////////////
NasaPdsDtmImporter::NasaPdsDtmImporter()
  : System(), dataPtr(std::make_unique<NasaPdsDtmImporterPrivate>())
{
}


//////////////////////////////////////////////////
void NasaPdsDtmImporter::Configure(
  const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  this->dataPtr->entity = _entity;
  this->dataPtr->eventMgr = &_eventMgr;

  if (_sdf->HasElement("terrain_name"))
  {
    this->dataPtr->terrainName = _sdf->Get<std::string>("terrain_name");
  }

  if (_sdf->HasElement("use_color"))
  {
    this->dataPtr->useColor = _sdf->Get<bool>("use_color");
  }

  if (_sdf->HasElement("terrain_color"))
  {
    this->dataPtr->color = _sdf->Get<math::Vector3d>("color");
  }

  if (_sdf->HasElement("dtm_pds_url"))
  {
    this->dataPtr->dtmPdsUrl = _sdf->Get<std::string>("dtm_pds_url");
  }

  if (_sdf->HasElement("raster_x_size_limit"))
  {
    this->dataPtr->rasterXSizeLimit = _sdf->Get<double>("raster_x_size_limit");
    this->dataPtr->demImporter->SetRasterXSizeLimit(this->dataPtr->rasterXSizeLimit);
  }

  if (_sdf->HasElement("raster_y_size_limit"))
  {
    this->dataPtr->rasterYSizeLimit = _sdf->Get<double>("raster_y_size_limit");
    this->dataPtr->demImporter->SetRasterXSizeLimit(this->dataPtr->rasterYSizeLimit);
  }

  if (_sdf->HasElement("equatorial_axis_radius"))
  {
    this->dataPtr->equatorialAxisRadius = _sdf->Get<double>("equatorial_axis_radius");
  }

  if (_sdf->HasElement("polar_axis_radius"))
  {
    this->dataPtr->polarAxisRadius = _sdf->Get<double>("polar_axis_radius");
  }

  // Set Spherical Coordinates for the DEM
  this->dataPtr->terrainSphericalCoordinates = math::SphericalCoordinates(
    math::SphericalCoordinates::CUSTOM_SURFACE, this->dataPtr->equatorialAxisRadius,
    this->dataPtr->polarAxisRadius);
  this->dataPtr->demImporter->SetSphericalCoordinates(this->dataPtr->terrainSphericalCoordinates);

  // Load the dem using the URL
  int res = this->dataPtr->demImporter->Load("/vsicurl/" + this->dataPtr->dtmPdsUrl);

  if (res != 0)
  {
    gzerr << "[NASA PDS DTM IMPORTER] Failed in loading the DTM named: "
      << this->dataPtr->terrainName << "from URL: " << this->dataPtr->dtmPdsUrl << std::endl;

    this->dataPtr->demLoadedCorrectly = false;
    return;
  }
  else
  {
    gzmsg << "[NASA PDS DTM IMPORTER] Successfully loaded DTM named: "
      << this->dataPtr->terrainName << " from URL: " << this->dataPtr->dtmPdsUrl << std::endl;

    gzmsg << "[NASA PDS DTM IMPORTER] The width and height of the imported DTM is "
      << this->dataPtr->demImporter->WorldWidth() << ", "
      << this->dataPtr->demImporter->WorldHeight() << std::endl;

    gzmsg << "[NASA PDS DTM IMPORTER] The min and max elevation of the DTM is "
      << this->dataPtr->demImporter->MinElevation() << ", "
      << this->dataPtr->demImporter->MaxElevation() << std::endl;
  }

  this->dataPtr->heightmapDescriptor.SetName(this->dataPtr->terrainName);
  this->dataPtr->heightmapDescriptor.SetData(this->dataPtr->demImporter);
  this->dataPtr->heightmapDescriptor.SetSize({20, 20, 6.85});
  this->dataPtr->heightmapDescriptor.SetSampling(2u);
  this->dataPtr->heightmapDescriptor.SetUseTerrainPaging(false);
  this->heightmapTexture = gz::rendering::HeightmapTexture();
  // this->dataPtr->heightmapTexture.SetSize(20.0);
  // this->dataPtr->heightmapTexture.SetDiffuse("../media/moon_diffuse.png");
  // this->dataPtr->heightmapTexture.SetNormal("../media/moon_normal.png");

  this->dataPtr->heightmapDescriptor.AddTexture(this->dataPtr->heightmapTexture);
  this->dataPtr->heightmapDescriptor.SetPosition({0, 0,
    std::abs(this->dataPtr->demImporter->MinElevation())});

  this->dataPtr->demLoadedCorrectly = true;

  this->dataPtr->preRenderConnection =
      _eventMgr.Connect<gz::sim::events::PreRender>(std::bind(
          &NasaPdsDtmImporterPrivate::AddHeightmapToScene,
          this->dataPtr.get()));
  std::cout << "Created rendering events connection: " << std::endl;
}

//////////////////////////////////////////////////
void NasaPdsDtmImporterPrivate::AddHeightmapToScene()
{
  std::cout << "Post Render Event " << std::endl;
  if (!this->addedHeightmapVisualToScene)
  {
    std::cout << "Adding heightmap to scene " << std::endl;
    gz::rendering::ScenePtr scene(nullptr);
    scene = gz::rendering::sceneFromFirstRenderEngine();

    if (scene == nullptr || !scene->IsInitialized())
    {
      gzerr << "Scene is not ready at the moment" << std::endl;
      return;
    }

    gz::rendering::VisualPtr rootVis(nullptr);
    rootVis = scene->RootVisual();
    if (rootVis == nullptr)
    {
      gzerr << "No root visual found. Returning" << std::endl;
      return;
    }

    if (this->demLoadedCorrectly)
    {
      this->heightmapVisual = scene->CreateVisual(
        this->terrainName + "_visual");
      this->heightmapGeometry = scene->CreateHeightmap(this->heightmapDescriptor);
      this->heightmapVisual->AddGeometry(this->heightmapGeometry);
      rootVis->AddChild(this->heightmapVisual);
    }

    this->addedHeightmapVisualToScene = true;
    std::cout << "Added heightmap to scene " << std::endl;
  }

  this->preRenderConnection.reset();
}

GZ_ADD_PLUGIN(NasaPdsDtmImporter,
        System,
        NasaPdsDtmImporter::ISystemConfigure)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(NasaPdsDtmImporter, "gz::sim::systems::NasaPdsDtmImporter")
