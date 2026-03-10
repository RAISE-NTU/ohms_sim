#include "world_query_system/WorldQuerySystem.hh"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Util.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/math/Vector3.hh>

using namespace world_query_system;

// Register this plugin
IGNITION_ADD_PLUGIN(
    world_query_system::WorldQuerySystem,
    gz::sim::System,
    WorldQuerySystem::ISystemConfigure,
    WorldQuerySystem::ISystemPostUpdate)

/// \brief Private data class for WorldQuerySystem
class world_query_system::WorldQuerySystem::Implementation
{
  /// \brief Input CSV filename
  public: std::string csvFile;

  /// \brief Output CSV filename
  public: std::string outputCsvFile{"world_query_results.csv"};

  /// \brief Ray length for queries (in meters)
  public: double rayLength{0.1};

  /// \brief Query points loaded from CSV
  public: std::vector<QueryPoint> queryPoints;

  /// \brief Flag indicating if queries have been performed
  public: bool queriesPerformed{false};

  /// \brief Counter for iterations (to wait for scene initialization)
  public: int iterationCount{0};

  /// \brief Minimum iterations before performing queries
  public: int minIterations{200};
  
  /// \brief Flag to track if we've tried to initialize rendering
  public: bool initAttempted{false};

  /// \brief Pointer to rendering scene
  public: gz::rendering::ScenePtr scene;

  /// \brief Ray query object
  public: gz::rendering::RayQueryPtr rayQuery;
};

//////////////////////////////////////////////////
WorldQuerySystem::WorldQuerySystem()
  : dataPtr(std::make_unique<Implementation>())
{
}

//////////////////////////////////////////////////
WorldQuerySystem::~WorldQuerySystem()
{
}

//////////////////////////////////////////////////
void WorldQuerySystem::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &/*_eventMgr*/)
{
  ignmsg << "WorldQuerySystem: Configuring plugin..." << std::endl;

  // Get CSV file path
  if (_sdf->HasElement("csv_file"))
  {
    this->dataPtr->csvFile = _sdf->Get<std::string>("csv_file");
    ignmsg << "WorldQuerySystem: Input CSV file: " << this->dataPtr->csvFile << std::endl;
  }
  else
  {
    ignerr << "WorldQuerySystem: <csv_file> not specified!" << std::endl;
    return;
  }

  // Get output CSV file path
  if (_sdf->HasElement("output_csv_file"))
  {
    this->dataPtr->outputCsvFile = _sdf->Get<std::string>("output_csv_file");
  }
  ignmsg << "WorldQuerySystem: Output CSV file: " << this->dataPtr->outputCsvFile << std::endl;

  // Get ray length
  if (_sdf->HasElement("ray_length"))
  {
    this->dataPtr->rayLength = _sdf->Get<double>("ray_length");
  }
  ignmsg << "WorldQuerySystem: Ray length: " << this->dataPtr->rayLength << " meters" << std::endl;

  // Load query points
  this->dataPtr->queryPoints = LoadQueryPoints(this->dataPtr->csvFile);
  
  if (this->dataPtr->queryPoints.empty())
  {
    ignerr << "WorldQuerySystem: No query points loaded!" << std::endl;
    return;
  }

  ignmsg << "WorldQuerySystem: Loaded " << this->dataPtr->queryPoints.size() 
        << " query points" << std::endl;
}

//////////////////////////////////////////////////
void WorldQuerySystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                                   const gz::sim::EntityComponentManager &_ecm)
{
  // Perform queries only once, after scene is ready
  if (this->dataPtr->queriesPerformed)
    return;

  this->dataPtr->iterationCount++;

  // Wait for rendering scene to be ready
  if (this->dataPtr->iterationCount < this->dataPtr->minIterations)
    return;

  // Get rendering scene
  if (!this->dataPtr->scene)
  {
    if (!this->dataPtr->initAttempted)
    {
      ignmsg << "WorldQuerySystem: Attempting to initialize rendering scene..." << std::endl;
      this->dataPtr->initAttempted = true;
    }
    
    this->dataPtr->scene = gz::rendering::sceneFromFirstRenderEngine();
    
    if (!this->dataPtr->scene)
    {
      // Try again later, but don't spam the logs
      if (this->dataPtr->iterationCount % 100 == 0)
      {
        ignwarn << "WorldQuerySystem: Rendering scene not ready yet (iteration " 
                << this->dataPtr->iterationCount << ")" << std::endl;
      }
      return;
    }

    ignmsg << "WorldQuerySystem: Rendering scene initialized successfully!" << std::endl;

    // Create ray query object
    this->dataPtr->rayQuery = this->dataPtr->scene->CreateRayQuery();
    if (!this->dataPtr->rayQuery)
    {
      ignerr << "WorldQuerySystem: Failed to create ray query!" << std::endl;
      this->dataPtr->queriesPerformed = true;
      return;
    }
    
    ignmsg << "WorldQuerySystem: Ray query object created successfully!" << std::endl;
  }

  ignmsg << "WorldQuerySystem: Performing queries on " 
         << this->dataPtr->queryPoints.size() << " points..." << std::endl;

  // Perform queries
  std::vector<QueryResult> results;
  results.reserve(this->dataPtr->queryPoints.size());

  for (const auto &point : this->dataPtr->queryPoints)
  {
    QueryResult result = PerformRayQuery(point, _ecm);
    results.push_back(result);
  }

  // Save results
  SaveResults(this->dataPtr->outputCsvFile, results);

  ignmsg << "WorldQuerySystem: Query results saved to " 
        << this->dataPtr->outputCsvFile << std::endl;

  this->dataPtr->queriesPerformed = true;
}

//////////////////////////////////////////////////
std::vector<WorldQuerySystem::QueryPoint> 
WorldQuerySystem::LoadQueryPoints(const std::string &_filename)
{
  std::vector<QueryPoint> points;
  std::ifstream file(_filename);

  if (!file.is_open())
  {
    ignerr << "WorldQuerySystem: Failed to open CSV file: " << _filename << std::endl;
    return points;
  }

  std::string line;
  bool firstLine = true;

  while (std::getline(file, line))
  {
    // Skip header if it contains non-numeric data
    if (firstLine)
    {
      firstLine = false;
      // Check if first line is header (contains letters)
      if (line.find_first_not_of("0123456789.,-+eE \t") != std::string::npos)
      {
        continue;
      }
    }

    // Parse CSV line
    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;

    while (std::getline(ss, token, ','))
    {
      try
      {
        values.push_back(std::stod(token));
      }
      catch (const std::exception &e)
      {
        ignwarn << "WorldQuerySystem: Failed to parse value: " << token << std::endl;
        break;
      }
    }

    // Need at least x, y, z
    if (values.size() >= 3)
    {
      QueryPoint point;
      point.x = values[0];
      point.y = values[1];
      point.z = values[2];
      points.push_back(point);
    }
  }

  file.close();
  return points;
}

//////////////////////////////////////////////////
WorldQuerySystem::QueryResult 
WorldQuerySystem::PerformRayQuery(const QueryPoint &_point,
                                   const gz::sim::EntityComponentManager &_ecm)
{
  QueryResult result;
  result.x = _point.x;
  result.y = _point.y;
  result.z = _point.z;
  result.is_occupied = false;
  result.semantic_class = 0;

  if (!this->dataPtr->rayQuery)
  {
    ignwarn << "WorldQuerySystem: Ray query not initialized for point ("
            << _point.x << ", " << _point.y << ", " << _point.z << ")" << std::endl;
    return result;
  }

  // Create a small ray downward from the query point
  gz::math::Vector3d origin(_point.x, _point.y, _point.z);
  gz::math::Vector3d direction(0, 0, -1);  // Downward
  
  // Set ray query parameters
  this->dataPtr->rayQuery->SetOrigin(origin);
  this->dataPtr->rayQuery->SetDirection(direction);

  // Perform the ray query
  auto rayResult = this->dataPtr->rayQuery->ClosestPoint();

  if (rayResult)
  {
    result.is_occupied = true;

    // Try to get the distance
    double distance = rayResult.distance;
    
    // Check if the hit is within our ray length
    if (distance <= this->dataPtr->rayLength)
    {
      result.is_occupied = true;
      
      // Try to get semantic information - this would require mapping
      // the objectId back to an entity, which is complex
      // For now, use a simple approach based on object ID
      result.semantic_class = 0;  // Unknown by default
    }
    else
    {
      result.is_occupied = false;
    }
  }

  return result;
}

//////////////////////////////////////////////////
void WorldQuerySystem::SaveResults(const std::string &_filename,
                                    const std::vector<QueryResult> &_results)
{
  std::ofstream file(_filename);

  if (!file.is_open())
  {
    ignerr << "WorldQuerySystem: Failed to open output file: " << _filename << std::endl;
    return;
  }

  // Write header
  file << "x,y,z,is_occupied,semantic_class" << std::endl;

  // Write results
  file << std::fixed;
  file.precision(3);

  for (const auto &result : _results)
  {
    file << result.x << ","
         << result.y << ","
         << result.z << ","
         << (result.is_occupied ? 1 : 0) << ","
         << result.semantic_class << std::endl;
  }

  file.close();
}

//////////////////////////////////////////////////
int WorldQuerySystem::GetSemanticLabel(gz::sim::Entity _entity,
                                        const gz::sim::EntityComponentManager &_ecm)
{
  // This is a placeholder implementation
  // In a full implementation, you would:
  // 1. Check for label components attached to the entity
  // 2. Check parent entities for labels
  // 3. Use entity name or type to infer labels
  
  return 0;  // Unknown/default
}
