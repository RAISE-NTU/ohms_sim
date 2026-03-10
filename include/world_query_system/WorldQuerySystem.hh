#ifndef WORLD_QUERY_SYSTEM__WORLDQUERYSYSTEM_HH_
#define WORLD_QUERY_SYSTEM__WORLDQUERYSYSTEM_HH_

#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <gz/sim/System.hh>

namespace world_query_system
{
  /// \brief A system plugin that performs raycasting queries at specified 3D points
  /// to determine occupancy and semantic labels using the physics engine.
  ///
  /// This plugin reads query points from a CSV file, performs raycasting queries
  /// using Gazebo's rendering engine, and outputs results including occupancy status
  /// and semantic labels to an output CSV file.
  ///
  /// ## System Parameters
  ///
  /// - <csv_file> Path to input CSV file containing query points (x,y,z format)
  /// - <output_csv_file> Path to output CSV file (default: world_query_results.csv)
  /// - <ray_length> Length of rays to cast in meters (default: 0.1)
  ///
  /// ## Example Usage in SDF:
  ///
  /// \code{.xml}
  /// <plugin name="world_query_system::WorldQuerySystem" filename="WorldQuerySystem">
  ///   <csv_file>/path/to/query_points.csv</csv_file>
  ///   <output_csv_file>world_query_results.csv</output_csv_file>
  ///   <ray_length>0.1</ray_length>
  /// </plugin>
  /// \endcode
  class WorldQuerySystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: WorldQuerySystem();

    /// \brief Destructor
    public: ~WorldQuerySystem() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                           const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Structure to hold a query point
    private: struct QueryPoint
    {
      double x;
      double y;
      double z;
    };

    /// \brief Structure to hold a query result
    private: struct QueryResult
    {
      double x;
      double y;
      double z;
      bool is_occupied;
      int semantic_class;
    };

    /// \brief Load query points from CSV file
    /// \param[in] _filename Path to CSV file
    /// \return Vector of query points
    private: std::vector<QueryPoint> LoadQueryPoints(const std::string &_filename);

    /// \brief Perform ray query at a specific point
    /// \param[in] _point Query point
    /// \param[in] _ecm Entity component manager
    /// \return Query result with occupancy and semantic info
    private: QueryResult PerformRayQuery(const QueryPoint &_point,
                                         const gz::sim::EntityComponentManager &_ecm);

    /// \brief Save results to CSV file
    /// \param[in] _filename Output CSV filename
    /// \param[in] _results Vector of query results
    private: void SaveResults(const std::string &_filename,
                             const std::vector<QueryResult> &_results);

    /// \brief Extract semantic label from an entity
    /// \param[in] _entity Entity to check
    /// \param[in] _ecm Entity component manager
    /// \return Semantic label ID, or 0 if not found
    private: int GetSemanticLabel(gz::sim::Entity _entity,
                                   const gz::sim::EntityComponentManager &_ecm);

    /// \brief Private data pointer
    private: class Implementation;
    private: std::unique_ptr<Implementation> dataPtr;
  };
}

#endif  // WORLD_QUERY_SYSTEM__WORLDQUERYSYSTEM_HH_
