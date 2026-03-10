# World Query System

A Gazebo system plugin that queries occupancy and semantic labels for a set of 3D points specified in a CSV file.

## Overview

The `WorldQuerySystem` plugin performs raycasting queries in a Gazebo simulation world to determine:
- Whether a point is occupied (intersects with an object)
- The semantic label of the object at that point
- The distance to the nearest object
- The name of the entity at that point

## Features

- Load query points from CSV files
- Perform raycasting queries using Gazebo's rendering engine
- Extract semantic labels from scene objects
- Save results to output CSV file
- Support for custom max range configuration

## Installation

The plugin is built as part of the `ohms_sim` package. Build using colcon:

```bash
cd ~/Fusion_Experiment/ohms_sim_ws
colcon build --packages-select ohms_sim
source install/setup.bash
```

## Usage

### 1. Prepare Query Points CSV

Create a CSV file with query points in the following format:

```csv
x,y,z
0.0,0.0,1.0
5.0,5.0,1.0
10.0,10.0,1.0
```

The first line is an optional header. Each subsequent line represents a 3D point with x, y, z coordinates.

### 2. Add Plugin to World SDF

Add the plugin to your world's SDF file:

```xml
<plugin name="world_query_system::WorldQuerySystem" filename="WorldQuerySystem">
  <!-- Path to input CSV file with query points -->
  <csv_file>/path/to/your/query_points.csv</csv_file>
  
  <!-- Path to output CSV file for results (optional, default: world_query_results.csv) -->
  <output_csv_file>/path/to/output/results.csv</output_csv_file>
  
  <!-- Maximum range for ray queries in meters (optional, default: 100.0) -->
  <max_range>50.0</max_range>
</plugin>
```

Example integration in a world file:

```xml
<sdf version="1.9">
  <world name="marsyard2020">
    <!-- ... other world content ... -->
    
    <!-- Add the WorldQuerySystem plugin -->
    <plugin name="world_query_system::WorldQuerySystem" filename="WorldQuerySystem">
      <csv_file>$(find ohms_sim)/config/query_points_example.csv</csv_file>
      <output_csv_file>world_query_results.csv</output_csv_file>
      <max_range>100.0</max_range>
    </plugin>
    
  </world>
</sdf>
```

### 3. Run Simulation

Launch your Gazebo simulation as usual. The plugin will:
1. Load the query points from the CSV file
2. Wait for the rendering scene to be ready
3. Perform raycasting queries for each point
4. Save results to the output CSV file

### 4. View Results

The output CSV file contains:

```csv
x,y,z,occupied,semantic_label,distance,entity_name
0.0,0.0,1.0,1,1,0.5,"ground_plane::link::visual"
19.7,7.6,2.0,1,2,1.2,"Oak tree::link::branch"
```

Fields:
- `x,y,z`: Query point coordinates
- `occupied`: 1 if object detected, 0 otherwise
- `semantic_label`: Integer label ID (-1 if unknown)
- `distance`: Distance to nearest object in meters (-1 if not occupied)
- `entity_name`: Name of the detected entity (empty if not occupied)

## Semantic Labels

The system attempts to extract semantic labels from:
1. Visual component metadata (if configured with label plugins)
2. Model/entity name patterns (e.g., "tree" → label 2, "ground" → label 1)

For proper semantic labeling, ensure your world objects use the `ignition-gazebo-label-system` plugin:

```xml
<visual name="visual">
  <!-- ... geometry and material ... -->
  <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
    <label>2</label>
  </plugin>
</visual>
```

## Configuration Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `csv_file` | string | (required) | Path to input CSV file with query points |
| `output_csv_file` | string | `world_query_results.csv` | Path to output CSV file |
| `max_range` | double | `100.0` | Maximum range for ray queries (meters) |

## Example Use Cases

1. **Occupancy Grid Generation**: Query points on a regular grid to create an occupancy map
2. **Semantic Mapping**: Build semantic maps by querying points and collecting labels
3. **Path Validation**: Check if waypoints along a planned path are collision-free
4. **Sensor Simulation**: Simulate range sensors or LiDAR by querying along rays

## Limitations

- Queries are performed once after simulation starts (after ~100 iterations)
- Ray queries are cast downward (-Z direction) from each point
- Semantic label extraction depends on proper world configuration
- Large numbers of query points may impact simulation startup time

## Troubleshooting

**Plugin not loading:**
- Ensure `colcon build` completed successfully
- Source the workspace: `source install/setup.bash`
- Check that `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` includes your workspace

**No results generated:**
- Check CSV file path is correct and accessible
- Verify rendering engine is "ogre2" in world file
- Look for error messages in Gazebo console output

**Incorrect semantic labels:**
- Verify objects in your world have label plugins configured
- Check entity naming patterns match the label inference logic
- Consider extending `GetSemanticLabel()` method for custom label extraction

## Advanced Usage

### Custom Ray Directions

To modify ray direction (currently downward), edit `WorldQuerySystem.cc`:

```cpp
// In PerformRayQuery method, change direction vector:
gz::math::Vector3d direction(0, 0, -1);  // Current: downward
// Examples:
// gz::math::Vector3d direction(1, 0, 0);  // X-axis
// gz::math::Vector3d direction(0, 1, 0);  // Y-axis
```

### Multiple Ray Queries per Point

To cast multiple rays per point (e.g., in different directions), modify the `PerformRayQuery` method to return a vector of results.

## API Reference

See header file: `include/world_query_system/WorldQuerySystem.hh`

## Contributing

To extend functionality:
1. Modify header file to add new data structures or methods
2. Implement functionality in `src/WorldQuerySystem.cc`
3. Update CMakeLists.txt if adding new dependencies
4. Rebuild and test

## License

Same as parent `ohms_sim` package.
