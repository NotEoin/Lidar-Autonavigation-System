# Laser-Guided Pathfinding and Obstacle Detection System for Stormworks

<img src="./media/debug screen.png" style="width:4in"
alt="A screenshot of the debug screen" />

### Project Overview
This project is a **Laser-Guided Pathfinding and Obstacle Detection System**, an advanced LUA script that enables autonomous navigation and obstacle avoidance for vehicles in the physics-based simulation game Stormworks: Build and Rescue. Utilizing the LifeBoatAPI, this script implements real-time scanning, pathfinding, and movement adjustments to guide a vehicle towards a specified target, avoiding detected obstructions along the way.

---

### Features
1. **Laser Sweep Mechanism**  
   - Controls a laser sensor that sweeps horizontally to detect obstacles.
   - Adjusts sweep angle based on vehicle pitch and roll to maintain a level scanning path.
   - Outputs yaw and pitch values of the laser’s orientation to optimize for real-time obstacle detection.

2. **Dynamic Obstacle Detection and Mapping**  
   - Detects obstacles within a configurable range and adds them to a real-time map grid.
   - Converts world coordinates to grid-based nodes, which are stored in a table of obstructions for efficient path computation.

3. **A* Pathfinding Algorithm**  
   - Implements A* algorithm to navigate from a start to a goal node, avoiding obstacles.
   - Uses heuristic functions to estimate cost and optimize path efficiency, integrating octile distance for improved performance.
   - Reconstructs the path and merges nodes for smoother navigation, dynamically recalculating if obstacles are detected.

4. **Graphical Map Display**  
   - Visualizes the vehicle’s current position, target direction, and path on a mini-map.
   - Displays obstacles and laser direction, with color-coded path indicators for active navigation and goal-reaching status.

---

### Technical Details

- **Laser and Sensor Configuration**  
   Configurable parameters for cell size, laser range, sweep speed, and path smoothing allow flexible setup to suit different environments and navigation challenges.

- **Efficient Memory Management**  
   Prunes outdated obstruction data to maintain performance, ensuring only relevant obstacles within a specified range are retained.

- **Advanced Trigonometric Calculations**  
   Uses complex trigonometry and vector rotations for yaw and pitch adjustments based on real-time vehicle movement, providing robust and responsive obstacle detection.

---

### Usage
This script is intended to be inserted into a premade 'microcontroller' block in the game Stormworks. Due to some optimisation issues, as well as limitations imposed by the 4096 character limit for in-game embedded Lua scripts, the system is not currently in a final release state. When I am satisfied with the script performance a complete in-game implementation will be uploaded to the steam worshop.

---

### Future Improvements
- **Optimisations**: Remove redundant calls to isObstructed() function.
- **Optimised Path Smoothing**: Implement Bézier or spline curves for smoother path transitions.
- **Enhanced Obstacle Detection**: Add 3D detection for height-based obstacles.
- **User Interface**: Enhance map and visual feedback with more detailed color-coding and obstacle identifiers.

---