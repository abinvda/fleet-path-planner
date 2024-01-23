# Fleet Planner Overview
The Fleet Planner is a C++-based application designed to find optimal paths for a fleet of AGVs operating in dynamic environments with time-varying costs. The process considers the current system state, generates a graph representation of the surroundings, and calculates optimal paths in response to box requests from robotic arms.

## Building and Execution

1. Compile and build the project using a C++ compiler (e.g., g++) with the appropriate flags.
    - Make sure the `compile.sh` file correctly points to the yaml-cpp installation
    - Run `./compile.sh fleetPlanner.cpp` to generate the executable

2. Execute the generated executable (`fleetPlanner`) to initiate the Fleet Planner.
    - Run `./fleetPlanner`

3. Define and modify the locations of different entities (robots, pallets, etc.) in the `initial_state.yaml` file.


## Main Components
  
1.  **`fleetPlanner.cpp`**
	- Manages the overall planning process, leveraging support files such as `getPath` and `systemState`.
	- The `requestBox(robot, box)` function locates the robot and box/pallet positions, identifies the most suitable AGV for delivery, and plans an optimal path for that AGV.

2.  **`systemState.cpp`**
	- Manages the current system state, encompassing the locations of robot arms, AGV statuses and locations, pallet locations, and AGV travel information on the graph.
	- Provides essential functions to update and query the system state. 

3.  **`getGraph.cpp`**
	- Constructs the graph representation of the environment using information from `waypoints.yaml`.
	- Parses waypoint locations, converting them into a directed weighted graph where weights represent travel durations.
	- Identifies the waypoints where specific entities (robot arms, AGVs, etc.) are located.

4.  **`getPath.cpp`**
	- Implements the path planning algorithms.
	- Conventional A-star algorithm for AGVs without considering plans of other AGVs.
	- Time-dependent path planning algorithm that plans for an AGV path while avoiding congestion on the map.


## Execution Flow

1.  **Initialization:**
	-  `systemState.cpp` initializes the system state based on the `initial_state.yaml` file.
```cpp
systemState = loadInitialState("config/initial_state.yaml");
```

2.  **Graph Generation:**
	-  `getGraph.cpp` constructs a graph based on the `waypoints.yaml` file.
```cpp
graphNodes = parseNodes("config/waypoints.yaml");
graph = buildGraph(graphNodes, agvSpeed);
```

3.  **Path Request:**
	- Given a robot arm and a box name, call the `requestBox` function to obtain the required path.
```cpp
string box = "coke";
string robotArm = "robot_arm_0";
Path path1 = fleetPlanner.requestBox<Path>(robotArm, box);
```

## Path Planning
The path planning process can be broken into following steps:

1. **Identification Process:**
    - The function commences by identifying the locations of the robot arm and the requested box/pallet within the map.

2. **AGV Availability Check:**
	- Following identification, the system checks for the availability of AGVs to fulfill the request.
	- Among the available AGVs, the algorithm selects the one closest to the box/pallet position, considering the time required to reach the designated pallet.

3. **Simple Path (No Congestion):**
	- For the simple path, the algorithm utilizes the `getPath` function.
	- This function generates the optimal path as a sequence of waypoints.
	- The resulting path can be visualized using the `printPath` function.

4. **Congestion-Aware Path:**
	- In the congestion-aware scenario, the path is defined not only by waypoints but also by arrival times at those waypoints.
	- Optionally, the system can display any waiting periods required before departing from a waypoint.

5. **System State Update:**
	- After planning the AGV's path in congestion-aware scenarios, the system state undergoes an update.
	- This update integrates information about the planned path for the AGV, enabling effective planning for subsequent requests.
