# Overview

  

The Fleet Planner is a C++-based application designed to find optimal paths for a fleet of AGVs operating in dynamic environments with time-varying costs. The process considers the current system state, generates a graph representation of the surroundings, and calculates optimal paths in response to box requests from robotic arms.

  

## Building and Execution

  

1. Compile and build the project using a C++ compiler (e.g., g++) with the appropriate flags.
    - Edit `compile.sh` file to correctly point at your yaml-cpp installation
    - Open directory in terminal
    - Run `./compile.sh fleetPlanner.cpp`

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

4. ## **Path Planning**
The path planning process can be broken into following steps:

a. **Identification Process:**
    - The function commences by identifying the locations of the robot arm and the requested box/pallet within the map.

b. **AGV Availability Check:**
	- Following identification, the system checks for the availability of AGVs to fulfill the request.
	- Among the available AGVs, the algorithm selects the one closest to the box/pallet position, considering the time required to reach the designated pallet.

c. **Simple Path (No Congestion):**
	- For the simple path, the algorithm utilizes the `getPath` function.
	- This function generates the optimal path as a sequence of waypoints.
	- The resulting path can be visualized using the `printPath` function.

d. **Congestion-Aware Path:**
	- In the congestion-aware scenario, the path is defined not only by waypoints but also by arrival times at those waypoints.
	- Optionally, the system can display any waiting periods required before departing from a waypoint.

e. **System State Update:**
	- After planning the AGV's path in congestion-aware scenarios, the system state undergoes an update.
	- This update integrates information about the planned path for the AGV, enabling effective planning for subsequent requests.

## Number of AGVs required

The optimal number of AGVs per robot is contingent on factors like box request frequency, waypoint distances, and the optimization goals of the path planning algorithm. Determining the minimum number of AGVs per robot requires an in-depth analysis of system requirements, operational constraints, and the specific characteristics of the warehouse or operational environment.

In an extreme scenario, utilizing two AGVs per robot arm can effectively minimize pallet exchange time, as long as the pallets are not requested at a faster rate than the time it takes to bring them to the robot arm. This setup allows one AGV to remove the pallet from the robot arm, while the second AGV simultaneously brings a new pallet to the robot arm. The efficacy of this approach depends on the balance between pallet request frequency and the time it takes to transport pallets.

Here are some considerations and potential ways to optimize the number of AGVs per robot:
1. **Predictive Task Allocation:**
   - Leverage predictive analytics to forecast future box requests and allocate AGVs proactively. This approach reduces waiting times for AGVs, enhancing overall system efficiency.

2. **Load Balancing:**
   - Implement load balancing strategies to evenly distribute the workload among available AGVs. Idle AGVs can strategically position themselves to minimize travel distances, promptly responding to new pallet requests.

3. **Anticipating Pallet Removal:**
    - Ensure AGVs are strategically positioned near robot arms when a new pallet is expected, and the current pallet needs removal.

4. **Optimal Waypoint Placement:**
   - Analyze waypoint placement for strategic positioning, minimizing travel times and enhancing overall system performance.

These considerations collectively contribute to an efficient AGV fleet, minimizing pallet exchange times while optimizing resource utilization.

## ROS Components
While the current implementation is in plain C++, we can build a a ROS implementation with the following components: 
  

1.  **fleet_planner_node:** Corresponding to `fleetPlanner.cpp`
	- Main executable file. Integrates functions from other ROS nodes.

2. **system_state_node:** Corresponding to `systemState.cpp`
	- Could be implemented as a ROS node publishing system state information.

3. **graph_generator_node:** Corresponding to `getGraph.cpp`
	- Could be implemented as a ROS node generating a graph and publishing it.

4. **path_calculator_node:** Corresponding to `getPath.cpp`
	- Could be a ROS service that takes the current system state and returns optimal paths.  

5. **Additional Considerations**
	- For a ROS implementation, additional components like nodes, services, and messages would be necessary.
	- We may also need to define custom messages to communicate system state, box requests, and path information.
	- Implement a Header File containing ROS message and service definitions for proper communication between nodes.