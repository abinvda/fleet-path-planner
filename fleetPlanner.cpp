#include <iostream>
#include <map>
#include <vector>
#include "systemState.cpp"
#include "getPath.cpp"

using namespace std;

// Constants
const double agvSpeed = 1.0;                              // AGV speed
const double congestionCost = numeric_limits<int>::max(); // Currently setting a large cost on sharing edges
const double maxWait = 20.0;                               // Max amount of time we want to wait at a waypoint

class FleetPlanner
{
public:
    FleetPlanner()
    {
        initSystem();
    }

    // Initialize the system
    void initSystem()
    {
        // Parse waypoint information and build the graph
        graphNodes = parseNodes("config/waypoints.yaml");
        graph = buildGraph(graphNodes, agvSpeed);

        // Load the initial system state
        systemState = loadInitialState("config/initial_state.yaml");
    }

    // Get the system state
    const SystemState &getSystemState() const
    {
        return systemState;
    }

    // Get a reference to the system state
    SystemState &getMutableSystemState()
    {
        return systemState;
    }

    // Request a box // Can be either a simple topological path or a time-based execution path
    template <typename PathType>
    PathType requestBox(const string &robotArm, const string &box)
    {
        // Get the location of the requested box
        string startPosID;
        PathType finalPath;
        bool boxFound = false;
        for (const auto &pallet : systemState.pallets)
        {
            if (pallet.object == box)
            {
                startPosID = identifyWaypoint(graphNodes, pallet.pose.x, pallet.pose.y);
                boxFound = true;
                break;
            }
        }
        if (!boxFound)
        {
            cerr << "Error: The requested object '" << box << "' was not found on any pallet." << endl;
            exit(EXIT_FAILURE);
        }

        // Get the location of the robot arm
        string goalPosID;
        bool armFound = false;
        for (const auto &arm : systemState.robotArms)
        {
            if (arm.id == robotArm)
            {
                goalPosID = identifyWaypoint(graphNodes, arm.pose.x, arm.pose.y);
                armFound = true;
                break;
            }
        }
        if (!armFound)
        {
            cerr << "Error: The requesting robot arm '" << robotArm << "' is not present on the map." << endl;
            exit(EXIT_FAILURE);
        }

        // Get a list of idle AGVs
        vector<AGV> idleAGVs = getFreeAGVs(systemState);

        if (!idleAGVs.empty())
        {
            cout << "Idle AGVs: ";
            for (const auto &agv : idleAGVs)
            {
                cout << agv.id << " ";
            }

            // Select the closest AGV
            AGV selectedAGV;
            double minTime = numeric_limits<double>::max();
            PathType pathToPallet;

            for (const auto &agv : idleAGVs)
            {
                string agvPoseID = identifyWaypoint(graphNodes, agv.pose.x, agv.pose.y);
                PathType tempPath;
                if constexpr (std::is_same<PathType, Path>::value)
                {
                    tempPath = getPath(graphNodes, graph, agvPoseID, startPosID); // Traversal path distance
                }
                else if constexpr (std::is_same<PathType, TimedPath>::value)
                {
                    tempPath = getPath(graphNodes, graph, agvPoseID, startPosID, systemState.edgeUsageTimes, 0.0);
                }

                if (tempPath.totalTime < minTime)
                {
                    pathToPallet = tempPath;
                    minTime = tempPath.totalTime;
                    selectedAGV = agv;
                }
            }

            cout << " | Selected AGV: " << selectedAGV.id << endl;

            // Get the current pose of the selected AGV
            Pose currentAGVPose = selectedAGV.pose;

            // Plan path from pallet position to position of the requesting robot arm
            PathType pathToRobotArm;
            if constexpr (std::is_same<PathType, Path>::value)
            {
                pathToRobotArm = getPath(graphNodes, graph, startPosID, goalPosID);
            }
            else if constexpr (std::is_same<PathType, TimedPath>::value)
            {
                double startTime = pathToPallet.totalTime;
                pathToRobotArm = getPath(graphNodes, graph, startPosID, goalPosID, systemState.edgeUsageTimes, startTime);
            }

            // Check if a valid path to the robot arm is obtained
            if (pathToRobotArm.waypoints.empty())
            {
                cerr << "No path available to the robot." << endl;
                exit(EXIT_FAILURE);
            }

            finalPath.selectedAGVID = selectedAGV.id;
            
            string pickUpAction = "pick_up_box";
            string placeAction = "place_box";

            // Obtain final path, set AGV status, and update edge usage times
            finalPath = combinePaths(pathToPallet, pickUpAction, pathToRobotArm);

            if constexpr (std::is_same<PathType, Path>::value)
            {
                finalPath = combinePaths(finalPath, placeAction, Path{});
            }
            else if constexpr (std::is_same<PathType, TimedPath>::value)
            {
                finalPath = combinePaths(finalPath, placeAction, TimedPath{});
                updateEdgeUsageTimes(systemState.edgeUsageTimes, pathToRobotArm);
            }

            setStatus(systemState, selectedAGV.id, "busy");
            setPath(systemState, selectedAGV.id, finalPath.waypoints);
            return finalPath;
        }
        else
        {
            cerr << "No idle AGVs available." << endl;
            exit(EXIT_FAILURE);
        }
    }

private:
    map<string, Node> graphNodes;
    map<string, map<string, double>> graph;
    SystemState systemState;
};

int main()
{
    // Plan without considering congestion
    FleetPlanner fleetPlanner;
    cout << "Planning without considering congestion: " << endl;

    string box = "coke";
    string robotArm = "robot_arm_0";
    Path path1 = fleetPlanner.requestBox<Path>(robotArm, box);
    printPath(path1, robotArm, box);
    cout << endl;

    box = "water";
    robotArm = "robot_arm_1";
    Path path2 = fleetPlanner.requestBox<Path>(robotArm, box);
    printPath(path2, robotArm, box);

    cout << endl;

    // Congestion-aware planning
    fleetPlanner.initSystem(); // Reset
    cout << "Congestion-aware planning: " << endl;
    box = "coke";
    robotArm = "robot_arm_0";
    TimedPath timedPath1 = fleetPlanner.requestBox<TimedPath>(robotArm, box);
    printPath(timedPath1, robotArm, box);
    cout << endl;

    box = "water";
    robotArm = "robot_arm_1";
    TimedPath timedPath2 = fleetPlanner.requestBox<TimedPath>(robotArm, box);
    printPath(timedPath2, robotArm, box);

    cout << endl;

    box = "green_tea";
    robotArm = "robot_arm_2";
    TimedPath timedPath3 = fleetPlanner.requestBox<TimedPath>(robotArm, box);
    printPath(timedPath3, robotArm, box);

    return 0;
}
