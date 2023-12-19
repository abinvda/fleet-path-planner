#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include "header.h"

using namespace std;

// Structures for robot arm, AGV, pallet, and waypoint
struct Pose
{
    double x;
    double y;
    double theta;
};

struct RobotArm
{
    string id;
    Pose pose;
};

struct AGV
{
    string id;
    Pose pose;
    string status;
    vector<string> path;
};

struct Pallet
{
    string id;
    Pose pose;
    string object;
};

struct Waypoint
{
    string id;
    Pose pose;
};

// Data structure to store edge usage times
using EdgeUsageTimes = map<string, map<string, vector<double>>>;

// Structure to hold the entire system state
struct SystemState
{
    vector<RobotArm> robotArms;
    vector<AGV> agvs;
    vector<Pallet> pallets;
    vector<Waypoint> palletPlaces;
    EdgeUsageTimes edgeUsageTimes;
};

// Function to load initial state from YAML file
SystemState loadInitialState(const string &filename)
{
    SystemState systemState;

    try
    {
        YAML::Node config = YAML::LoadFile(filename);

        // Load robot arms
        for (const auto &armNode : config["robot_arms"])
        {
            RobotArm arm;
            arm.id = armNode.begin()->first.as<string>();
            arm.pose.x = armNode[arm.id]["pose"]["x"].as<double>();
            arm.pose.y = armNode[arm.id]["pose"]["y"].as<double>();
            arm.pose.theta = armNode[arm.id]["pose"]["theta"].as<double>();
            systemState.robotArms.push_back(arm);
        }

        // Load AGVs
        for (const auto &agvNode : config["agvs"])
        {
            AGV agv;
            agv.id = agvNode.begin()->first.as<string>();
            agv.pose.x = agvNode[agv.id]["pose"]["x"].as<double>();
            agv.pose.y = agvNode[agv.id]["pose"]["y"].as<double>();
            agv.pose.theta = agvNode[agv.id]["pose"]["theta"].as<double>();
            agv.status = agvNode[agv.id]["status"].as<string>();
            systemState.agvs.push_back(agv);
        }

        // Load pallets
        for (const auto &palletNode : config["pallets"])
        {
            Pallet pallet;
            pallet.id = palletNode.begin()->first.as<string>();
            pallet.pose.x = palletNode[pallet.id]["pose"]["x"].as<double>();
            pallet.pose.y = palletNode[pallet.id]["pose"]["y"].as<double>();
            pallet.pose.theta = palletNode[pallet.id]["pose"]["theta"].as<double>();
            pallet.object = palletNode[pallet.id]["object"].as<string>();
            systemState.pallets.push_back(pallet);
        }

        // Load pallet places (parkings) // Not used for now
        for (const auto &placeNode : config["pallet_place"])
        {
            Waypoint waypoint;
            waypoint.id = placeNode.begin()->first.as<string>();
            systemState.palletPlaces.push_back(waypoint);
        }
    }
    catch (const YAML::Exception &e)
    {
        cerr << "Error reading YAML file: " << e.what() << endl;
    }

    return systemState;
}

// Function to get free AGVs
vector<AGV> getFreeAGVs(const SystemState &systemState)
{
    vector<AGV> freeAGVs;
    for (const auto &agv : systemState.agvs)
    {
        if (agv.status == "idle")
        {
            freeAGVs.push_back(agv);
        }
    }
    return freeAGVs;
}

// Function to get current pose of an AGV
Pose getCurrentPose(const SystemState &systemState, const string &agvId)
{
    for (const auto &agv : systemState.agvs)
    {
        if (agv.id == agvId)
        {
            return agv.pose;
        }
    }
    // Return a warning if AGV is not found
    cerr << "AGV with ID " << agvId << " not found." << endl;
    return {0.0, 0.0, 0.0};
}

// Function to calculate distance between two poses
double calculateDistancePose(const Pose &n1, const Pose &n2)
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

// Function to set status of an AGV
void setStatus(SystemState &systemState, const string &agvId, const string &status)
{
    for (auto &agv : systemState.agvs)
    {
        if (agv.id == agvId)
        {
            agv.status = status;
            return;
        }
    }
    cerr << "AGV with ID " << agvId << " not found." << endl;
}

// Function to set path of an AGV
void setPath(SystemState &systemState, const string &agvId, const vector<string> &path)
{
    for (auto &agv : systemState.agvs)
    {
        if (agv.id == agvId)
        {
            agv.path = path;
            return;
        }
    }
    cerr << "AGV with ID " << agvId << " not found." << endl;
}

// Function to print the path of an AGV
string printAGVPath(const SystemState &systemState, const string &agvId)
{
    for (const auto &agv : systemState.agvs)
    {
        if (agv.id == agvId)
        {
            if (!agv.path.empty())
            {
                cout << "AGV " << agv.id << " is moving on the path: ";
                for (const auto &waypoint : agv.path)
                {
                    cout << waypoint << " -> ";
                }
                cout << "End." << endl;
                cout << endl;
            }
            else
            {
                cout << "AGV does not have any path planned." << endl;
            }
            return "Printed.";
        }
    }
    // Return a warning if AGV is not found
    cerr << "AGV with ID " << agvId << " not found." << endl;
    return "Unknown";
}

// Function to print the status of all AGVs
void printAGVsStatus(const SystemState &systemState)
{
    for (const auto &agv : systemState.agvs)
    {
        cout << "AGV " << agv.id << " is: " << agv.status << endl;
    }
}

// Function to update edge usage times based on a timed path
void updateEdgeUsageTimes(EdgeUsageTimes &edgeUsageTimes, const TimedPath &path)
{
    // Ensure that the path has at least two waypoints (start and end), otherwise no edge will be used
    if (path.waypoints.size() < 2 || path.arrivalTimes.size() < 2)
    {
        cerr << "Error: Path should have at least two waypoints (start and end)." << endl;
        exit(EXIT_FAILURE);
    }

    // Iterate through the waypoints in the path
    for (size_t i = 0; i < path.waypoints.size() - 1; ++i)
    {
        const string &source = path.waypoints[i];
        const string &sink = path.waypoints[i + 1];
        double startTime = path.arrivalTimes[i];

        // Update edgeUsageTimes for the edge (source, sink) 
        edgeUsageTimes[source][sink].push_back(startTime);
        // Right now, we're only saving the startTime assuming that the edge traversal time is constant and known. If that's not the case, we should then save the (start time, end time) pair.
    }
}

// Function to print edge usage times
void printEdgeUsageTimes(const EdgeUsageTimes &edgeUsageTimes)
{
    cout << "Edge Usage Times:\n";

    for (const auto &entry : edgeUsageTimes)
    {
        const auto &edge = entry.first;
        const auto &times = entry.second;

        cout << "Edge: " << edge << "\n";

        cout << "Times: ";
        for (const auto &timeEntry : times)
        {
            const auto &startTime = timeEntry.first;
            const auto &usageTimes = timeEntry.second;

            cout << "(" << startTime << ": ";
            for (const auto &usageTime : usageTimes)
            {
                cout << usageTime << " ";
            }
            cout << ")";
        }
        cout << "\n";
    }
}
