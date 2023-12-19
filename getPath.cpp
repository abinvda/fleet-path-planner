#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <cmath>
#include "getGraph.cpp"

using namespace std;

// External constants
extern const double agvSpeed;
extern const double maxWait;
extern const double congestionCost;

// Small constant to handle floating-point precision
const double epsilon = 1e-6;

// Data structure to store edge usage times
using EdgeUsageTimes = map<string, map<string, vector<double>>>;

// Structure to represent a simple path
struct Path
{
    vector<string> waypoints;
    double totalDist;
    double totalTime;
    string selectedAGVID;
};

// Structure to represent search nodes for timed path planning
struct CongestionNode
{
    string waypoint;
    double arrivalTime;
    double waitTime;

    // Defining the comparison operators
    bool operator<(const CongestionNode &other) const
    {
        return arrivalTime < other.arrivalTime;
    }

    bool operator>(const CongestionNode &other) const
    {
        return arrivalTime > other.arrivalTime;
    }
};

// Function to check for congestion (simultaneous use of an edge) based on edge usage times
double getCongestionCost(const EdgeUsageTimes &edgeUsageTimes, const string &source, const string &sink, const double &startTime, const double &edgeTraversalTime)
{
    if (edgeUsageTimes.count(sink) && edgeUsageTimes.at(sink).count(source)) // Checks if an AGV is going in opposite direction
    {
        const vector<double> &usageTimes = edgeUsageTimes.at(sink).at(source);
        for (double usageTime : usageTimes)
        {
            if (abs(startTime - usageTime) <= edgeTraversalTime) // This means the AGVs will be using the edge at the same time.
            {
                // Additional cost when the edge is in use
                return congestionCost;
            }
        }
    }
    // No additional cost
    return 0.0;
}

// A* algorithm to find the path from start to goal waypoints without considering congestion
Path getPath(const map<string, Node> &nodes, const map<string, map<string, double>> &graph, const string &start, const string &goal)
{
    // Initialize priority queue and other storage variables
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> openList;
    map<string, double> gScore;
    map<string, string> parentNode;

    openList.push({0.0, start});
    gScore[start] = 0.0;

    while (!openList.empty())
    {
        string current = openList.top().second;
        openList.pop();

        if (current == goal)
        {
            // Reconstruct the path
            Path path;
            path.totalTime = gScore[goal];
            path.totalDist = gScore[goal] * agvSpeed;

            while (current != start)
            {
                path.waypoints.push_back(current);
                current = parentNode[current];
            }

            path.waypoints.push_back(start);
            reverse(path.waypoints.begin(), path.waypoints.end());

            return path;
        }

        for (const auto &neighbor : nodes.at(current).neighbours)
        {
            // Expand the current node for each neighbor
            double tentativeGScore = gScore[current] + graph.at(current).at(neighbor);

            if (!gScore.count(neighbor) || tentativeGScore < gScore[neighbor])
            {
                gScore[neighbor] = tentativeGScore;
                double priority = tentativeGScore + (calculateDistance(nodes.at(neighbor), nodes.at(goal)) / agvSpeed);
                openList.push({priority, neighbor});
                parentNode[neighbor] = current;
            }
        }
    }

    // If no path found
    return Path{};
}

// A* algorithm to find the path from start to goal waypoints while considering congestion
TimedPath getPath(const map<string, Node> &nodes, const map<string, map<string, double>> &graph, const string &start, const string &goal, const EdgeUsageTimes &edgeUsageTimes, const double &startTime)
{
    // Initialize priority queue and other storage variables
    priority_queue<CongestionNode, vector<CongestionNode>, greater<CongestionNode>> openList;
    map<pair<string, double>, double> gScore;
    map<CongestionNode, CongestionNode> parentNode;

    openList.push({start, startTime, 0.0});
    gScore[{start, startTime}] = 0.0;

    while (!openList.empty())
    {
        CongestionNode current = openList.top();
        openList.pop();
        if (current.waypoint == goal)
        {
            // Reconstruct the path
            TimedPath path;
            path.totalDist = gScore[{goal, current.arrivalTime}] * agvSpeed;
            path.totalTime = gScore[{goal, current.arrivalTime}];

            while (!(current.waypoint == start && current.arrivalTime == startTime))
            {
                path.waypoints.push_back(current.waypoint);
                path.arrivalTimes.push_back(current.arrivalTime);
                path.waitTimes.push_back(current.waitTime);
                current = parentNode.at(current);
            }

            path.waypoints.push_back(start);
            path.arrivalTimes.push_back(current.arrivalTime);
            path.waitTimes.push_back(current.waitTime);
            reverse(path.waypoints.begin(), path.waypoints.end());
            reverse(path.arrivalTimes.begin(), path.arrivalTimes.end());
            reverse(path.waitTimes.begin(), path.waitTimes.end());
            return path;
        }

        for (const auto &neighbor : nodes.at(current.waypoint).neighbours)
        {
            // Expand the current node for each neighbor
            double waitTime = 0.0;
            double edgeTravelTime = graph.at(current.waypoint).at(neighbor);

            // Calculate the minimum wait time required to have no congestion
            while (waitTime <= maxWait)
            {
                double extraCost = getCongestionCost(edgeUsageTimes, current.waypoint, neighbor, current.arrivalTime + waitTime, edgeTravelTime);

                if (extraCost < epsilon)
                {
                    double newArrivalTime = current.arrivalTime + waitTime + edgeTravelTime; // newArrivalTime at the next waypoint
                    CongestionNode neighborNode{neighbor, newArrivalTime, waitTime};
                    if (!gScore.count({neighborNode.waypoint, neighborNode.arrivalTime}) || newArrivalTime < gScore[{neighborNode.waypoint, neighborNode.arrivalTime}]) // This is acceptable as long as system follows the FIFO property.
                    {
                        gScore[{neighborNode.waypoint, neighborNode.arrivalTime}] = newArrivalTime;
                        double priority = newArrivalTime + (calculateDistance(nodes.at(neighbor), nodes.at(goal)) / agvSpeed);
                        openList.push({neighbor, newArrivalTime, waitTime});
                        parentNode[neighborNode] = current;
                    }
                    break;
                }
                waitTime += 1.0; // Increment wait time if current departure time is not congestion-free
            }
        }
    }

    // If no path found
    return TimedPath{};
}

Path combinePaths(const Path &path1, const string &action, const Path &path2)
{
    Path combinedPath;

    // Combine waypoints
    combinedPath.waypoints = path1.waypoints;
    combinedPath.waypoints.push_back(action);
    combinedPath.waypoints.insert(combinedPath.waypoints.end(), path2.waypoints.begin() + 1, path2.waypoints.end());

    // Update total distance and time
    combinedPath.totalDist = path1.totalDist + path2.totalDist;
    combinedPath.totalTime = path1.totalTime + path2.totalTime;

    return combinedPath;
}

TimedPath combinePaths(const TimedPath &path1, const string &action, const TimedPath &path2)
{
    TimedPath combinedPath;

    // Combine waypoints
    combinedPath.waypoints = path1.waypoints;
    combinedPath.waypoints.push_back(action);
    combinedPath.waypoints.insert(combinedPath.waypoints.end(), path2.waypoints.begin() + 1, path2.waypoints.end());

    // Combine arrival times
    combinedPath.arrivalTimes = path1.arrivalTimes;
    combinedPath.arrivalTimes.push_back(path1.totalTime);
    combinedPath.arrivalTimes.insert(combinedPath.arrivalTimes.end(), path2.arrivalTimes.begin() + 1, path2.arrivalTimes.end());

    // Combine wait times
    combinedPath.waitTimes = path1.waitTimes;
    combinedPath.waitTimes.push_back(0.0);
    combinedPath.waitTimes.insert(combinedPath.waitTimes.end(), path2.waitTimes.begin() + 1, path2.waitTimes.end());

    // Update total distance and time
    combinedPath.totalDist = path1.totalDist + path2.totalDist;
    combinedPath.totalTime = path1.totalTime + path2.totalTime;
    if (path2.arrivalTimes.size() > 0)
    {
        combinedPath.totalTime -= path2.arrivalTimes[0]; // If the path2 started at a non-zero time, subtract that from total time.
    }

    return combinedPath;
}

// Print a simple path
void printPath(const Path &path, const string &robotArm, const string &box)
{
    if (!path.waypoints.empty())
    {
        cout << "Path to get " << box << " to " << robotArm << ": ";
        for (const auto &waypoint : path.waypoints)
        {
            cout << waypoint << " -> ";
        }
        cout << "End." << endl;
        cout << "Total path time is " << path.totalTime << " seconds. ";
        cout << endl;
    }
    else
    {
        cout << "Path is empty." << endl;
    }
}

// Print a timed path
void printPath(const TimedPath &path, const string &robotArm, const string &box)
{
    if (!path.waypoints.empty() && path.waypoints.size() == path.arrivalTimes.size() && path.waypoints.size() == path.waitTimes.size())
    {
        cout << "Path to get " << box << " to " << robotArm << ": ";
        for (size_t i = 0; i < path.waypoints.size(); ++i)
        {
            if (i < path.waitTimes.size() - 1)
            {
                if (path.waitTimes[i] > epsilon)
                {
                    cout << "Wait for " << path.waitTimes[i] << " seconds -> ";
                }
            }

            cout << path.waypoints[i] << " (" << path.arrivalTimes[i] << ")";
            if (i < path.waypoints.size() - 1)
            {
                cout << " -> ";
            }
        }
        cout << endl;
        cout << "Total path time is " << path.totalTime << " seconds. ";
        cout << endl;
    }
    else
    {
        cout << "Path is empty or contains inconsistent data." << endl;
    }
}
