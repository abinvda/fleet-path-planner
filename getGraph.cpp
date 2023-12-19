// Get the graph from the given waypoint file

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h>

using namespace std;

struct Node
{ // Defined by its pose and also contains a list of its neighbours
    double x;
    double y;
    double theta; // Not using theta for now
    vector<string> neighbours;
};

map<string, Node> parseNodes(const string &filename)
{
    ifstream file(filename);
    map<string, Node> nodes;

    if (file.is_open())
    {
        YAML::Node waypointsNode = YAML::Load(file);
        for (const auto &node : waypointsNode["waypoints"])
        {
            for (const auto &entry : node)
            {
                Node n;
                n.x = entry.second["x"].as<double>();
                n.y = entry.second["y"].as<double>();
                n.theta = entry.second["theta"].as<double>();

                for (const auto &neighbor : entry.second["neighbours"])
                {
                    n.neighbours.push_back(neighbor.as<string>());
                }

                nodes[entry.first.as<string>()] = n;
            }
        }
    }
    else
    {
        cerr << "Unable to open file: " << filename << endl;
    }

    return nodes;
}

double calculateDistance(const Node &n1, const Node &n2)
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

map<string, map<string, double>> buildGraph(const map<string, Node> &nodes, double agvSpeed)
{
    map<string, map<string, double>> graph;

    for (const auto &entry : nodes)
    {
        const string &currentID = entry.first;
        const Node &currentNode = entry.second;
        map<string, double> edges;

        for (const auto &neighborId : currentNode.neighbours)
        {
            const Node &neighborNode = nodes.at(neighborId);
            double distance = calculateDistance(currentNode, neighborNode);
            double travelTime = distance / agvSpeed; // Time = Distance / Speed
            edges[neighborId] = travelTime;
        }

        graph[currentID] = edges;
    }

    return graph;
}

string identifyWaypoint(const map<string, Node> &nodes, double x, double y)
{
    // Returns the waypoint closest to the given location
    string closestWaypoint;
    double minDistance = numeric_limits<double>::max();

    for (const auto &entry : nodes)
    {
        const Node &currentNode = entry.second;
        double distance = calculateDistance({x, y, 0.0}, currentNode);

        if (distance < minDistance)
        {
            minDistance = distance;
            closestWaypoint = entry.first;
        }
    }

    return closestWaypoint;
}

void printGraph(const map<string, map<string, double>> &graph)
{
    for (const auto &entry : graph)
    {
        const string &vertex1 = entry.first;
        const map<string, double> &edges = entry.second;

        for (const auto &edge : edges)
        {
            cout << "Edge: " << vertex1 << "-" << edge.first << " : " << edge.second << " ";
        }
        cout << endl;
    }
}