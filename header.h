#ifndef PATH_STRUCT_H
#define PATH_STRUCT_H

#include <vector>
#include <string>

struct TimedPath {
    std::vector<std::string> waypoints;
    std::vector<double> arrivalTimes;
    std::vector<double> waitTimes;
    double totalDist;
    double totalTime;
    std::string selectedAGVID;
};

#endif // PATH_STRUCT_H