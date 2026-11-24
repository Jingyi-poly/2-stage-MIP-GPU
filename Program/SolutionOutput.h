#ifndef HGS_SOLUTION_OUTPUT_H
#define HGS_SOLUTION_OUTPUT_H

#include "Individual.h"

#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

inline void writeSolution(const Individual& best, const std::string& path, bool optionalVisit)
{
    std::ofstream output(path);
    if (!output) throw std::runtime_error("Cannot open solution file: " + path);

    int routeNumber = 0;
    for (const auto& route : best.chromR)
    {
        bool started = false;
        for (int client : route)
        {
            if (optionalVisit && !best.clientVisited[client]) continue;
            if (!started)
            {
                output << "Route #" << ++routeNumber << ':';
                started = true;
            }
            output << ' ' << client;
        }
        if (started) output << '\n';
    }
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "Cost " << best.eval.penalizedCost << '\n';
    output.close();
    if (!output) throw std::runtime_error("Cannot write solution file: " + path);
}

#endif
