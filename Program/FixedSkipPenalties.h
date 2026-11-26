#pragma once

#include "Params.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

// Isolated experiment extension. The default native penalty generator runs
// first; this file replaces its values only when the environment variable is
// explicitly supplied. Both the exact frontend and evaluator use this code.
inline void apply_fixed_skip_penalties(Params& params) {
    const char* path = std::getenv("PCTSP_SKIP_PENALTIES_FILE");
    if (!path || !*path) return;
    if (!params.ap.optionalVisit)
        throw std::runtime_error("fixed skip penalties require optionalVisit");
    std::ifstream in(path);
    if (!in) throw std::runtime_error(std::string("cannot open fixed skip penalties: ") + path);
    std::string header;
    if (!std::getline(in, header))
        throw std::runtime_error("fixed skip penalties need client,penalty header");
    if (!header.empty() && header.back() == '\r') header.pop_back();
    if (header != "client,penalty")
        throw std::runtime_error("fixed skip penalties need client,penalty header");
    for (int i = 1; i <= params.nbClients; ++i) {
        std::string line;
        if (!std::getline(in, line)) throw std::runtime_error("fixed skip penalty row missing");
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto split = line.find(',');
        if (split == std::string::npos || std::stoi(line.substr(0, split)) != i)
            throw std::runtime_error("fixed skip penalty client mismatch");
        const double value = std::stod(line.substr(split + 1));
        if (!std::isfinite(value) || value < 0)
            throw std::runtime_error("invalid fixed skip penalty");
        params.cli[i].skipPenalty = value;
    }
    std::string extra;
    if (std::getline(in, extra)) throw std::runtime_error("extra fixed skip penalty row");
    std::cout << "FIXED_SKIP_PENALTIES_FILE " << path << "\nFIXED_SKIP_PENALTIES";
    for (int i = 1; i <= params.nbClients; ++i)
        std::cout << ' ' << std::setprecision(17) << params.cli[i].skipPenalty;
    std::cout << '\n';
}
