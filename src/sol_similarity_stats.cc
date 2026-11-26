#include <iostream>

#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "util/convert_utils.h"
#include "util/sol_similarity.h"

int main(int argc, char* argv[]) {
    if (4 != argc) {
        std::cout << "Usage: " << argv[0] << " <instance_file> <solution_file1> <solution_file2>" << std::endl;
        return 1;
    }

    std::string instanceFilePath = argv[1];
    std::string solutionFilePath1 = argv[2];
    std::string solutionFilePath2 = argv[3];

    PDPTW problem;
    ReadPDPTW(instanceFilePath, problem);

    Solution firstSolution(problem);
    Solution secondSolution(problem);

    ReadSol(solutionFilePath1, firstSolution);
    ReadSol(solutionFilePath2, secondSolution);

    VrpProblem vrpProblem;
    ConvertVrpProblem(problem, vrpProblem);

    VrpSolution firstSol(&vrpProblem);
    ConvertVrpSolution(firstSolution, firstSol);

    VrpSolution secondSol(&vrpProblem);
    ConvertVrpSolution(secondSolution, secondSol);

    printf("First solution: %zu, %.10f, %.10f, %.10f\n", firstSol.routes.size(), firstSol.getTotalDistance(),
           firstSol.getTotalTwViolation(), firstSol.getTotalCapacityViolation());
    printf("Second solution: %zu, %.10f, %.10f, %.10f\n", secondSol.routes.size(), secondSol.getTotalDistance(),
           secondSol.getTotalTwViolation(), secondSol.getTotalCapacityViolation());

    double similarity = calculateSimilarity(firstSol, secondSol);

    std::cout.precision(2);
    std::cout.flags(std::ostream::fixed);
    std::cout << "Similarity: " << similarity << std::endl;
    return 0;
}