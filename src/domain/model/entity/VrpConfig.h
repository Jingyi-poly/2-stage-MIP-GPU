#ifndef AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_CONFIG_H_
#define AMDAHL_SRC_DOMAIN_MODEL_ENTITY_VRP_CONFIG_H_

class VrpConfig {
   public:
    double vehicleCost{1E5};
    double twPenaltyCoeff{1E5};
    double capacityPenaltyCoeff{1E5};

    // Scenario evaluators store capacity violation as a raw sum over scenarios.
    // When enabled, memetic interprets capacityPenaltyCoeff as the coefficient
    // of the scenario mean and scales it (and its adaptive bounds) by 1/N once
    // at solver entry. The default preserves legacy deterministic callers.
    bool useMeanScenarioCapacityViolation{false};
};

#endif
