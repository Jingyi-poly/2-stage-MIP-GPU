#include "model/solution.h"

#include <iostream>
#include <numeric>

void Solution::Evaluate() {
    this->unserved_tasks_.resize(this->problem_.task_num - 1);
    std::iota(this->unserved_tasks_.begin(), this->unserved_tasks_.end(), 1);
    this->unserved_task_num_ = this->problem_.task_num - 1;

    this->route_distance_.assign(this->routes_.size(), 0.0);
    this->tw_violations_.assign(this->routes_.size(), 0.0);
    this->capacity_violations_.assign(this->routes_.size(), 0);
    this->total_distance_ = 0.0;
    this->total_tw_violations_ = 0.0;
    this->total_capacity_violations_ = 0;
    this->total_pickup_delivery_precedence_violations_ = 0;
    this->total_pickup_delivery_pairing_violations_ = 0;
    this->total_infeasible_arcs_violations_ = 0;

    std::vector<real_t> arrival_times;
    std::vector<real_t> departure_times;
    std::vector<uint8_t> task_visited_flags(this->problem_.task_num, 0);
    std::vector<idx_t> request_route_index(this->problem_.task_num / 2, -1);
    for (size_t i = 0; i < this->routes_.size(); ++i) {
        const std::vector<idx_t>& route = this->routes_[i];
        if (route.empty()) {
            continue;
        }

        if (4 > (int)route.size()) {
            std::cout << "Error: Route " << i << " length is less than 4." << std::endl;
            exit(0);
        }
        if (0 != route[0]) {
            std::cout << "Error: Route " << i << " is not started from depot." << std::endl;
            exit(0);
        }
        task_visited_flags[0] = 1;
        arrival_times.resize(route.size(), 0.0);
        departure_times.resize(route.size(), 0.0);
        departure_times[0] = GetEarliestStartTime(problem_, 0) + problem_.task_service_times[0];

        real_t tw_violation = 0.0;
        demand_t capacity_violation = 0;
        std::vector<demand_t> accumulated_demands(problem_.scenario_num, 0);
        real_t accumulated_distance = 0.0;
        idx_t prev_task_index = 0;
        real_t start_service_time;
        for (size_t j = 1; j < route.size(); ++j) {
            idx_t current_task_index = route[j];
            if (0 == IsArcFeasible(problem_, prev_task_index, current_task_index)) {
                this->total_infeasible_arcs_violations_ += 1;
            }
            real_t distance = GetDistance(problem_, prev_task_index, current_task_index);
            arrival_times[j] = departure_times[j - 1] + distance;
            start_service_time =
                std::min(std::max(arrival_times[j], GetEarliestStartTime(problem_, current_task_index)),
                         GetLatestStartTime(problem_, current_task_index));
            departure_times[j] = start_service_time + problem_.task_service_times[current_task_index];

            tw_violation += std::max(0.0, arrival_times[j] - GetLatestStartTime(problem_, current_task_index));
            for (int i = 0; i < problem_.scenario_num; ++i) {
                accumulated_demands[i] += problem_.task_demands[current_task_index][i];
                capacity_violation += std::max(0, accumulated_demands[i] - problem_.vehicle_capacity);
            }

            accumulated_distance += distance;

            if (0 != current_task_index) {
                if (0 == current_task_index % 2) {
                    if (!task_visited_flags[current_task_index - 1]) {
                        this->total_pickup_delivery_precedence_violations_ += 1;
                    } else if (i != request_route_index[(current_task_index - 1) / 2]) {
                        this->total_pickup_delivery_pairing_violations_ += 1;
                    }
                } else {
                    request_route_index.at(current_task_index / 2) = (idx_t)i;
                }
            }

            task_visited_flags[current_task_index] = 1;
            prev_task_index = current_task_index;
        }

        if (0 != route.back()) {
            std::cout << "Error: Route " << i << " is not returned to depot." << std::endl;
            exit(0);
        }

        for (idx_t j = 0; j < this->unserved_task_num_;) {
            if (task_visited_flags[this->unserved_tasks_[j]]) {
                this->unserved_tasks_[j] = this->unserved_tasks_[this->unserved_task_num_ - 1];
                this->unserved_task_num_ -= 1;
            } else {
                ++j;
            }
        }
        this->unserved_tasks_.resize(this->unserved_task_num_);

        this->route_distance_[i] = accumulated_distance;
        this->total_distance_ += accumulated_distance;

        this->tw_violations_[i] = tw_violation;
        this->total_tw_violations_ += tw_violation;

        this->capacity_violations_[i] = capacity_violation;
        this->total_capacity_violations_ += capacity_violation;
    }

    if (this->total_infeasible_arcs_violations_ > 0 || this->total_pickup_delivery_precedence_violations_ > 0 ||
        this->total_pickup_delivery_pairing_violations_ > 0) {
        printf("Error: Invalid solution\n");
        exit(0);
    }
}
