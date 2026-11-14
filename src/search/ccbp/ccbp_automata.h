#ifndef CCBP_CCBP_AUTOMATA_H
#define CCBP_CCBP_AUTOMATA_H

#include "pipeline.h"

#include "../evaluator.h"
#include "../search_algorithm.h"
#include "../landmarks/landmark_factory.h"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace ccbp {
/*
  The automata of CCBP (Collaborative Constraint-Based Planning): builds the
  atomic automata of the task, refines them (preprocessing, merges, label
  reduction, shrinking) and writes the CP automata, the constraints file and
  the files used to translate the plans, then computes the plan length bounds
  into the constraints file. It does not search itself: it exits once the
  files are written.
*/
class CCBPAutomata : public SearchAlgorithm {
    PipelineOptions options;
    bool unit_cost_automata;
    std::shared_ptr<Evaluator> operator_counting;
    // Bound to the unit-cost version of the task in write_constraint_inputs.
    std::shared_ptr<landmarks::TaskIndependentLandmarkFactory> landmark_factory;
    std::shared_ptr<Evaluator> lower_bound;
    // Bound to the task only when they run.
    std::vector<std::shared_ptr<TaskIndependentSearchAlgorithm>>
        lower_bound_searches;
    std::vector<std::shared_ptr<TaskIndependentSearchAlgorithm>>
        upper_bound_searches;
    double max_time_for_bounds;
    std::optional<std::pair<long long, long long>> fixed_plan_length;

    // The operator-counting LP and the action landmark graph over the original operators.
    void write_constraint_inputs();
    void append_bounds(long long plan_min, long long plan_max) const;
    std::shared_ptr<SearchAlgorithm> construct_search(
        const std::shared_ptr<TaskIndependentSearchAlgorithm> &config) const;
    // The plan length bounds, appended to the constraints file.
    void compute_bounds(long long automata_lower_bound);

protected:
    virtual SearchStatus step() override;

public:
    CCBPAutomata(
        const std::shared_ptr<AbstractTask> &task,
        const PipelineOptions &options, bool unit_cost_automata,
        const std::shared_ptr<Evaluator> &operator_counting,
        const std::shared_ptr<landmarks::TaskIndependentLandmarkFactory>
            &landmark_factory,
        const std::shared_ptr<Evaluator> &lower_bound,
        const std::vector<std::shared_ptr<TaskIndependentSearchAlgorithm>>
            &lower_bound_searches,
        const std::vector<std::shared_ptr<TaskIndependentSearchAlgorithm>>
            &upper_bound_searches,
        double max_time_for_bounds,
        const std::optional<std::pair<long long, long long>> &fixed_plan_length,
        OperatorCost cost_type, int bound, double max_time,
        const std::string &description, utils::Verbosity verbosity);

    virtual void print_statistics() const override;
    virtual bool is_complete_within_bound() const override {
        return false;
    }
};
}

#endif
