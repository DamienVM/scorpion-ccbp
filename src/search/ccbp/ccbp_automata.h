#ifndef CCBP_CCBP_AUTOMATA_H
#define CCBP_CCBP_AUTOMATA_H

#include "pipeline.h"

#include "../search_algorithm.h"
#include "../parser/decorated_abstract_syntax_tree.h"

#include <memory>
#include <optional>
#include <utility>
#include <vector>

class Evaluator;

namespace landmarks {
class LandmarkFactory;
}

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
    std::shared_ptr<landmarks::LandmarkFactory> landmark_factory;
    std::shared_ptr<Evaluator> lower_bound;
    std::vector<parser::LazyValue> lower_bound_searches;
    std::vector<parser::LazyValue> upper_bound_searches;
    double max_time_for_bounds;
    std::optional<std::pair<long long, long long>> fixed_plan_length;

    // The operator-counting LP and the action landmark graph over the original operators.
    void write_constraint_inputs();
    void append_bounds(long long plan_min, long long plan_max) const;
    std::shared_ptr<SearchAlgorithm> construct_search(const parser::LazyValue &config) const;
    // The plan length bounds, appended to the constraints file.
    void compute_bounds(long long automata_lower_bound);

protected:
    virtual SearchStatus step() override;

public:
    explicit CCBPAutomata(const plugins::Options &opts);

    virtual void print_statistics() const override;
};
}

#endif
