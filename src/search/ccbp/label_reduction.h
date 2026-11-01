#ifndef CCBP_LABEL_REDUCTION_H
#define CCBP_LABEL_REDUCTION_H

#include "deterministic_sets.h"
#include "operator_counting.h"
#include "reduction_loop.h"

#include <string>
#include <vector>

namespace ccbp {
struct Automata;

/*
  Deterministic label reduction of the former CCBP driver (driver/label_reduction):
  for each automaton theta (last to first), the actions that are locally
  equivalent in every other automaton are split into sets that can be merged
  deterministically in theta, and each set is reduced to a single action.

  The reduction keeps track of the composed mapping from the actions it was
  given to the current actions.
*/
class LabelReduction {
    std::vector<ConflictRule> pass_rules;
    ReductionState state;

public:
    // Supported strategies: greedy, sss, dss, sss_greedy, lropc, lropc_sss_greedy.
    explicit LabelReduction(const std::string &strategy);

    // True for the strategies restricted to the operator-counting partition.
    bool uses_operator_counting() const {return state.restrict_to_operator_counting;}

    /*
      Operator-counting LP of the lropc strategies. original_to_initial[o] is
      the initial action of the original operator o (empty: the identity).
    */
    void set_operator_counting(OperatorCountingLP lp, std::vector<int> original_to_initial);

    // One reduction step (for sss_greedy: an sss pass then a greedy pass).
    // Returns true iff at least one reduction was applied.
    bool apply_all_automata(Automata &automata);

    // Repeat apply_all_automata until nothing changes. Returns the number of
    // steps that applied a reduction.
    int apply_fix_point(Automata &automata);

    // mapping()[a] is the current action of the initial action a.
    const std::vector<int> &get_mapping() const {return state.mapping;}
};
}

#endif
