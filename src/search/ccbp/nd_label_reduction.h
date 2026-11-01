#ifndef CCBP_ND_LABEL_REDUCTION_H
#define CCBP_ND_LABEL_REDUCTION_H

#include "operator_counting.h"
#include "reduction_loop.h"

#include <string>
#include <vector>

namespace ccbp {
struct NDAutomata;

/*
  Non-deterministic label reduction of the former driver
  (driver/label_reduction/nd_label_reduction.py): for each automaton theta
  (last to first), the actions that are equivalent in every other automaton
  are reduced to a single action, whose transitions in theta are the union of
  theirs. lropc (NDLROPCLabelReduction) only merges actions lying in the same
  set of the operator-counting partition.

  The reduction keeps track of the composed mapping from the actions it was
  given to the current actions.
*/
class NDLabelReduction {
    ReductionState state;

public:
    // Supported strategies: greedy, lropc.
    explicit NDLabelReduction(const std::string &strategy);

    bool uses_operator_counting() const {return state.restrict_to_operator_counting;}
    void set_operator_counting(OperatorCountingLP lp, std::vector<int> original_to_initial);

    // One pass over all automata. Returns true iff at least one reduction was applied.
    bool apply_all_automata(NDAutomata &automata);

    // Repeat apply_all_automata until nothing changes. Returns the number of passes that applied a reduction.
    int apply_fix_point(NDAutomata &automata);

    // mapping()[a] is the current action of the initial action a.
    const std::vector<int> &get_mapping() const {return state.mapping;}
};
}

#endif
