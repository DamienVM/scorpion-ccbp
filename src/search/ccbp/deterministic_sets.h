#ifndef CCBP_DETERMINISTIC_SETS_H
#define CCBP_DETERMINISTIC_SETS_H

#include <utility>
#include <vector>

namespace ccbp {
struct Automaton;

enum class ConflictRule {
    // Conflict iff not deterministically reducible (same state, different
    // targets) or different local costs.
    GREEDY,
    // GREEDY + distinct local actions without a common successor state.
    SSS,
    // GREEDY + local actions (including a local action with itself) sharing
    // a successor state.
    DSS,
};

// For each local action of an automaton: its (state, target) transitions, by state.
using LocalTransitions = std::vector<std::vector<std::pair<int, int>>>;

LocalTransitions compute_local_transitions(const Automaton &automaton);

/*
  Split the global actions of `block` (sorted increasingly) into sets that can
  be reduced together in `theta`: build the conflict graph over the block and
  repeatedly extract a greedy maximal independent set (minimum virtual degree
  first, smallest action id on ties). `local_transitions` must be the result of
  compute_local_transitions(theta); with it, the work for a block only depends
  on the transitions of its local actions, not on the size of theta.
*/
std::vector<std::vector<int>> compute_deterministic_sets(
    const Automaton &theta, const LocalTransitions &local_transitions,
    const std::vector<int> &block, ConflictRule rule);
}

#endif
