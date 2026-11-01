#ifndef CCBP_AUTOMATON_BASE_H
#define CCBP_AUTOMATON_BASE_H

#include <vector>

namespace ccbp {
/*
  What deterministic and non-deterministic automata share (as the Automaton
  base class of the driver): the global actions are mapped to local actions,
  and the transitions (in the derived types) are given over the local actions.
*/
struct AutomatonBase {
    int num_states = 0;
    int init_state = 0;
    std::vector<int> goal_states;
    int num_local_actions = 0;
    std::vector<int> local_mapping; // global action -> local action (-1: none, non-deterministic automata only)
    bool has_costs = false;
    std::vector<int> local_costs;   // only meaningful if has_costs

    // Cost of a local action as the driver reads it (local action -1 is the last one, as a Python index).
    int local_cost(int local_action) const {
        if (local_action == -1)
            return local_costs.empty() ? 0 : local_costs.back();
        return local_costs[local_action];
    }
};

// Automata over a common set of global actions.
template<typename AutomatonType>
struct AutomataBase {
    std::vector<int> costs; // cost of each global action
    std::vector<AutomatonType> automata;

    int num_actions() const {return static_cast<int>(costs.size());}
};
}

#endif
