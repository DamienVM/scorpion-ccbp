#ifndef CCBP_AUTOMATA_H
#define CCBP_AUTOMATA_H

#include "automaton_base.h"

#include "../abstract_task.h"

#include <iosfwd>
#include <vector>

namespace ccbp {
/*
  Deterministic automaton over the global actions, in the layout of the CCBP
  automata file: global actions are mapped to local actions, and the
  transition function is a dense (state x local action) matrix where -1 means
  that the local action is not applicable in the state.
*/
struct Automaton : AutomatonBase {
    std::vector<int> transitions;   // row-major: [state * num_local_actions + local action]
    // For the merges: the variables of the task in the automaton (sorted) and the facts holding in each state
    // (sorted), as Automaton._atoms and Automaton._state_facts.
    std::vector<int> variables;
    std::vector<std::vector<FactPair>> state_facts;

    int target(int state, int local_action) const {
        return transitions[state * num_local_actions + local_action];
    }
};

struct Automata : AutomataBase<Automaton> {
    int non_unit_cost = 0;  // "NonUnitCost:" of the file
};

Automata read_automata(std::istream &in);

/*
  Automata of a CP automata file (CPAutomataParser of the driver, action cost
  flags 0 to 3): returns the automata and sets the plan length bounds and the
  action cost flag of the file.
*/
Automata read_cp_automata(std::istream &in, int &action_cost_flag);
void write_automata(const Automata &automata, std::ostream &out);

// Automaton.has_local_mapping: the local actions are not the global actions themselves.
bool has_local_mapping(const Automaton &automaton);

// Automaton._remove_local_mapping: one local action per global action.
void remove_local_mapping(Automaton &automaton);

/*
  Automaton._reduce_local_equivalent_actions: without a local mapping, the
  global actions with the same transitions and cost become one local action
  (ids by first occurrence). Does nothing if the automaton has a local mapping.
*/
void reduce_local_equivalent_actions(Automaton &automaton);
}

#endif
