#ifndef CCBP_ND_AUTOMATA_H
#define CCBP_ND_AUTOMATA_H

#include "automaton_base.h"

#include <iosfwd>
#include <utility>
#include <vector>

namespace ccbp {
/*
  Non-deterministic automaton over the global actions, as NonDeterministicAutomaton of the
  driver (driver/automata/nondeterministic_automaton.py): global actions are mapped to local
  actions (-1: none), and each state has, for each local action, a set of
  successor states.
*/
struct NDAutomaton : AutomatonBase {
    // For each state: (local action, sorted successors), sorted by local action, no empty successor set.
    std::vector<std::vector<std::pair<int, std::vector<int>>>> transitions;
};

struct NDAutomata : AutomataBase<NDAutomaton> {
};

/*
  Same layout as the deterministic automata file (header "Actions:",
  "NonUnitCost:", "Costs:", "Automata:"), with the state lines of the CP
  automata of NonDeterministicAutomaton._compose_transitions_to:
  "<#actions>  <action> <#successors> <successors>  ...".
*/
NDAutomata read_nd_automata(std::istream &in);
void write_nd_automata(const NDAutomata &automata, std::ostream &out);

/*
  The global actions grouped by the behaviour of their local action (same
  successor sets in every state, same cost), as
  the former AutomatonND.get_global_equivalent_action_sets: each set sorted, the sets
  ordered by their smallest action. The automaton must be reduced
  (reduce_local_equivalent_actions): two local actions then never behave the
  same, so the sets are the global actions of each local action.
*/
std::vector<std::vector<int>> equivalent_action_sets(const NDAutomaton &automaton);

/*
  Merge the local actions with the same successor sets in every state and the
  same cost, and drop the local actions no global action maps to
  (NonDeterministicAutomaton._reduce_local_equivalent_actions).
*/
void reduce_local_equivalent_actions(NDAutomaton &automaton);

/*
  Reduce the global actions (the former AutomataND.apply_global_mapping): group g holds
  the old actions of the new action g, sorted increasingly. In theta, the new
  action has the union of the transitions of its old actions; elsewhere, the
  old actions of a group are equivalent and only the mapping changes.
*/
void apply_nd_reduction(NDAutomata &automata, int theta_index, const std::vector<std::vector<int>> &groups);

/*
  Remove the states that are not reachable from the initial state or from
  which no goal state is reachable; the kept states keep their order. Returns
  false (and changes nothing) if the initial state would be removed. With
  state_mapping, sets it to the new state of each old state (-1: removed).
*/
bool prune_unreachable_and_dead_end_states(NDAutomaton &automaton, std::vector<int> *state_mapping = nullptr);

/*
  Exact bisimulation shrinking as in Scorpion (the former AutomatonND.shrink_bisimulation):
  remove the unreachable and dead-end states, then merge the bisimilar states.
  Returns the number of removed states. With state_mapping, sets it to the new
  state of each old state (-1: removed); it is left unchanged if the initial
  state is a dead end (nothing is removed then).
*/
int shrink_bisimulation(NDAutomaton &automaton, std::vector<int> *state_mapping = nullptr);
}

#endif
