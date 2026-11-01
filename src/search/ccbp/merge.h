#ifndef CCBP_MERGE_H
#define CCBP_MERGE_H

#include "../abstract_task.h"

#include <optional>
#include <vector>

namespace ccbp {
struct Automaton;
struct Automata;

/*
  FactsRelations of the former driver (driver/sas/facts_relations.py): tells
  whether the facts of a state of a merged automaton can hold together.
*/
class FactsRelations {
    const AbstractTask &task;

public:
    explicit FactsRelations(const AbstractTask &task) : task(task) {}

    // FactsRelations.is_state_valid for a set of facts: no two of them are mutex in the task.
    bool is_state_valid(const std::vector<FactPair> &state) const;
};

/*
  synchronize_automatons: the product of the automata restricted to its
  reachable states, without the states whose facts are not valid. States are
  numbered in breadth-first order, the actions are the global actions. With
  max_states >= 0, the product is abandoned (nullopt) as soon as it has more
  than max_states states, so that its construction stays bounded.
*/
std::optional<Automaton> synchronize_automata(const std::vector<const Automaton *> &automata, int action_cost_flag,
                                              const FactsRelations &relations, int max_states = -1);

/*
  Remove the states of an automaton that are unreachable from its initial
  state or from which no goal state is reachable, then, with shrink, merge its
  bisimilar states (exact bisimulation). The facts of a merged state are the
  facts that hold in all its states, so the mutex checks of later merges stay
  sound. The automaton is left as it is when no state is removed. Returns the
  number of removed states; throws if the initial state is a dead end (the
  task is unsolvable).
*/
int prune_and_shrink(Automaton &automaton, bool shrink);

/*
  MergeMin (--merge-min PERCENT): merge the two automata with the fewest
  states until the number of automata is reduced by PERCENT. Every product is
  pruned (and shrunk with shrink) by prune_and_shrink. With max_states >= 0,
  merging stops at the first product (of the two smallest automata) with more
  than max_states reachable states. Prints the merges as the driver.
*/
void merge_min(Automata &automata, int percent, int action_cost_flag, const AbstractTask &task,
               const FactsRelations &relations, bool shrink, int max_states = -1);

/*
  MergeInvariantsObjects (--merge-objects): merge the automata of the
  variables whose atoms all share one object. Every product is pruned (and
  shrunk with shrink) by prune_and_shrink. With max_states >= 0, the merge of
  an object whose product has more than max_states reachable states is
  skipped. Prints the merges as the driver.
*/
void merge_objects(Automata &automata, int action_cost_flag, const AbstractTask &task, const FactsRelations &relations,
                   bool error_on_missing_automaton, bool shrink, int max_states = -1);
}

#endif
