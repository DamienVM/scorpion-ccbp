#ifndef CCBP_PREPROCESS_H
#define CCBP_PREPROCESS_H

#include "../abstract_task.h"

#include <unordered_map>
#include <vector>

namespace ccbp {
struct Automaton;
struct NDAutomaton;
struct NDAutomata;

// The facts of an operator, sorted and without duplicates.
struct OperatorFacts {
    std::vector<FactPair> preconditions; // including the preconditions of the effects
    std::vector<FactPair> effects;
    std::vector<FactPair> postconditions; // true after the operator: its effects and its unchanged preconditions
};

// The operators of the task (throws on conditional effects).
std::vector<OperatorFacts> get_operator_facts(const AbstractTask &task);

/*
  PartialState.operator_applicable of the former driver, for the partial state
  partial_state (variable -> value): no precondition of op is contradicted by
  the value of its variable nor, with use_mutex_relations, mutex with a value.
*/
bool operator_applicable(const AbstractTask &task, const std::unordered_map<int, int> &partial_state,
                         const OperatorFacts &op, bool use_mutex_relations);

/*
  Automaton.prune_inapplicable_transitions (--remove-mutex-transitions): in
  each state, remove the transitions of the actions whose preconditions
  contradict the fact of the state, or are mutex with it, and the transitions
  whose target fact is mutex with a postcondition of the action (the state
  after the action would violate a mutex, so the action is never applied
  there). The automaton must be atomic, with its state facts. Returns the
  number of removed transitions.
*/
int prune_inapplicable_transitions(Automaton &automaton, const AbstractTask &task,
                                   const std::vector<OperatorFacts> &operators);

/*
  Automata.add_no_op_action (--no-op): a zero-cost action that loops on the
  goal states of every automaton, and, with right_pad_automaton, an extra
  automaton that forbids other actions after a no-op. Updates the action cost
  flag as the driver (0 becomes 1).
*/
void add_no_op_action(NDAutomata &automata, int &action_cost_flag, bool right_pad_automaton);

// Automata.add_no_op_action2 (--no-op-2): the no-op leads from the goal states to a new goal state where it loops.
void add_no_op_action2(NDAutomata &automata, int &action_cost_flag);

// The former AutomatonND.from_automaton.
NDAutomaton to_nd_automaton(const Automaton &automaton);

/*
  The automaton itself as a deterministic automaton, with the same states and
  local actions. Every local action must have at most one successor per state.
*/
Automaton deterministic_copy(const NDAutomaton &automaton);

// NonDeterministicAutomaton.has_local_mapping / _remove_local_mapping of the driver.
bool has_local_mapping(const NDAutomaton &automaton);
void remove_local_mapping(NDAutomaton &automaton);
}

#endif
