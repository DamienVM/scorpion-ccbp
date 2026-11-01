#include "preprocess.h"

#include "automata.h"
#include "nd_automata.h"

#include "../task_proxy.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <unordered_map>

using namespace std;

namespace ccbp {
vector<OperatorFacts> get_operator_facts(const AbstractTask &task) {
    auto sorted_unique = [](vector<FactPair> &facts) {
        sort(facts.begin(), facts.end());
        facts.erase(unique(facts.begin(), facts.end()), facts.end());
    };
    vector<OperatorFacts> operators;
    for (OperatorProxy op : TaskProxy(task).get_operators()) {
        OperatorFacts &facts = operators.emplace_back();
        // The preconditions of the task include the preconditions of the effects.
        for (FactProxy precondition : op.get_preconditions())
            facts.preconditions.push_back(precondition.get_pair());
        for (EffectProxy effect : op.get_effects()) {
            if (!effect.get_conditions().empty())
                throw runtime_error("Conditional effects are not supported yet.");
            facts.effects.push_back(effect.get_fact().get_pair());
        }
        sorted_unique(facts.preconditions);
        sorted_unique(facts.effects);
        facts.postconditions = facts.effects;
        for (const FactPair &precondition : facts.preconditions) {
            bool changed = any_of(facts.effects.begin(), facts.effects.end(),
                                  [&](const FactPair &effect) {return effect.var == precondition.var;});
            if (!changed)
                facts.postconditions.push_back(precondition);
        }
        sorted_unique(facts.postconditions);
    }
    return operators;
}

bool operator_applicable(const AbstractTask &task, const unordered_map<int, int> &partial_state,
                         const OperatorFacts &op, bool use_mutex_relations) {
    for (const FactPair &precondition : op.preconditions) {
        auto it = partial_state.find(precondition.var);
        if (it != partial_state.end() && it->second != precondition.value)
            return false;
    }
    if (use_mutex_relations) {
        // Mutexes.states_conflicts
        for (const auto &[var, value] : partial_state) {
            for (const FactPair &precondition : op.preconditions) {
                if (precondition.var != var && task.are_facts_mutex(FactPair(var, value), precondition))
                    return false;
            }
        }
    }
    return true;
}

int prune_inapplicable_transitions(Automaton &automaton, const AbstractTask &task,
                                   const vector<OperatorFacts> &operators) {
    remove_local_mapping(automaton);
    int num_actions = automaton.local_mapping.size();
    int num_removed = 0;
    for (int state = 0; state < automaton.num_states; ++state) {
        const FactPair &fact = automaton.state_facts.at(state).at(0);
        unordered_map<int, int> partial_state = {{fact.var, fact.value}};
        for (int action = 0; action < num_actions; ++action) {
            int &target = automaton.transitions[static_cast<size_t>(state) * num_actions + action];
            if (target == -1)
                continue;
            const FactPair &target_fact = automaton.state_facts.at(target).at(0);
            auto mutex_with_target = [&](const FactPair &postcondition) {
                return postcondition.var != target_fact.var && task.are_facts_mutex(target_fact, postcondition);
            };
            const vector<FactPair> &postconditions = operators[action].postconditions;
            if (!operator_applicable(task, partial_state, operators[action], true) ||
                any_of(postconditions.begin(), postconditions.end(), mutex_with_target)) {
                target = -1;
                ++num_removed;
            }
        }
    }
    reduce_local_equivalent_actions(automaton);
    return num_removed;
}

namespace {
// The automaton that forbids other actions after a no-op (the no-op is the last action).
Automaton no_op_right_pad_automaton(const vector<int> &costs) {
    // State 0: before the first no-op, state 1: after it (only no-ops).
    int num_actions = costs.size();
    Automaton no_op_automaton;
    no_op_automaton.num_states = 2;
    no_op_automaton.init_state = 0;
    no_op_automaton.goal_states = {0, 1};
    no_op_automaton.num_local_actions = num_actions;
    no_op_automaton.local_mapping.resize(num_actions);
    for (int action = 0; action < num_actions; ++action)
        no_op_automaton.local_mapping[action] = action;
    no_op_automaton.transitions.assign(static_cast<size_t>(2) * num_actions, -1);
    for (int action = 0; action < num_actions; ++action)
        no_op_automaton.transitions[action] = 0;
    no_op_automaton.transitions[num_actions - 1] = 1;
    no_op_automaton.transitions[static_cast<size_t>(num_actions) + num_actions - 1] = 1;
    no_op_automaton.has_costs = true;
    no_op_automaton.local_costs = costs;
    reduce_local_equivalent_actions(no_op_automaton);
    return no_op_automaton;
}

// Append a local action (cost 0) that is not applicable anywhere; returns its id.
int append_local_action(NDAutomaton &automaton) {
    int no_op = automaton.num_local_actions++;
    automaton.local_mapping.push_back(no_op);
    if (!automaton.has_costs) {
        automaton.has_costs = true;
        automaton.local_costs.assign(automaton.num_local_actions, 1);
        automaton.local_costs[no_op] = 0;
    } else {
        automaton.local_costs.push_back(0);
    }
    return no_op;
}
}

bool has_local_mapping(const NDAutomaton &automaton) {
    if (automaton.num_local_actions != static_cast<int>(automaton.local_mapping.size()))
        return true;
    for (size_t i = 0; i < automaton.local_mapping.size(); ++i) {
        if (automaton.local_mapping[i] != static_cast<int>(i))
            return true;
    }
    return false;
}

void remove_local_mapping(NDAutomaton &automaton) {
    if (!has_local_mapping(automaton))
        return;
    int num_actions = automaton.local_mapping.size();
    for (auto &state_transitions : automaton.transitions) {
        map<int, vector<int>> by_local(state_transitions.begin(), state_transitions.end());
        vector<pair<int, vector<int>>> new_state_transitions;
        for (int action = 0; action < num_actions; ++action) {
            auto it = by_local.find(automaton.local_mapping[action]);
            if (it != by_local.end())
                new_state_transitions.emplace_back(action, it->second);
        }
        state_transitions = move(new_state_transitions);
    }
    if (automaton.has_costs) {
        vector<int> new_costs(num_actions);
        for (int action = 0; action < num_actions; ++action)
            new_costs[action] = automaton.local_cost(automaton.local_mapping[action]);
        automaton.local_costs = move(new_costs);
    }
    for (int action = 0; action < num_actions; ++action)
        automaton.local_mapping[action] = action;
    automaton.num_local_actions = num_actions;
}

NDAutomaton to_nd_automaton(const Automaton &automaton) {
    NDAutomaton nd;
    static_cast<AutomatonBase &>(nd) = automaton;
    nd.transitions.resize(automaton.num_states);
    for (int state = 0; state < automaton.num_states; ++state) {
        for (int local = 0; local < automaton.num_local_actions; ++local) {
            int target = automaton.target(state, local);
            if (target != -1)
                nd.transitions[state].emplace_back(local, vector<int>{target});
        }
    }
    // The constructor of NonDeterministicAutomaton reduces the local actions when the automaton has a local mapping.
    if (has_local_mapping(automaton))
        reduce_local_equivalent_actions(nd);
    return nd;
}

/*
  The no-op is a new local action, the last one, so the transitions of each
  state stay sorted by local action.
*/
void add_no_op_action(NDAutomata &automata, int &action_cost_flag, bool right_pad_automaton) {
    automata.costs.push_back(0);
    for (NDAutomaton &automaton : automata.automata) {
        int no_op = append_local_action(automaton);
        for (int goal : automaton.goal_states)
            automaton.transitions[goal].emplace_back(no_op, vector<int>{goal});
    }
    if (action_cost_flag == 0)
        action_cost_flag = 1;
    if (right_pad_automaton)
        automata.automata.push_back(to_nd_automaton(no_op_right_pad_automaton(automata.costs)));
}

void add_no_op_action2(NDAutomata &automata, int &action_cost_flag) {
    automata.costs.push_back(0);
    for (NDAutomaton &automaton : automata.automata) {
        int no_op = append_local_action(automaton);
        int no_op_state = automaton.num_states++;
        automaton.transitions.emplace_back();
        automaton.transitions[no_op_state].emplace_back(no_op, vector<int>{no_op_state});
        for (int goal : automaton.goal_states)
            automaton.transitions[goal].emplace_back(no_op, vector<int>{no_op_state});
        automaton.goal_states.push_back(no_op_state);
    }
    if (action_cost_flag == 0)
        action_cost_flag = 1;
}

Automaton deterministic_copy(const NDAutomaton &nd) {
    Automaton automaton;
    static_cast<AutomatonBase &>(automaton) = nd;
    automaton.transitions.assign(static_cast<size_t>(nd.num_states) * nd.num_local_actions, -1);
    for (int state = 0; state < nd.num_states; ++state) {
        for (const auto &[local, successors] : nd.transitions[state]) {
            if (successors.size() != 1)
                throw runtime_error("deterministic_copy of a non-deterministic automaton");
            automaton.transitions[static_cast<size_t>(state) * nd.num_local_actions + local] = successors[0];
        }
    }
    return automaton;
}
}
