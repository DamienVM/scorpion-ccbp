#include "label_reduction.h"

#include "automata.h"
#include "operator_counting.h"
#include "partition.h"

#include <numeric>
#include <stdexcept>
#include <unordered_map>

using namespace std;

namespace ccbp {
namespace {
struct ColumnHash {
    size_t operator()(const pair<vector<int>, int> &key) const {
        size_t hash = hash_combine(0, key.second);
        for (int value : key.first)
            hash = hash_combine(hash, value);
        return hash;
    }

    static size_t hash_combine(size_t seed, int value) {
        return seed ^ (std::hash<int>()(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
    }
};

/*
  Theta after reducing its actions: new action g takes, in every state, the
  transition of the first (smallest) of its old actions applicable there.
  Then locally equivalent actions (same transitions, same cost) are grouped,
  local ids given by first occurrence (Automaton._reduce_local_equivalent_actions).
*/
void reduce_theta(Automaton &theta, const vector<vector<int>> &groups) {
    int num_new_actions = groups.size();
    vector<int> new_local_mapping(num_new_actions);
    vector<int> new_transitions;
    vector<int> new_costs;
    unordered_map<pair<vector<int>, int>, int, ColumnHash> column_to_local;
    vector<vector<int>> local_columns;

    for (int g = 0; g < num_new_actions; ++g) {
        vector<int> column(theta.num_states, -1);
        for (int state = 0; state < theta.num_states; ++state) {
            for (int old_action : groups[g]) {
                int target = theta.target(state, theta.local_mapping[old_action]);
                if (target != -1) {
                    column[state] = target;
                    break;
                }
            }
        }
        int cost = theta.has_costs ? theta.local_costs[theta.local_mapping[groups[g][0]]] : 0;
        auto [it, inserted] = column_to_local.try_emplace(
            make_pair(column, cost), static_cast<int>(local_columns.size()));
        if (inserted) {
            local_columns.push_back(move(column));
            if (theta.has_costs)
                new_costs.push_back(cost);
        }
        new_local_mapping[g] = it->second;
    }

    int num_locals = local_columns.size();
    new_transitions.resize(static_cast<size_t>(theta.num_states) * num_locals);
    for (int local = 0; local < num_locals; ++local) {
        for (int state = 0; state < theta.num_states; ++state)
            new_transitions[static_cast<size_t>(state) * num_locals + local] = local_columns[local][state];
    }
    theta.num_local_actions = num_locals;
    theta.local_mapping = move(new_local_mapping);
    theta.transitions = move(new_transitions);
    theta.local_costs = move(new_costs);
}

/*
  Reduce the actions of all automata. The groups are the new actions (group g
  holds the old actions mapped to g, sorted increasingly). Outside theta, the
  actions of a group are locally equivalent, so only the mapping changes.
*/
void apply_reduction(Automata &automata, int theta_index, const vector<vector<int>> &groups) {
    int num_new_actions = groups.size();
    for (size_t i = 0; i < automata.automata.size(); ++i) {
        Automaton &automaton = automata.automata[i];
        if (static_cast<int>(i) == theta_index) {
            reduce_theta(automaton, groups);
        } else {
            vector<int> new_local_mapping(num_new_actions);
            for (int g = 0; g < num_new_actions; ++g)
                new_local_mapping[g] = automaton.local_mapping[groups[g][0]];
            automaton.local_mapping = move(new_local_mapping);
        }
    }

    vector<int> new_costs(num_new_actions);
    for (int g = 0; g < num_new_actions; ++g) {
        new_costs[g] = automata.costs[groups[g][0]];
        for (int old_action : groups[g]) {
            if (automata.costs[old_action] != new_costs[g])
                throw runtime_error("Should have the same cost for all actions of a reduced group");
        }
    }
    automata.costs = move(new_costs);
}
}

LabelReduction::LabelReduction(const string &strategy) {
    if (strategy == "greedy") {
        pass_rules = {ConflictRule::GREEDY};
    } else if (strategy == "sss") {
        pass_rules = {ConflictRule::SSS};
    } else if (strategy == "dss") {
        pass_rules = {ConflictRule::DSS};
    } else if (strategy == "sss_greedy") {
        pass_rules = {ConflictRule::SSS, ConflictRule::GREEDY};
    } else if (strategy == "lropc") {
        pass_rules = {ConflictRule::GREEDY};
        state.restrict_to_operator_counting = true;
    } else if (strategy == "lropc_sss_greedy") {
        pass_rules = {ConflictRule::SSS, ConflictRule::GREEDY};
        state.restrict_to_operator_counting = true;
    } else {
        throw invalid_argument("Unknown label reduction strategy: " + strategy);
    }
}

void LabelReduction::set_operator_counting(OperatorCountingLP lp, vector<int> original_to_initial) {
    state.operator_counting_lp = move(lp);
    state.original_to_initial = move(original_to_initial);
}

namespace {
/*
  Policy of reduce_all_automata for the deterministic label reduction: an
  automaton tells apart the actions with different local actions, and the
  actions it cannot tell apart are split into sets that can be merged
  deterministically in theta (conflict rule of the strategy).
*/
struct DeterministicPolicy {
    using Automata = ccbp::Automata;
    using Snapshot = vector<int>; // labelling of the actions

    ConflictRule rule;

    void refine_by_automaton(Partition &partition, const Automaton &automaton) const {
        partition.refine_by_labels(automaton.local_mapping);
    }

    // The conflict graph keeps actions of different local costs apart.
    bool separates_costs(const Automaton &theta) const {
        return theta.has_costs;
    }

    Snapshot snapshot(const Partition &partition) const {
        return partition.labelling();
    }

    void refine_by_snapshot(Partition &partition, const Snapshot &snapshot) const {
        partition.refine_by_labels(snapshot);
    }

    void translate_snapshot(Snapshot &snapshot, const vector<int> &step_mapping, int num_new_actions) const {
        // Actions merged by a reduction share a block in every snapshot, hence a label.
        vector<int> new_labels(num_new_actions, 0);
        for (size_t action = 0; action < snapshot.size(); ++action)
            new_labels[step_mapping[action]] = snapshot[action];
        snapshot = move(new_labels);
    }

    vector<vector<int>> groups(const Automaton &theta, const Partition &combinable) const {
        LocalTransitions local_transitions = compute_local_transitions(theta);
        vector<vector<int>> groups;
        for (const vector<int> &block : combinable.blocks()) {
            for (vector<int> &set : compute_deterministic_sets(theta, local_transitions, block, rule))
                groups.push_back(move(set));
        }
        return groups;
    }

    void apply(Automata &automata, int theta_index, const vector<vector<int>> &groups) const {
        apply_reduction(automata, theta_index, groups);
    }
};
}

bool LabelReduction::apply_all_automata(Automata &automata) {
    bool changed = false;
    for (ConflictRule rule : pass_rules) {
        if (reduce_all_automata(automata, DeterministicPolicy{rule}, state))
            changed = true;
    }
    return changed;
}

int LabelReduction::apply_fix_point(Automata &automata) {
    int num_passes = 0;
    while (apply_all_automata(automata))
        ++num_passes;
    return num_passes;
}
}
