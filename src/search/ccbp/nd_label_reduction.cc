#include "nd_label_reduction.h"

#include "nd_automata.h"
#include "partition.h"

#include <algorithm>
#include <stdexcept>

using namespace std;

namespace ccbp {
namespace {
/*
  Policy of reduce_all_automata for the non-deterministic label reduction: an
  automaton tells apart the actions of different equivalent action sets
  (refined set by set, as the former driver), and all the actions that the
  other automata cannot tell apart are merged in theta.
*/
struct NonDeterministicPolicy {
    using Automata = NDAutomata;
    using Snapshot = vector<vector<int>>; // blocks of the actions, in block id order

    // Theta takes the union of the transitions of a group, whatever their costs.
    bool separates_costs(const NDAutomaton &) const {
        return false;
    }

    void refine_by_automaton(Partition &partition, const NDAutomaton &automaton) const {
        for (const vector<int> &set : equivalent_action_sets(automaton))
            partition.refine(set);
    }

    Snapshot snapshot(const Partition &partition) const {
        return partition.blocks();
    }

    void refine_by_snapshot(Partition &partition, const Snapshot &snapshot) const {
        for (const vector<int> &set : snapshot)
            partition.refine(set);
    }

    void translate_snapshot(Snapshot &snapshot, const vector<int> &step_mapping, int) const {
        for (vector<int> &set : snapshot) {
            for (int &action : set)
                action = step_mapping[action];
            sort(set.begin(), set.end());
            set.erase(unique(set.begin(), set.end()), set.end());
        }
    }

    vector<vector<int>> groups(const NDAutomaton &, const Partition &combinable) const {
        return combinable.blocks();
    }

    void apply(NDAutomata &automata, int theta_index, const vector<vector<int>> &groups) const {
        apply_nd_reduction(automata, theta_index, groups);
    }
};
}

NDLabelReduction::NDLabelReduction(const string &strategy) {
    if (strategy == "lropc") {
        state.restrict_to_operator_counting = true;
    } else if (strategy != "greedy") {
        throw invalid_argument("Unknown non-deterministic label reduction strategy: " + strategy);
    }
}

void NDLabelReduction::set_operator_counting(OperatorCountingLP lp, vector<int> original_to_initial) {
    state.operator_counting_lp = move(lp);
    state.original_to_initial = move(original_to_initial);
}

bool NDLabelReduction::apply_all_automata(NDAutomata &automata) {
    return reduce_all_automata(automata, NonDeterministicPolicy{}, state);
}

int NDLabelReduction::apply_fix_point(NDAutomata &automata) {
    int num_passes = 0;
    while (apply_all_automata(automata))
        ++num_passes;
    return num_passes;
}
}
