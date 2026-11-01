#ifndef CCBP_REDUCTION_LOOP_H
#define CCBP_REDUCTION_LOOP_H

#include "operator_counting.h"
#include "partition.h"

#include <numeric>
#include <vector>

namespace ccbp {
/*
  State of a label reduction kept across its passes: the composed mapping
  from the initial actions to the current actions, and the operator-counting
  restriction of the lropc strategies (OPCRestriction of the former driver):
  only actions in the same set of the operator-counting partition may be
  merged, so that the operator-counting constraints are preserved by the
  reduction. The LP stays over the original operators; original_to_initial
  maps them to the initial actions of the reduction (empty: the identity).
*/
struct ReductionState {
    std::vector<int> mapping; // initial action -> current action
    bool restrict_to_operator_counting = false;
    OperatorCountingLP operator_counting_lp;
    std::vector<int> original_to_initial;

    // The operator-counting partition over the current actions.
    Partition operator_counting_partition(int num_actions) const {
        std::vector<int> original_to_current;
        if (original_to_initial.empty()) {
            original_to_current = mapping;
        } else {
            original_to_current.reserve(original_to_initial.size());
            for (int initial : original_to_initial)
                original_to_current.push_back(mapping[initial]);
        }
        return build_operator_counting_partition(operator_counting_lp, original_to_current, num_actions);
    }
};

/*
  One pass of a label reduction over all automata, as
  LabelReduction.apply_all_automaton of the former driver: for each automaton
  theta (last to first), the actions that every other automaton cannot tell
  apart are grouped, and each group is reduced to a single action.

  The two label reductions differ only by the Policy:
  - Policy::Automata: the automata type;
  - separates_costs(theta): whether groups(theta, ...) never mixes actions of different costs;
  - Policy::Snapshot: a partition of the actions saved during the forward pass;
  - refine_by_automaton(partition, automaton): refine by the actions an automaton tells apart;
  - snapshot(partition), refine_by_snapshot(partition, snapshot),
    translate_snapshot(snapshot, step_mapping, num_new_actions);
  - groups(theta, block_partition): the groups to reduce in theta, in order;
  - apply(automata, theta_index, groups): reduce the actions of the groups.
*/
// True iff a block of the partition holds values with different labels.
inline bool mixes_values(const Partition &partition, const std::vector<int> &labels) {
    std::vector<int> block_label(partition.size(), 0);
    std::vector<bool> seen(partition.size(), false);
    const std::vector<int> &block_of = partition.labelling();
    for (size_t value = 0; value < block_of.size(); ++value) {
        int block = block_of[value];
        if (!seen[block]) {
            seen[block] = true;
            block_label[block] = labels[value];
        } else if (block_label[block] != labels[value]) {
            return true;
        }
    }
    return false;
}

template<typename Policy>
bool reduce_all_automata(typename Policy::Automata &automata, const Policy &policy, ReductionState &state) {
    using Snapshot = typename Policy::Snapshot;
    int num_automata = automata.automata.size();
    int num_initial_actions = automata.num_actions();
    if (state.mapping.empty()) {
        state.mapping.resize(num_initial_actions);
        std::iota(state.mapping.begin(), state.mapping.end(), 0);
    }

    // Operator-counting partition, rebuilt at the start of every pass and translated after each reduction.
    Partition operator_counting_groups(0);
    if (state.restrict_to_operator_counting)
        operator_counting_groups = state.operator_counting_partition(num_initial_actions);

    /*
      Forward pass: snapshots[indices[i]] is the partition of the actions that
      the automata before i cannot tell apart. Consecutive equal snapshots are
      stored once.
    */
    std::vector<Snapshot> snapshots;
    std::vector<int> indices;
    {
        Partition current(num_initial_actions);
        snapshots.push_back(policy.snapshot(current));
        for (int i = 0; i < num_automata; ++i) {
            indices.push_back(snapshots.size() - 1);
            if (current.size() == num_initial_actions) {
                indices.resize(num_automata, snapshots.size() - 1);
                break;
            }
            int previous_size = current.size();
            policy.refine_by_automaton(current, automata.automata[i]);
            if (current.size() != previous_size)
                snapshots.push_back(policy.snapshot(current));
        }
    }

    /*
      Backward pass: suffix is the partition for the automata after i, so
      suffix refined by the prefix snapshot of i covers all automata but i.
    */
    bool changed = false;
    Partition suffix(num_initial_actions);
    for (int i = num_automata - 1; i >= 0; --i) {
        int num_actions = automata.num_actions();
        if (suffix.size() == num_actions)
            break;

        Partition combinable = suffix;
        policy.refine_by_snapshot(combinable, snapshots[indices[i]]);
        /*
          A reduced label only groups actions of the same cost. The other
          automata usually tell actions of different costs apart already; when
          they do not (few automata left after merging, automata without
          costs) and the policy does not split them in theta, split by cost.
        */
        if (!policy.separates_costs(automata.automata[i]) && mixes_values(combinable, automata.costs))
            combinable.refine_by_labels(automata.costs);

        if (combinable.size() < num_actions) {
            if (state.restrict_to_operator_counting) {
                for (const std::vector<int> &group : operator_counting_groups.blocks()) {
                    if (!group.empty())
                        combinable.refine(group);
                }
            }
            std::vector<std::vector<int>> groups = policy.groups(automata.automata[i], combinable);

            if (static_cast<int>(groups.size()) < num_actions) {
                int num_new_actions = groups.size();
                std::vector<int> step_mapping(num_actions);
                for (int g = 0; g < num_new_actions; ++g) {
                    for (int action : groups[g])
                        step_mapping[action] = g;
                }
                policy.apply(automata, i, groups);
                changed = true;

                for (int &action : state.mapping)
                    action = step_mapping[action];
                // Only the snapshots of the automata before i are still needed.
                int num_needed = i > 0 ? indices[i - 1] + 1 : 0;
                for (int k = 0; k < num_needed; ++k)
                    policy.translate_snapshot(snapshots[k], step_mapping, num_new_actions);
                suffix.translate(step_mapping, num_new_actions);
                if (state.restrict_to_operator_counting)
                    operator_counting_groups.translate(step_mapping, num_new_actions);
            }
        }

        policy.refine_by_automaton(suffix, automata.automata[i]);
    }
    return changed;
}
}

#endif
