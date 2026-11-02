#include "nd_automata.h"

#include <algorithm>
#include <functional>
#include <istream>
#include <limits>
#include <map>
#include <ostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>

using namespace std;

namespace ccbp {
namespace {
void expect_word(istream &in, const string &word) {
    string read;
    if (!(in >> read) || read != word)
        throw runtime_error("Expected '" + word + "' in automata file but read '" + read + "'");
}

int read_int(istream &in) {
    int value;
    if (!(in >> value))
        throw runtime_error("Unexpected end of automata file");
    return value;
}

NDAutomaton read_nd_automaton(istream &in, int num_actions) {
    NDAutomaton automaton;
    automaton.num_states = read_int(in);
    automaton.init_state = read_int(in);
    automaton.goal_states.resize(read_int(in));
    for (int &goal : automaton.goal_states)
        goal = read_int(in);
    automaton.num_local_actions = read_int(in);
    automaton.local_mapping.resize(num_actions);
    for (int &local_action : automaton.local_mapping)
        local_action = read_int(in);
    automaton.transitions.resize(automaton.num_states);
    for (auto &state_transitions : automaton.transitions) {
        int num_entries = read_int(in);
        map<int, vector<int>> entries;
        for (int i = 0; i < num_entries; ++i) {
            int action = read_int(in);
            vector<int> &successors = entries[action];
            int num_successors = read_int(in);
            for (int j = 0; j < num_successors; ++j)
                successors.push_back(read_int(in));
        }
        for (auto &[action, successors] : entries) {
            sort(successors.begin(), successors.end());
            successors.erase(unique(successors.begin(), successors.end()), successors.end());
            if (!successors.empty())
                state_transitions.emplace_back(action, move(successors));
        }
    }
    automaton.has_costs = read_int(in) == 1;
    if (automaton.has_costs) {
        automaton.local_costs.resize(automaton.num_local_actions);
        for (int &cost : automaton.local_costs)
            cost = read_int(in);
    }
    return automaton;
}

void write_line(const vector<int> &values, ostream &out) {
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            out << ' ';
        out << values[i];
    }
    out << '\n';
}

struct VectorHash {
    size_t operator()(const vector<int> &values) const {
        size_t hash = values.size();
        for (int value : values)
            hash ^= std::hash<int>()(value) + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        return hash;
    }
};
}


NDAutomata read_nd_automata(istream &in) {
    NDAutomata automata;
    expect_word(in, "Actions:");
    int num_actions = read_int(in);
    expect_word(in, "NonUnitCost:");
    read_int(in);
    expect_word(in, "Costs:");
    automata.costs.resize(num_actions);
    for (int &cost : automata.costs)
        cost = read_int(in);
    expect_word(in, "Automata:");
    int num_automata = read_int(in);
    automata.automata.reserve(num_automata);
    for (int i = 0; i < num_automata; ++i)
        automata.automata.push_back(read_nd_automaton(in, num_actions));
    return automata;
}

void write_nd_automata(const NDAutomata &automata, ostream &out) {
    bool unit_cost = all_of(automata.costs.begin(), automata.costs.end(), [](int cost) {return cost == 1;});
    out << "Actions: " << automata.num_actions() << '\n';
    out << "NonUnitCost: " << (unit_cost ? 0 : 1) << '\n';
    out << "Costs: ";
    write_line(automata.costs, out);
    out << "Automata: " << automata.automata.size() << "\n\n";
    for (const NDAutomaton &automaton : automata.automata) {
        out << automaton.num_states << '\n' << automaton.init_state << '\n';
        out << automaton.goal_states.size() << ' ';
        for (int goal : automaton.goal_states)
            out << ' ' << goal;
        out << '\n' << automaton.num_local_actions << '\n';
        write_line(automaton.local_mapping, out);
        for (const auto &state_transitions : automaton.transitions) {
            out << state_transitions.size() << "  ";
            for (const auto &[action, successors] : state_transitions) {
                out << action << ' ' << successors.size() << ' ';
                for (size_t i = 0; i < successors.size(); ++i) {
                    if (i > 0)
                        out << ' ';
                    out << successors[i];
                }
                out << "  ";
            }
            out << '\n';
        }
        out << (automaton.has_costs ? 1 : 0) << '\n';
        if (automaton.has_costs)
            write_line(automaton.local_costs, out);
        out << '\n';
    }
}

vector<vector<int>> equivalent_action_sets(const NDAutomaton &automaton) {
    // The automaton is reduced: one set per local action, in the order of their first global action.
    unordered_map<int, int> local_to_set;
    vector<vector<int>> sets;
    for (int action = 0; action < static_cast<int>(automaton.local_mapping.size()); ++action) {
        auto [it, inserted] = local_to_set.try_emplace(automaton.local_mapping[action], static_cast<int>(sets.size()));
        if (inserted)
            sets.emplace_back();
        sets[it->second].push_back(action);
    }
    return sets;
}

void reduce_local_equivalent_actions(NDAutomaton &automaton) {
    vector<bool> live(max(automaton.num_local_actions, 0), false);
    int max_local = -1;
    for (int local_action : automaton.local_mapping)
        max_local = max(max_local, local_action);
    if (max_local >= static_cast<int>(live.size()))
        live.resize(max_local + 1, false);
    for (int local_action : automaton.local_mapping) {
        if (local_action != -1)
            live[local_action] = true;
    }

    // Signature of a live local action: (state, successors) for each state where it has successors, then its cost.
    vector<vector<int>> signatures(live.size());
    for (int state = 0; state < automaton.num_states; ++state) {
        for (const auto &[local_action, successors] : automaton.transitions[state]) {
            if (local_action < static_cast<int>(live.size()) && live[local_action]) {
                vector<int> &signature = signatures[local_action];
                signature.push_back(state);
                signature.push_back(successors.size());
                signature.insert(signature.end(), successors.begin(), successors.end());
            }
        }
    }

    vector<int> old_to_new(live.size(), -1);
    unordered_map<vector<int>, int, VectorHash> key_to_new;
    vector<int> new_costs;
    for (int local_action = 0; local_action < static_cast<int>(live.size()); ++local_action) {
        if (!live[local_action])
            continue;
        int cost = automaton.has_costs ? automaton.local_costs[local_action] : 0;
        vector<int> key = signatures[local_action];
        key.push_back(-1); // separator: a signature ends with a state, never with -1
        key.push_back(cost);
        auto [it, inserted] = key_to_new.try_emplace(move(key), static_cast<int>(key_to_new.size()));
        if (inserted && automaton.has_costs)
            new_costs.push_back(cost);
        old_to_new[local_action] = it->second;
    }

    for (auto &state_transitions : automaton.transitions) {
        vector<pair<int, vector<int>>> new_state_transitions;
        for (auto &[local_action, successors] : state_transitions) {
            if (local_action < static_cast<int>(old_to_new.size()) && old_to_new[local_action] != -1)
                new_state_transitions.emplace_back(old_to_new[local_action], move(successors));
        }
        sort(new_state_transitions.begin(), new_state_transitions.end());
        // Merged local actions have the same successors: keep one entry per new local action.
        new_state_transitions.erase(
            unique(new_state_transitions.begin(), new_state_transitions.end(),
                   [](const auto &a, const auto &b) {return a.first == b.first;}),
            new_state_transitions.end());
        state_transitions = move(new_state_transitions);
    }
    for (int &local_action : automaton.local_mapping) {
        if (local_action != -1)
            local_action = old_to_new[local_action];
    }
    automaton.num_local_actions = key_to_new.size();
    if (automaton.has_costs)
        automaton.local_costs = move(new_costs);
}

void apply_nd_reduction(NDAutomata &automata, int theta_index, const vector<vector<int>> &groups) {
    int num_new_actions = groups.size();
    for (size_t i = 0; i < automata.automata.size(); ++i) {
        NDAutomaton &automaton = automata.automata[i];
        if (static_cast<int>(i) == theta_index) {
            // The new actions of each old local action (the groups holding one of its global actions), increasing.
            vector<vector<int>> local_to_groups(automaton.num_local_actions);
            for (int g = 0; g < num_new_actions; ++g) {
                for (int old_action : groups[g]) {
                    int local_action = automaton.local_mapping[old_action];
                    if (local_action == -1)
                        continue;
                    vector<int> &local_groups = local_to_groups[local_action];
                    if (local_groups.empty() || local_groups.back() != g)
                        local_groups.push_back(g);
                }
            }
            /*
              In each state, the successors of a new action are the union of
              those of its old local actions: each transition of the state is
              added to the new actions of its local action.
            */
            vector<vector<pair<int, vector<int>>>> new_transitions(automaton.num_states);
            vector<vector<int>> group_successors(num_new_actions);
            vector<int> touched;
            for (int state = 0; state < automaton.num_states; ++state) {
                for (const auto &[local_action, successors] : automaton.transitions[state]) {
                    for (int g : local_to_groups[local_action]) {
                        if (group_successors[g].empty())
                            touched.push_back(g);
                        group_successors[g].insert(group_successors[g].end(), successors.begin(), successors.end());
                    }
                }
                sort(touched.begin(), touched.end());
                for (int g : touched) {
                    vector<int> &successors = group_successors[g];
                    sort(successors.begin(), successors.end());
                    successors.erase(unique(successors.begin(), successors.end()), successors.end());
                    new_transitions[state].emplace_back(g, move(successors));
                    successors.clear();
                }
                touched.clear();
            }
            if (automaton.has_costs) {
                vector<int> new_costs(num_new_actions);
                for (int g = 0; g < num_new_actions; ++g)
                    new_costs[g] = automaton.local_cost(automaton.local_mapping[groups[g][0]]);
                automaton.local_costs = move(new_costs);
            }
            automaton.local_mapping.resize(num_new_actions);
            for (int g = 0; g < num_new_actions; ++g)
                automaton.local_mapping[g] = g;
            automaton.transitions = move(new_transitions);
            automaton.num_local_actions = num_new_actions;
            reduce_local_equivalent_actions(automaton);
        } else {
            /*
              The old actions of a group cannot be told apart here: in a
              reduced automaton, they share their local action, so the
              automaton stays reduced and only its mapping changes. Otherwise,
              reduce it again.
            */
            vector<int> new_local_mapping(num_new_actions);
            bool same_local_actions = true;
            for (int g = 0; g < num_new_actions; ++g) {
                new_local_mapping[g] = automaton.local_mapping[groups[g][0]];
                for (int old_action : groups[g]) {
                    if (automaton.local_mapping[old_action] != new_local_mapping[g])
                        same_local_actions = false;
                }
            }
            automaton.local_mapping = move(new_local_mapping);
            if (!same_local_actions)
                reduce_local_equivalent_actions(automaton);
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

namespace {
/*
  Remove the states that are not reachable from the initial state or from
  which no goal state is reachable; the kept states keep their order. Returns
  false (and changes nothing) if the initial state would be removed.
*/
bool prune_states(NDAutomaton &automaton, vector<int> *state_mapping) {
    int num_states = automaton.num_states;
    vector<vector<int>> predecessors(num_states);
    for (int state = 0; state < num_states; ++state) {
        for (const auto &[action, successors] : automaton.transitions[state]) {
            for (int successor : successors)
                predecessors[successor].push_back(state);
        }
    }

    vector<bool> reachable(num_states, false);
    vector<int> stack = {automaton.init_state};
    reachable[automaton.init_state] = true;
    while (!stack.empty()) {
        int state = stack.back();
        stack.pop_back();
        for (const auto &[action, successors] : automaton.transitions[state]) {
            for (int successor : successors) {
                if (!reachable[successor]) {
                    reachable[successor] = true;
                    stack.push_back(successor);
                }
            }
        }
    }

    vector<bool> alive(num_states, false);
    stack = automaton.goal_states;
    for (int goal : stack)
        alive[goal] = true;
    while (!stack.empty()) {
        int state = stack.back();
        stack.pop_back();
        for (int predecessor : predecessors[state]) {
            if (!alive[predecessor]) {
                alive[predecessor] = true;
                stack.push_back(predecessor);
            }
        }
    }

    if (!reachable[automaton.init_state] || !alive[automaton.init_state])
        return false;
    vector<int> old_to_new(num_states, -1);
    int num_kept = 0;
    for (int state = 0; state < num_states; ++state) {
        if (reachable[state] && alive[state])
            old_to_new[state] = num_kept++;
    }
    if (state_mapping)
        *state_mapping = old_to_new;
    if (num_kept == num_states)
        return true;

    vector<vector<pair<int, vector<int>>>> new_transitions(num_kept);
    for (int state = 0; state < num_states; ++state) {
        if (old_to_new[state] == -1)
            continue;
        for (const auto &[action, successors] : automaton.transitions[state]) {
            vector<int> new_successors;
            for (int successor : successors) {
                if (old_to_new[successor] != -1)
                    new_successors.push_back(old_to_new[successor]);
            }
            if (!new_successors.empty())
                new_transitions[old_to_new[state]].emplace_back(action, move(new_successors));
        }
    }
    vector<int> new_goal_states;
    for (int goal : automaton.goal_states) {
        if (old_to_new[goal] != -1)
            new_goal_states.push_back(old_to_new[goal]);
    }
    sort(new_goal_states.begin(), new_goal_states.end());
    automaton.transitions = move(new_transitions);
    automaton.num_states = num_kept;
    automaton.init_state = old_to_new[automaton.init_state];
    automaton.goal_states = move(new_goal_states);
    return true;
}

// Cost of the cheapest path to a goal state (Dijkstra), unit cost without action costs.
vector<long long> goal_distances(const NDAutomaton &automaton) {
    vector<vector<pair<int, int>>> predecessors(automaton.num_states);
    for (int state = 0; state < automaton.num_states; ++state) {
        for (const auto &[action, successors] : automaton.transitions[state]) {
            int cost = automaton.has_costs ? automaton.local_costs[action] : 1;
            for (int successor : successors)
                predecessors[successor].emplace_back(state, cost);
        }
    }
    const long long unknown = numeric_limits<long long>::max();
    vector<long long> distances(automaton.num_states, unknown);
    priority_queue<pair<long long, int>, vector<pair<long long, int>>, greater<>> queue;
    for (int goal : automaton.goal_states)
        queue.emplace(0, goal);
    while (!queue.empty()) {
        auto [distance, state] = queue.top();
        queue.pop();
        if (distances[state] != unknown)
            continue;
        distances[state] = distance;
        for (auto [predecessor, cost] : predecessors[state]) {
            if (distances[predecessor] == unknown)
                queue.emplace(distance + cost, predecessor);
        }
    }
    return distances;
}

struct Signature {
    long long h_and_goal; // -1 for goal states, the goal distance otherwise
    int group;
    vector<pair<int, int>> successors; // sorted unique (local action, successor group)
    int state;

    bool operator<(const Signature &other) const {
        return tie(h_and_goal, group, successors, state) <
               tie(other.h_and_goal, other.group, other.successors, other.state);
    }
};
}

bool prune_unreachable_and_dead_end_states(NDAutomaton &automaton, vector<int> *state_mapping) {
    return prune_states(automaton, state_mapping);
}

int shrink_bisimulation(NDAutomaton &automaton, vector<int> *state_mapping) {
    int num_original_states = automaton.num_states;
    if (!prune_states(automaton, state_mapping))
        return 0;
    int num_states = automaton.num_states;
    vector<bool> is_goal(num_states, false);
    for (int goal : automaton.goal_states)
        is_goal[goal] = true;
    vector<long long> distances = goal_distances(automaton);

    // Group 0 holds the goal states, then one group per goal distance (initialize_groups of Scorpion).
    vector<int> state_to_group(num_states, 0);
    unordered_map<long long, int> h_to_group;
    int num_groups = 1;
    for (int state = 0; state < num_states; ++state) {
        if (!is_goal[state]) {
            auto [it, inserted] = h_to_group.try_emplace(distances[state], num_groups);
            if (inserted)
                ++num_groups;
            state_to_group[state] = it->second;
        }
    }

    bool stable = false;
    vector<Signature> signatures(num_states);
    while (!stable) {
        stable = true;
        for (int state = 0; state < num_states; ++state) {
            Signature &signature = signatures[state];
            signature.h_and_goal = is_goal[state] ? -1 : distances[state];
            signature.group = state_to_group[state];
            signature.state = state;
            signature.successors.clear();
            for (const auto &[action, successors] : automaton.transitions[state]) {
                for (int successor : successors)
                    signature.successors.emplace_back(action, state_to_group[successor]);
            }
            sort(signature.successors.begin(), signature.successors.end());
            signature.successors.erase(unique(signature.successors.begin(), signature.successors.end()),
                                       signature.successors.end());
        }
        sort(signatures.begin(), signatures.end());

        for (int start = 0; start < num_states;) {
            int end = start;
            while (end < num_states && signatures[end].h_and_goal == signatures[start].h_and_goal)
                ++end;
            // The signature before a block has another group (as the sentinel of Scorpion).
            int num_old_groups = 0;
            int num_new_groups = 0;
            for (int i = start; i < end; ++i) {
                if (i == start || signatures[i - 1].group != signatures[i].group) {
                    ++num_old_groups;
                    ++num_new_groups;
                } else if (signatures[i - 1].successors != signatures[i].successors) {
                    ++num_new_groups;
                }
            }
            if (num_new_groups != num_old_groups) {
                stable = false;
                int new_group = -1;
                for (int i = start; i < end; ++i) {
                    if (i == start || signatures[i - 1].group != signatures[i].group)
                        new_group = signatures[i].group;
                    else if (signatures[i - 1].successors != signatures[i].successors)
                        new_group = num_groups++;
                    state_to_group[signatures[i].state] = new_group;
                }
            }
            start = end;
        }
    }

    if (num_groups < num_states) {
        vector<map<int, vector<int>>> grouped(num_groups);
        for (int state = 0; state < num_states; ++state) {
            for (const auto &[action, successors] : automaton.transitions[state]) {
                vector<int> &new_successors = grouped[state_to_group[state]][action];
                for (int successor : successors)
                    new_successors.push_back(state_to_group[successor]);
            }
        }
        vector<vector<pair<int, vector<int>>>> new_transitions(num_groups);
        for (int group = 0; group < num_groups; ++group) {
            for (auto &[action, successors] : grouped[group]) {
                sort(successors.begin(), successors.end());
                successors.erase(unique(successors.begin(), successors.end()), successors.end());
                new_transitions[group].emplace_back(action, move(successors));
            }
        }
        vector<int> new_goal_states;
        for (int goal : automaton.goal_states)
            new_goal_states.push_back(state_to_group[goal]);
        sort(new_goal_states.begin(), new_goal_states.end());
        new_goal_states.erase(unique(new_goal_states.begin(), new_goal_states.end()), new_goal_states.end());
        automaton.transitions = move(new_transitions);
        automaton.num_states = num_groups;
        automaton.init_state = state_to_group[automaton.init_state];
        automaton.goal_states = move(new_goal_states);
        if (state_mapping) {
            for (int &state : *state_mapping) {
                if (state != -1)
                    state = state_to_group[state];
            }
        }
    }
    return num_original_states - automaton.num_states;
}
}
