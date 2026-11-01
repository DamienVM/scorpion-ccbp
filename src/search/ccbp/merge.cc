#include "merge.h"

#include "automata.h"
#include "nd_automata.h"
#include "preprocess.h"

#include "../task_proxy.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

using namespace std;

namespace ccbp {
namespace {
template<typename T>
vector<T> sorted_union(const vector<T> &a, const vector<T> &b) {
    vector<T> result;
    set_union(a.begin(), a.end(), b.begin(), b.end(), back_inserter(result));
    return result;
}

// An atom of the task, "name(p1,p2)" without the sign, as in driver/sas/fact.py.
struct Atom {
    string key;
    string name;
    vector<string> parameters;
};

/*
  The atoms of the values of each variable, in the order of the values and
  without duplicates ("Atom p" and "NegatedAtom p" have the atom p, "<none of
  those>" has none).
*/
vector<vector<Atom>> variable_atoms(const AbstractTask &task) {
    auto strip = [](const string &s) {
        size_t begin = s.find_first_not_of(" \t\r\n\f\v");
        if (begin == string::npos)
            return string();
        return s.substr(begin, s.find_last_not_of(" \t\r\n\f\v") - begin + 1);
    };
    vector<vector<Atom>> atoms;
    for (VariableProxy variable : TaskProxy(task).get_variables()) {
        vector<Atom> &var_atoms = atoms.emplace_back();
        for (int value = 0; value < variable.get_domain_size(); ++value) {
            string atom_string = strip(variable.get_fact(value).get_name());
            if (atom_string.find("<none of those>") != string::npos)
                continue;
            if (atom_string.rfind("NegatedAtom ", 0) == 0)
                atom_string = atom_string.substr(string("NegatedAtom ").size());
            else if (atom_string.rfind("Atom ", 0) == 0)
                atom_string = atom_string.substr(string("Atom ").size());
            // Python: name, parameters = atom_string.split('('); parameters without ')' and spaces.
            Atom atom;
            size_t open = atom_string.find('(');
            atom.name = atom_string.substr(0, open);
            string parameters;
            if (open != string::npos) {
                size_t next_open = atom_string.find('(', open + 1);
                parameters = atom_string.substr(open + 1, next_open == string::npos ? string::npos : next_open - open - 1);
                if (!parameters.empty() && parameters.back() == ')')
                    parameters.pop_back();
                parameters.erase(remove(parameters.begin(), parameters.end(), ' '), parameters.end());
            }
            atom.key = atom.name + "(" + parameters + ")";
            for (size_t start = 0; !parameters.empty();) {
                size_t comma = parameters.find(',', start);
                atom.parameters.push_back(parameters.substr(start, comma == string::npos ? string::npos : comma - start));
                if (comma == string::npos)
                    break;
                start = comma + 1;
            }
            bool known = any_of(var_atoms.begin(), var_atoms.end(), [&](const Atom &a) {return a.key == atom.key;});
            if (!known)
                var_atoms.push_back(move(atom));
        }
    }
    return atoms;
}

// The atoms of the variables, as "name(p1,p2)" sorted by name.
vector<string> sorted_atom_strings(const vector<vector<Atom>> &atoms, const vector<int> &variables) {
    vector<string> names;
    for (int var : variables) {
        for (const Atom &atom : atoms[var])
            names.push_back(atom.key);
    }
    sort(names.begin(), names.end());
    names.erase(unique(names.begin(), names.end()), names.end());
    return names;
}

string atoms_string(const vector<vector<Atom>> &atoms, const vector<int> &variables, bool as_list) {
    vector<string> names = sorted_atom_strings(atoms, variables);
    string text = as_list ? "[" : "{";
    for (size_t i = 0; i < names.size(); ++i)
        text += (i > 0 ? ", " : "") + names[i];
    return text + (as_list ? "]" : "}");
}

// _compute_action_cost of synchronize_automatons: the cost of each global action, from the local costs.
vector<int> compute_action_cost(const vector<const Automaton *> &automata, int action_cost_flag) {
    size_t first = 0;
    while (first < automata.size() && !automata[first]->has_costs)
        ++first;
    if (first == automata.size())
        throw runtime_error("No automaton with action costs to merge");
    int num_actions = automata[first]->local_mapping.size();
    vector<int> costs(num_actions);
    for (int action = 0; action < num_actions; ++action)
        costs[action] = automata[first]->local_costs[automata[first]->local_mapping[action]];
    if (action_cost_flag == 1)
        return costs;
    for (size_t i = first + 1; i < automata.size(); ++i) {
        const Automaton &automaton = *automata[i];
        if (!automaton.has_costs)
            continue;
        for (int action = 0; action < num_actions; ++action) {
            int cost = automaton.local_costs[automaton.local_mapping[action]];
            if (action_cost_flag == 2)
                costs[action] += cost;
            else if (action_cost_flag == 3)
                costs[action] = max(costs[action], cost);
            else
                throw runtime_error("Global action cost flag not recognised: " + to_string(action_cost_flag));
        }
    }
    return costs;
}

void print_pruning(const Automaton &merged, int num_removed) {
    cout << "merged automaton: " << merged.num_states << " state(s), " << num_removed
         << " removed by pruning and shrinking" << endl;
}

void replace_by_merge(Automata &automata, const vector<size_t> &indices, Automaton merged) {
    for (size_t i = 0; i < automata.automata.size(); ++i) {
        if (find(indices.begin(), indices.end(), i) == indices.end() && automata.automata[i].variables == merged.variables)
            throw runtime_error("Automaton of the merged atoms already exists");
    }
    vector<Automaton> kept;
    for (size_t i = 0; i < automata.automata.size(); ++i) {
        if (find(indices.begin(), indices.end(), i) == indices.end())
            kept.push_back(move(automata.automata[i]));
    }
    kept.push_back(move(merged));
    automata.automata = move(kept);
}
}

bool FactsRelations::is_state_valid(const vector<FactPair> &state) const {
    for (size_t i = 0; i < state.size(); ++i) {
        for (size_t j = i + 1; j < state.size(); ++j) {
            if (task.are_facts_mutex(state[i], state[j]))
                return false;
        }
    }
    return true;
}

optional<Automaton> synchronize_automata(const vector<const Automaton *> &automata, int action_cost_flag,
                                         const FactsRelations &relations, int max_states) {
    if (automata.size() == 1)
        return *automata[0];
    int num_actions = automata[0]->local_mapping.size();
    for (const Automaton *automaton : automata) {
        if (static_cast<int>(automaton->local_mapping.size()) != num_actions)
            throw runtime_error("All Automatons should have the same number of global actions");
    }

    auto state_facts = [&](const vector<int> &state) {
        vector<FactPair> facts;
        for (size_t i = 0; i < automata.size(); ++i) {
            const vector<FactPair> &automaton_facts = automata[i]->state_facts.at(state[i]);
            facts.insert(facts.end(), automaton_facts.begin(), automaton_facts.end());
        }
        sort(facts.begin(), facts.end());
        facts.erase(unique(facts.begin(), facts.end()), facts.end());
        return facts;
    };

    // StateDict: id of each recorded state, -1 for a state recorded as not valid.
    map<vector<int>, int> states;
    deque<vector<int>> to_visit;
    int num_states = 0;
    Automaton merged;
    auto add_state = [&](const vector<int> &state) {
        auto [it, inserted] = states.try_emplace(state, num_states);
        if (inserted) {
            to_visit.push_back(state);
            ++num_states;
        }
        return it->second;
    };

    vector<int> initial_state;
    for (const Automaton *automaton : automata)
        initial_state.push_back(automaton->init_state);
    merged.state_facts.push_back(state_facts(initial_state));
    merged.init_state = add_state(initial_state);

    vector<int> transitions;
    while (!to_visit.empty()) {
        vector<int> state = to_visit.front();
        to_visit.pop_front();
        bool is_goal = true;
        for (size_t i = 0; i < automata.size(); ++i) {
            const vector<int> &goals = automata[i]->goal_states;
            if (find(goals.begin(), goals.end(), state[i]) == goals.end()) {
                is_goal = false;
                break;
            }
        }
        if (is_goal)
            merged.goal_states.push_back(states.at(state));

        for (int action = 0; action < num_actions; ++action) {
            vector<int> next(automata.size());
            bool applicable = true;
            for (size_t i = 0; i < automata.size(); ++i) {
                next[i] = automata[i]->target(state[i], automata[i]->local_mapping[action]);
                if (next[i] == -1) {
                    applicable = false;
                    break;
                }
            }
            if (!applicable) {
                transitions.push_back(-1);
                continue;
            }
            auto known = states.find(next);
            if (known != states.end() && known->second == -1) {
                transitions.push_back(-1);
                continue;
            }
            if (known == states.end()) {
                vector<FactPair> facts = state_facts(next);
                if (!relations.is_state_valid(facts)) {
                    states.emplace(next, -1);
                    transitions.push_back(-1);
                    continue;
                }
                if (max_states >= 0 && num_states >= max_states)
                    return nullopt; // one more state than max_states
                merged.state_facts.push_back(move(facts));
            }
            transitions.push_back(add_state(next));
        }
    }

    merged.num_states = num_states;
    merged.num_local_actions = num_actions;
    merged.local_mapping.resize(num_actions);
    for (int action = 0; action < num_actions; ++action)
        merged.local_mapping[action] = action;
    merged.transitions = move(transitions);
    if (action_cost_flag != 0) {
        merged.has_costs = true;
        merged.local_costs = compute_action_cost(automata, action_cost_flag);
    }
    for (const Automaton *automaton : automata)
        merged.variables = sorted_union(merged.variables, automaton->variables);
    reduce_local_equivalent_actions(merged);
    return merged;
}

int prune_and_shrink(Automaton &automaton, bool shrink) {
    NDAutomaton nd = to_nd_automaton(automaton);
    vector<int> state_mapping;
    if (!prune_unreachable_and_dead_end_states(nd, &state_mapping))
        throw runtime_error("The task is unsolvable: the initial state of an automaton is a dead end");
    if (shrink) {
        vector<int> shrink_mapping;
        shrink_bisimulation(nd, &shrink_mapping);
        for (int &state : state_mapping) {
            if (state != -1)
                state = shrink_mapping[state];
        }
    }
    int num_removed = automaton.num_states - nd.num_states;
    if (num_removed == 0)
        return 0;

    Automaton result = deterministic_copy(nd);
    result.variables = move(automaton.variables);
    if (!automaton.state_facts.empty()) {
        // The facts of a new state: the facts that hold in every old state mapped to it.
        result.state_facts.assign(result.num_states, {});
        vector<bool> seen(result.num_states, false);
        for (int state = 0; state < automaton.num_states; ++state) {
            int new_state = state_mapping[state];
            if (new_state == -1)
                continue;
            vector<FactPair> &facts = result.state_facts[new_state];
            if (!seen[new_state]) {
                facts = automaton.state_facts[state];
                seen[new_state] = true;
            } else {
                vector<FactPair> common;
                set_intersection(facts.begin(), facts.end(), automaton.state_facts[state].begin(),
                                 automaton.state_facts[state].end(), back_inserter(common));
                facts = move(common);
            }
        }
    }
    automaton = move(result);
    return num_removed;
}

void merge_min(Automata &automata, int percent, int action_cost_flag, const AbstractTask &task,
               const FactsRelations &relations, bool shrink, int max_states) {
    const vector<vector<Atom>> atoms = variable_atoms(task);
    int initial = automata.automata.size();
    int aimed = static_cast<int>(ceil(initial * (1 - percent / 100.0)));
    if (aimed == 0)
        aimed = 1;
    while (automata.automata.size() > 1 && static_cast<int>(automata.automata.size()) > aimed) {
        const vector<Automaton> &all = automata.automata;
        size_t first = 0;
        size_t second = 1;
        if (all[first].num_states > all[second].num_states)
            swap(first, second);
        for (size_t i = 2; i < all.size(); ++i) {
            if (all[i].num_states < all[second].num_states) {
                if (all[i].num_states < all[first].num_states) {
                    second = first;
                    first = i;
                } else {
                    second = i;
                }
            }
        }
        cout << "merging automaton " << atoms_string(atoms, all[first].variables, false) << " and "
             << atoms_string(atoms, all[second].variables, false) << endl;
        optional<Automaton> merged =
            synchronize_automata({&all[first], &all[second]}, action_cost_flag, relations, max_states);
        if (!merged) {
            cout << "merged automaton: more than " << max_states << " state(s), merging stopped" << endl;
            break;
        }
        print_pruning(*merged, prune_and_shrink(*merged, shrink));
        replace_by_merge(automata, {first, second}, move(*merged));
    }
}

void merge_objects(Automata &automata, int action_cost_flag, const AbstractTask &task, const FactsRelations &relations,
                   bool error_on_missing_automaton, bool shrink, int max_states) {
    const vector<vector<Atom>> atoms = variable_atoms(task);
    // Variables grouped by the one object all their real atoms share, objects in order of first occurrence.
    vector<string> objects;
    map<string, vector<int>> object_to_vars;
    for (int var = 0; var < static_cast<int>(atoms.size()); ++var) {
        map<string, int> counter;
        int num_real_atoms = 0;
        for (const Atom &atom : atoms[var]) {
            for (const string &parameter : atom.parameters)
                ++counter[parameter];
            if (atom.name != "void")
                ++num_real_atoms;
        }
        vector<string> common;
        for (const auto &[parameter, count] : counter) {
            if (count == num_real_atoms)
                common.push_back(parameter);
        }
        if (common.size() >= 2) {
            cout << "Multiple common parameters found: {";
            for (size_t i = 0; i < common.size(); ++i)
                cout << (i > 0 ? ", " : "") << '\'' << common[i] << '\'';
            cout << "}" << endl;
        } else if (common.size() == 1) {
            auto [it, inserted] = object_to_vars.try_emplace(common[0]);
            if (inserted)
                objects.push_back(common[0]);
            it->second.push_back(var);
        }
    }

    cout << "Object merges:" << endl;
    int count = 0;
    for (const string &object : objects) {
        const vector<int> &vars = object_to_vars[object];
        if (vars.size() < 2)
            continue;
        // The automaton holding each variable (the automata hold disjoint sets of variables).
        auto find_automaton = [&](int var) -> int {
            const vector<Automaton> &all = automata.automata;
            for (size_t i = 0; i < all.size(); ++i) {
                if (binary_search(all[i].variables.begin(), all[i].variables.end(), var))
                    return i;
            }
            return -1;
        };
        vector<size_t> to_merge;
        for (int var : vars) {
            if (atoms[var].empty())
                continue;
            int index = find_automaton(var);
            if (index == -1) {
                if (error_on_missing_automaton)
                    throw runtime_error("At least one of the requested automaton is not present: {" +
                                        atoms[var][0].key + "}.");
                continue;
            }
            if (find(to_merge.begin(), to_merge.end(), static_cast<size_t>(index)) == to_merge.end())
                to_merge.push_back(index);
        }
        // Python prints the atoms of each automaton sorted by name, automata in merge order.
        cout << object << ":\t[";
        bool first = true;
        for (size_t index : to_merge) {
            for (const string &name : sorted_atom_strings(atoms, automata.automata[index].variables)) {
                cout << (first ? "" : ", ") << name;
                first = false;
            }
        }
        cout << "]" << endl;
        if (to_merge.size() >= 2) {
            vector<const Automaton *> merged_automata;
            for (size_t index : to_merge)
                merged_automata.push_back(&automata.automata[index]);
            optional<Automaton> merged =
                synchronize_automata(merged_automata, action_cost_flag, relations, max_states);
            if (merged) {
                print_pruning(*merged, prune_and_shrink(*merged, shrink));
                replace_by_merge(automata, to_merge, move(*merged));
            } else {
                cout << "merged automaton: more than " << max_states << " state(s), merge skipped" << endl;
            }
        }
        ++count;
    }
    if (count == 0)
        cout << "none" << endl;
}
}
