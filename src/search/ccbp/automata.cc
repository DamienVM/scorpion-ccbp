#include "automata.h"

#include <algorithm>
#include <map>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>

using namespace std;

namespace ccbp {
static void expect_word(istream &in, const string &word) {
    string read;
    if (!(in >> read) || read != word) {
        throw runtime_error("Expected '" + word + "' in automata file but read '" + read + "'");
    }
}

static int read_int(istream &in) {
    int value;
    if (!(in >> value)) {
        throw runtime_error("Unexpected end of automata file");
    }
    return value;
}

static Automaton read_automaton(istream &in, int num_actions) {
    Automaton automaton;
    automaton.num_states = read_int(in);
    automaton.init_state = read_int(in);
    int num_goal_states = read_int(in);
    automaton.goal_states.resize(num_goal_states);
    for (int &goal : automaton.goal_states) {
        goal = read_int(in);
    }
    automaton.num_local_actions = read_int(in);
    automaton.local_mapping.resize(num_actions);
    for (int &local_action : automaton.local_mapping) {
        local_action = read_int(in);
    }
    automaton.transitions.resize(
        static_cast<size_t>(automaton.num_states) * automaton.num_local_actions);
    for (int &target : automaton.transitions) {
        target = read_int(in);
    }
    automaton.has_costs = read_int(in) == 1;
    if (automaton.has_costs) {
        automaton.local_costs.resize(automaton.num_local_actions);
        for (int &cost : automaton.local_costs) {
            cost = read_int(in);
        }
    }
    return automaton;
}

Automata read_automata(istream &in) {
    Automata automata;
    expect_word(in, "Actions:");
    int num_actions = read_int(in);
    expect_word(in, "NonUnitCost:");
    automata.non_unit_cost = read_int(in);
    expect_word(in, "Costs:");
    automata.costs.resize(num_actions);
    for (int &cost : automata.costs) {
        cost = read_int(in);
    }
    expect_word(in, "Automata:");
    int num_automata = read_int(in);
    automata.automata.reserve(num_automata);
    for (int i = 0; i < num_automata; ++i) {
        automata.automata.push_back(read_automaton(in, num_actions));
    }
    return automata;
}

Automata read_cp_automata(istream &in, int &action_cost_flag) {
    Automata automata;
    read_int(in); // plan length bounds
    read_int(in);
    int num_actions = read_int(in);
    action_cost_flag = read_int(in);
    if (action_cost_flag != 0) {
        read_int(in); // b and c
        read_int(in);
    }
    if (action_cost_flag < 0 || action_cost_flag > 3)
        throw runtime_error("Action cost flag not recognised");
    int num_automata = read_int(in);
    for (int i = 0; i < num_automata; ++i)
        automata.automata.push_back(read_automaton(in, num_actions));

    // Costs of the global actions from the local costs, as Automata.from_file.
    automata.costs.assign(num_actions, action_cost_flag == 0 ? 1 : 0);
    for (const Automaton &automaton : automata.automata) {
        if (action_cost_flag == 3) {
            for (int action = 0; action < num_actions; ++action)
                automata.costs[action] = max(automata.costs[action],
                                             automaton.local_costs.at(automaton.local_mapping[action]));
            continue;
        }
        if (action_cost_flag == 0 || !automaton.has_costs)
            continue;
        for (int action = 0; action < num_actions; ++action) {
            int cost = automaton.local_costs[automaton.local_mapping[action]];
            automata.costs[action] = action_cost_flag == 1 ? cost : automata.costs[action] + cost;
        }
        if (action_cost_flag == 1)
            break;
    }
    // The Automaton constructor reduces the local actions of the automata without a local mapping.
    for (Automaton &automaton : automata.automata)
        reduce_local_equivalent_actions(automaton);
    return automata;
}

static void write_line(const vector<int> &values, ostream &out) {
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            out << ' ';
        out << values[i];
    }
    out << '\n';
}

void write_automata(const Automata &automata, ostream &out) {
    bool unit_cost = all_of(automata.costs.begin(), automata.costs.end(),
                            [](int cost) {return cost == 1;});
    out << "Actions: " << automata.num_actions() << '\n';
    out << "NonUnitCost: " << (unit_cost ? 0 : 1) << '\n';
    out << "Costs: ";
    write_line(automata.costs, out);
    out << "Automata: " << automata.automata.size() << "\n\n";
    for (const Automaton &automaton : automata.automata) {
        out << automaton.num_states << '\n' << automaton.init_state << '\n';
        out << automaton.goal_states.size() << ' ';
        for (int goal : automaton.goal_states) {
            out << ' ' << goal;
        }
        out << '\n' << automaton.num_local_actions << '\n';
        write_line(automaton.local_mapping, out);
        for (int state = 0; state < automaton.num_states; ++state) {
            auto row = automaton.transitions.begin() + static_cast<size_t>(state) * automaton.num_local_actions;
            write_line(vector<int>(row, row + automaton.num_local_actions), out);
        }
        out << (automaton.has_costs ? 1 : 0) << '\n';
        if (automaton.has_costs) {
            write_line(automaton.local_costs, out);
        }
        out << '\n';
    }
}

bool has_local_mapping(const Automaton &automaton) {
    if (automaton.num_local_actions != static_cast<int>(automaton.local_mapping.size()))
        return true;
    for (size_t i = 0; i < automaton.local_mapping.size(); ++i) {
        if (automaton.local_mapping[i] != static_cast<int>(i))
            return true;
    }
    return false;
}

void remove_local_mapping(Automaton &automaton) {
    if (!has_local_mapping(automaton))
        return;
    int num_actions = automaton.local_mapping.size();
    vector<int> new_transitions(static_cast<size_t>(automaton.num_states) * num_actions);
    for (int state = 0; state < automaton.num_states; ++state) {
        for (int action = 0; action < num_actions; ++action) {
            new_transitions[static_cast<size_t>(state) * num_actions + action] =
                automaton.target(state, automaton.local_mapping[action]);
        }
    }
    if (automaton.has_costs) {
        vector<int> new_costs(num_actions);
        for (int action = 0; action < num_actions; ++action)
            new_costs[action] = automaton.local_costs[automaton.local_mapping[action]];
        automaton.local_costs = move(new_costs);
    }
    for (int action = 0; action < num_actions; ++action)
        automaton.local_mapping[action] = action;
    automaton.transitions = move(new_transitions);
    automaton.num_local_actions = num_actions;
}

void reduce_local_equivalent_actions(Automaton &automaton) {
    if (has_local_mapping(automaton))
        return;
    int num_actions = automaton.local_mapping.size();
    // ArrayIdentifierWithCost: (column of transitions, cost) -> id by first occurrence.
    map<pair<vector<int>, int>, int> column_ids;
    vector<vector<int>> columns;
    vector<int> new_costs;
    vector<int> new_local_mapping(num_actions);
    for (int action = 0; action < num_actions; ++action) {
        vector<int> column(automaton.num_states);
        for (int state = 0; state < automaton.num_states; ++state)
            column[state] = automaton.target(state, action);
        int cost = automaton.has_costs ? automaton.local_costs[action] : 0;
        auto [it, inserted] = column_ids.try_emplace(make_pair(column, cost), static_cast<int>(columns.size()));
        if (inserted) {
            columns.push_back(move(column));
            if (automaton.has_costs)
                new_costs.push_back(cost);
        }
        new_local_mapping[action] = it->second;
    }
    int num_locals = columns.size();
    vector<int> new_transitions(static_cast<size_t>(automaton.num_states) * num_locals);
    for (int local = 0; local < num_locals; ++local) {
        for (int state = 0; state < automaton.num_states; ++state)
            new_transitions[static_cast<size_t>(state) * num_locals + local] = columns[local][state];
    }
    automaton.num_local_actions = num_locals;
    automaton.local_mapping = move(new_local_mapping);
    automaton.transitions = move(new_transitions);
    if (automaton.has_costs)
        automaton.local_costs = move(new_costs);
}
}
