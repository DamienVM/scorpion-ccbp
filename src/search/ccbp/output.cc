#include "output.h"

#include "nd_automata.h"

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <iostream>

using namespace std;

namespace ccbp {
namespace {
const string PREFIX = "[CCBP] ";
const string SEPARATOR = "───────────────────────────────────────────────────────";

string strip(const string &s) {
    size_t begin = s.find_first_not_of(" \t\r\n\f\v");
    if (begin == string::npos)
        return "";
    size_t end = s.find_last_not_of(" \t\r\n\f\v");
    return s.substr(begin, end - begin + 1);
}

void write_ints(const vector<int> &values, ostream &out) {
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0)
            out << ' ';
        out << values[i];
    }
}

// The lines of a CP automaton before and after its transitions (driver/automata/cp_automata_format.txt).
void write_automaton_head(const AutomatonBase &automaton, ostream &out) {
    out << automaton.num_states << '\n' << automaton.init_state << '\n';
    out << automaton.goal_states.size();
    for (int goal : automaton.goal_states)
        out << ' ' << goal;
    out << '\n' << automaton.num_local_actions << '\n';
    write_ints(automaton.local_mapping, out);
    out << '\n';
}

void write_automaton_costs(const AutomatonBase &automaton, ostream &out) {
    out << (automaton.has_costs ? 1 : 0) << '\n';
    if (automaton.has_costs) {
        write_ints(automaton.local_costs, out);
        out << '\n';
    }
}

/*
  One line per state: its number of transitions, then each transition. A local
  action with one successor is written "action successor", otherwise
  "-(action + 1) number_of_successors successors". Successors are written
  relative to the state (successor - state): the states of merged automata are
  numbered breadth-first, so successors are often close to their state.
*/
void write_automaton_body(const NDAutomaton &automaton, ostream &out) {
    write_automaton_head(automaton, out);
    for (int state = 0; state < automaton.num_states; ++state) {
        const auto &state_transitions = automaton.transitions[state];
        out << state_transitions.size();
        for (const auto &[action, successors] : state_transitions) {
            if (successors.size() == 1) {
                out << ' ' << action << ' ' << successors[0] - state;
            } else {
                out << ' ' << -(action + 1) << ' ' << successors.size();
                for (int successor : successors)
                    out << ' ' << successor - state;
            }
        }
        out << '\n';
    }
    write_automaton_costs(automaton, out);
}

void write_cp_header(int num_actions, int num_automata, const vector<int> &costs,
                     const CPParameters &parameters, ostream &out) {
    out << "cp_automata 2\n" << num_actions << '\n';
    if (parameters.action_cost_flag == 0)
        out << 0 << '\n';
    else
        out << parameters.action_cost_flag << ' ' << parameters.b << ' ' << parameters.c << '\n';
    out << num_automata << '\n';
    if (parameters.action_cost_flag == 4) {
        write_ints(costs, out);
        out << '\n';
    }
}
}

// repr of a Python float: the shortest digits that read back the same value, in fixed notation for
// exponents in [-4, 16) and in scientific notation ("6e-05") otherwise.
string python_float_repr(double seconds) {
    char buffer[64];
    auto result = to_chars(buffer, buffer + sizeof(buffer), seconds, chars_format::scientific);
    string scientific(buffer, result.ptr);
    size_t e = scientific.find('e');
    string mantissa = scientific.substr(0, e);
    int exponent = stoi(scientific.substr(e + 1));
    bool negative = !mantissa.empty() && mantissa[0] == '-';
    if (negative)
        mantissa.erase(0, 1);
    string digits;
    for (char c : mantissa) {
        if (c != '.')
            digits += c;
    }
    string text;
    if (exponent >= -4 && exponent < 16) {
        if (exponent >= 0) {
            string integer = digits.substr(0, min<size_t>(digits.size(), exponent + 1));
            integer.append(exponent + 1 - integer.size(), '0');
            string fraction = digits.size() > static_cast<size_t>(exponent + 1) ? digits.substr(exponent + 1) : "0";
            text = integer + "." + fraction;
        } else {
            text = "0." + string(-exponent - 1, '0') + digits;
        }
    } else {
        string exponent_text = to_string(abs(exponent));
        if (exponent_text.size() < 2)
            exponent_text = "0" + exponent_text;
        text = mantissa + "e" + (exponent < 0 ? "-" : "+") + exponent_text;
    }
    return (negative ? "-" : "") + text;
}

void print_info_line(const string &line) {
    cout << PREFIX << strip(line) << '\n';
}

string solution_string(const string &operator_name) {
    // Python: the words of the name, the first one is the name, the others the parameters.
    size_t begin = operator_name.find_first_not_of(" \t\r\n\f\v");
    size_t end = operator_name.find_last_not_of(" \t\r\n\f\v");
    string line = begin == string::npos ? "" : operator_name.substr(begin, end - begin + 1);
    return "(" + line + (line.find(' ') == string::npos ? " " : "") + ")";
}

void print_info_block(const vector<string> &lines, bool before_separator, bool after_separator) {
    if (before_separator)
        cout << PREFIX << "┌" << SEPARATOR << '\n';
    for (const string &line : lines)
        print_info_line(line);
    if (after_separator)
        cout << PREFIX << "└" << SEPARATOR << '\n';
    cout.flush();
}

ActionReductions::ActionReductions(int num_operators)
    : groups(num_operators), original_to_group(num_operators) {
    for (int op = 0; op < num_operators; ++op) {
        groups[op].push_back({op, op});
        original_to_group[op] = op;
    }
}

void ActionReductions::apply_mapping(const vector<int> &mapping, int num_new_groups) {
    vector<vector<Action>> new_groups(num_new_groups);
    for (size_t old_group = 0; old_group < groups.size(); ++old_group) {
        vector<Action> &group = new_groups[mapping[old_group]];
        group.insert(group.end(), groups[old_group].begin(), groups[old_group].end());
    }
    groups = move(new_groups);
    for (int &group : original_to_group)
        group = mapping[group];
}

void ActionReductions::remove_equivalent_actions(const vector<vector<int>> &equivalent_groups,
                                                 const vector<int> &representatives) {
    vector<int> old_to_new(groups.size(), -1);
    for (size_t new_index = 0; new_index < equivalent_groups.size(); ++new_index) {
        for (int old_action : equivalent_groups[new_index])
            old_to_new[old_action] = new_index;
    }
    for (int &group : original_to_group)
        group = old_to_new[group];

    vector<vector<Action>> new_groups;
    for (int representative : representatives)
        new_groups.push_back(groups[representative]);
    for (size_t index = 0; index < new_groups.size(); ++index) {
        for (Action &action : new_groups[index])
            action.id = index;
    }
    groups = move(new_groups);
}

void ActionReductions::add_no_op() {
    groups.push_back({{-1, -1}});
}

void ActionReductions::write_yaml(const vector<string> &solution_strings, ostream &out) const {
    for (size_t group_id = 0; group_id < groups.size(); ++group_id) {
        out << group_id << ":\n";
        vector<Action> actions = groups[group_id];
        sort(actions.begin(), actions.end(), [](const Action &a, const Action &b) {
            return a.id != b.id ? a.id < b.id : a.op < b.op;
        });
        for (const Action &action : actions) {
            string name = action.op == -1 ? "(<no_op> )" : solution_strings[action.op];
            out << "  " << action.id << ": \"" << name << "\"\n";
        }
    }
}

void write_cp_automata(const NDAutomata &automata, const CPParameters &parameters, ostream &out) {
    write_cp_header(automata.num_actions(), automata.automata.size(), automata.costs, parameters, out);
    for (const NDAutomaton &automaton : automata.automata) {
        out << '\n';
        write_automaton_body(automaton, out);
    }
}

void print_automata_info(const vector<int> &num_states, const vector<int> &num_local_actions, int num_actions) {
    cout << "Automata to be solved: global actions: " << num_actions << ", automatons: " << num_states.size() << '\n';
    for (size_t i = 0; i < num_states.size(); ++i) {
        cout << "Automaton " << i << " #states:" << num_states[i] << ",\t#local_actions:" << num_local_actions[i]
             << '\n';
    }
}
}
