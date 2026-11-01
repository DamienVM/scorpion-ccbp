#ifndef CCBP_OUTPUT_H
#define CCBP_OUTPUT_H

#include <iosfwd>
#include <string>
#include <vector>

namespace ccbp {
struct NDAutomaton;
struct NDAutomata;

// repr of a Python float ("6e-05", "0.25", "3.0").
std::string python_float_repr(double value);

// print_info_line / print_info_block of driver/util/ccbp_io.py.
void print_info_line(const std::string &line);

// Operator.to_solution_string of the former driver: "(name p1 p2)" from the operator name "name p1 p2".
std::string solution_string(const std::string &operator_name);
void print_info_block(const std::vector<std::string> &lines, bool before_separator = true,
                      bool after_separator = true);

/*
  ActionReductions of the former driver: the original operators behind each label.
  An operator keeps an id, which is the label it got when the equivalent
  actions were removed (its original id otherwise), as written in
  reductions.yaml.
*/
class ActionReductions {
    struct Action {
        int id;
        int op; // original operator, -1 for the no-op
    };
    std::vector<std::vector<Action>> groups;
    std::vector<int> original_to_group;

public:
    explicit ActionReductions(int num_operators);

    int num_groups() const {return static_cast<int>(groups.size());}
    const std::vector<int> &original_action_to_label() const {return original_to_group;}

    // Actions mapped together form one group (mapping: label -> new label).
    void apply_mapping(const std::vector<int> &mapping, int num_new_groups);

    // ActionReductions.remove_equivalent_actions_set.
    void remove_equivalent_actions(const std::vector<std::vector<int>> &equivalent_groups,
                                   const std::vector<int> &representatives);

    void add_no_op();

    // reductions.yaml (the actions of a group by increasing id).
    // solution_strings[o]: the solution string of the original operator o.
    void write_yaml(const std::vector<std::string> &solution_strings, std::ostream &out) const;
};

/*
  The CP automata file, version 2 (driver/automata/cp_automata_format.txt):
  number of actions, cost flag with b and c, then the automata. The plan
  length bounds are in the constraints file.
*/
struct CPParameters {
    int action_cost_flag = 0;
    long long b = 0;
    long long c = 0;
};
void write_cp_automata(const NDAutomata &automata, const CPParameters &parameters, std::ostream &out);

// Automata.print_info.
void print_automata_info(const std::vector<int> &num_states, const std::vector<int> &num_local_actions,
                         int num_actions);
}

#endif
