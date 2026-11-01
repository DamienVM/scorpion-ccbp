#include "pipeline.h"

#include "automata.h"
#include "label_reduction.h"
#include "merge.h"
#include "nd_automata.h"
#include "nd_label_reduction.h"
#include "operator_counting.h"
#include "output.h"
#include "partition.h"
#include "preprocess.h"
#include "remap.h"

#include "../task_proxy.h"

#include <algorithm>
#include <charconv>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

using namespace std;

namespace ccbp {
namespace {
double cpu_seconds_since(clock_t start) {
    return static_cast<double>(clock() - start) / CLOCKS_PER_SEC;
}


template<typename T>
void write_file(const string &path, const T &write) {
    ofstream out(path);
    if (!out)
        throw runtime_error("Cannot open " + path);
    write(out);
}

bool contains(const vector<string> &values, const string &value) {
    return find(values.begin(), values.end(), value) != values.end();
}

// SasProblem.make_partial_state (variable -> value): an error for two different values of a variable.
unordered_map<int, int> make_partial_state(const vector<FactPair> &facts) {
    unordered_map<int, int> partial_state;
    for (const FactPair &fact : facts) {
        auto [it, inserted] = partial_state.try_emplace(fact.var, fact.value);
        if (!inserted && it->second != fact.value)
            throw runtime_error("Contradictory fact can not be added to partial state");
    }
    return partial_state;
}

// The pairs of operators whose effects are the preconditions of each other (SasProblem.get_opposite_operators).
vector<pair<int, int>> opposite_operators(const vector<OperatorFacts> &operators) {
    vector<pair<int, int>> pairs;
    set<int> unique_actions;
    int num_operators = operators.size();
    auto subset = [](const vector<FactPair> &a, const vector<FactPair> &b) {
        return includes(b.begin(), b.end(), a.begin(), a.end());
    };
    for (int i = 0; i < num_operators; ++i) {
        for (int j = i + 1; j < num_operators; ++j) {
            const OperatorFacts &op1 = operators[i];
            const OperatorFacts &op2 = operators[j];
            if (subset(op1.effects, op2.preconditions) && subset(op2.effects, op1.preconditions)) {
                pairs.emplace_back(i, j);
                unique_actions.insert(i);
                unique_actions.insert(j);
            }
        }
    }
    cout << "Found " << pairs.size() << " pairs of opposite actions (" << pairs.size() * 2
         << " tuples). Of these, " << unique_actions.size() << " are unique actions." << endl;
    return pairs;
}

/*
  SasProblem.get_count_sequential_operator_pairs: the ordered pairs of
  different operators where the second one is applicable after the first one,
  as far as their own facts tell.
*/
long long count_sequential_operator_pairs(const vector<OperatorFacts> &operators) {
    // The values that hold after each operator (its effects, else its preconditions).
    vector<unordered_map<int, int>> state_after(operators.size());
    for (size_t op = 0; op < operators.size(); ++op) {
        for (const FactPair &effect : operators[op].effects)
            state_after[op][effect.var] = effect.value;
        for (const FactPair &precondition : operators[op].preconditions)
            state_after[op].try_emplace(precondition.var, precondition.value);
    }
    long long count = 0;
    for (size_t op1 = 0; op1 < operators.size(); ++op1) {
        for (size_t op2 = 0; op2 < operators.size(); ++op2) {
            if (op1 == op2)
                continue;
            bool can_sequence = true;
            for (const FactPair &precondition : operators[op2].preconditions) {
                auto known = state_after[op1].find(precondition.var);
                if (known != state_after[op1].end() && known->second != precondition.value) {
                    can_sequence = false;
                    break;
                }
            }
            if (can_sequence)
                ++count;
        }
    }
    return count;
}

/*
  The consecutive actions relations over the original operators (and the
  no-op, last). They are translated to the labels of the CP automata when the
  constraints file is written (translate_to_labels).
*/
struct ConsecutiveRelations {
    bool has_successors = false;
    vector<vector<bool>> possible; // possible[a][b]: action b may follow action a
    bool has_opposite_pairs = false;
    vector<pair<int, int>> opposite_pairs;
};

// The same relations over the labels of the CP automata, as written to the constraints file.
struct LabelRelations {
    bool has_successors = false;
    int num_labels = 0;
    long long num_possible = 0;
    vector<pair<int, int>> forbidden_successors;
    bool has_opposite_pairs = false;
    vector<pair<int, int>> opposite_pairs; // each pair forbids both orders
};

/*
  Consecutive actions relations of generateCPAutomataLocal: the actions that
  may follow each other, or the opposite actions only.
  Returns false if the driver has to stop (--exit-after opposite_actions_detection).
*/
bool compute_consecutive_actions_relations(const AbstractTask &task, const vector<OperatorFacts> &operators,
                                           const PipelineOptions &options, ConsecutiveRelations &relations) {
    int num_operators = operators.size();
    bool has_no_op = options.no_op || options.no_op_2;
    if (options.mutex_actions_constraints != "none" && num_operators < 100'000) {
        int n = num_operators + (has_no_op ? 1 : 0);
        vector<vector<bool>> possible(n, vector<bool>(n, true));
        long long num_possible = static_cast<long long>(n) * n;
        auto forbid = [&](int a, int b) {
            if (possible[a][b]) {
                possible[a][b] = false;
                --num_possible;
            }
        };

        if (options.has_consecutive_actions_constraints) {
            const auto &constraints = options.consecutive_actions_constraints;
            if (contains(constraints, "opposite")) {
                for (auto [a, b] : opposite_operators(operators)) {
                    forbid(a, b);
                    forbid(b, a);
                }
            }
            if (contains(constraints, "same")) {
                for (int i = 0; i < n - (has_no_op ? 1 : 0); ++i)
                    forbid(i, i);
            }
            if (contains(constraints, "nop") && has_no_op) {
                for (int i = 0; i < n - 1; ++i)
                    forbid(n - 1, i);
            }
        }

        // SasProblem.remove_mutex_actions_tuples
        const string &mutex = options.mutex_actions_constraints;
        bool use_postconditions = mutex.find("postconditions") != string::npos;
        bool use_mutex = mutex.find("mutex") != string::npos;
        long long num_mutex_tuples = 0;
        for (int i = 0; i < num_operators; ++i) {
            const OperatorFacts &op1 = operators[i];
            unordered_map<int, int> partial_state;
            if (use_postconditions) {
                partial_state = make_partial_state(op1.preconditions);
                for (const FactPair &effect : op1.effects)
                    partial_state[effect.var] = effect.value;
            } else {
                partial_state = make_partial_state(op1.effects);
            }
            for (int j = 0; j < num_operators; ++j) {
                if (i == j)
                    continue;
                if (!operator_applicable(task, partial_state, operators[j], use_mutex)) {
                    forbid(i, j);
                    ++num_mutex_tuples;
                }
            }
        }
        cout << "Found " << num_mutex_tuples << " tuples of mutex actions" << endl;

        relations.has_successors = true;
        relations.possible = move(possible);
    } else {
        if (options.mutex_actions_constraints != "none" && num_operators >= 100'000)
            cout << "Too many actions for computing possible successive actions relations, skipping" << endl;
        if (options.has_consecutive_actions_constraints &&
            contains(options.consecutive_actions_constraints, "opposite")) {
            relations.has_opposite_pairs = true;
            relations.opposite_pairs = opposite_operators(operators);
        }
    }

    if (options.mutex_actions_constraints == "none")
        cout << "Found 0 tuples of mutex actions" << endl;
    if (options.print_nbr_sequential_actions_tuples)
        cout << "Found " << count_sequential_operator_pairs(operators) << " tuple(s) of possible sequential actions" << endl;
    if (options.exit_after == "opposite_actions_detection") {
        cout << "Exit after opposite actions detection" << endl;
        return false;
    }
    return true;
}

/*
  The pairs of labels (l1, l2) that the CP automata already rule out: in some
  automaton, no state reached by l1 has a transition for l2 (or l1 or l2 has no
  transition at all). l2 can never follow l1 in a solution of the CP model, so
  forbidding the pair adds nothing to it.
*/
vector<bool> ruled_out_by_automata(const NDAutomata &automata, const vector<pair<int, int>> &pairs) {
    vector<bool> ruled_out(pairs.size(), false);
    for (const NDAutomaton &automaton : automata.automata) {
        int num_local = automaton.num_local_actions;
        // The local actions with a transition from each state.
        vector<vector<int>> applicable(automaton.num_states);
        for (int state = 0; state < automaton.num_states; ++state) {
            for (const auto &[local, successors] : automaton.transitions[state])
                applicable[state].push_back(local);
        }
        // follows[a][b]: local action b has a transition from a state reached by local action a.
        vector<vector<bool>> follows(num_local, vector<bool>(num_local, false));
        vector<bool> has_transition(num_local, false);
        for (int state = 0; state < automaton.num_states; ++state) {
            for (const auto &[local, successors] : automaton.transitions[state]) {
                has_transition[local] = true;
                vector<bool> &row = follows[local];
                for (int successor : successors) {
                    for (int next : applicable[successor])
                        row[next] = true;
                }
            }
        }
        for (size_t i = 0; i < pairs.size(); ++i) {
            if (ruled_out[i])
                continue;
            int a = automaton.local_mapping[pairs[i].first];
            int b = automaton.local_mapping[pairs[i].second];
            if (a == -1 || b == -1 || !has_transition[a] || !has_transition[b] || !follows[a][b])
                ruled_out[i] = true;
        }
    }
    return ruled_out;
}

/*
  The consecutive actions relations over the labels of the CP automata. A label
  stands for several actions, any of which can be chosen in a plan: two labels
  may follow each other if some actions of them may, so a pair of labels is
  forbidden only if all the pairs of their actions are. action_labels[a] is the
  label of action a of the relations (the original operators, then the no-op).
  The pairs already ruled out by the CP automata are not kept: they would only
  make the CP model larger.
*/
LabelRelations translate_to_labels(const ConsecutiveRelations &relations, const vector<int> &action_labels,
                                   const NDAutomata &automata) {
    int num_labels = automata.num_actions();
    LabelRelations labels;
    labels.num_labels = num_labels;
    size_t num_ruled_out_successors = 0;
    size_t num_ruled_out_opposite = 0;
    int num_actions = action_labels.size();
    if (relations.has_successors) {
        if (static_cast<int>(relations.possible.size()) != num_actions)
            throw runtime_error("The consecutive actions relations are not over the actions of the reductions");
        vector<vector<bool>> possible(num_labels, vector<bool>(num_labels, false));
        for (int a = 0; a < num_actions; ++a) {
            vector<bool> &label_row = possible[action_labels[a]];
            const vector<bool> &row = relations.possible[a];
            for (int b = 0; b < num_actions; ++b) {
                if (row[b])
                    label_row[action_labels[b]] = true;
            }
        }
        labels.has_successors = true;
        vector<pair<int, int>> forbidden;
        for (int l1 = 0; l1 < num_labels; ++l1) {
            for (int l2 = 0; l2 < num_labels; ++l2) {
                if (possible[l1][l2])
                    ++labels.num_possible;
                else
                    forbidden.emplace_back(l1, l2);
            }
        }
        // The pairs ruled out by the automata are counted as pairs that may follow each other: the driver posts the
        // forbidden pairs or the allowed ones, whichever are fewer, and the automata forbid them anyway.
        vector<bool> ruled_out = ruled_out_by_automata(automata, forbidden);
        for (size_t i = 0; i < forbidden.size(); ++i) {
            if (ruled_out[i])
                ++labels.num_possible;
            else
                labels.forbidden_successors.push_back(forbidden[i]);
        }
        num_ruled_out_successors = forbidden.size() - labels.forbidden_successors.size();
    }
    if (relations.has_opposite_pairs) {
        // A pair of labels is opposite if all the pairs of their actions are: count the opposite pairs of actions
        // of each pair of labels (the relation is symmetric).
        vector<int> label_size(num_labels, 0);
        for (int label : action_labels)
            ++label_size[label];
        set<pair<int, int>> opposite;
        for (auto [a, b] : relations.opposite_pairs) {
            opposite.emplace(a, b);
            opposite.emplace(b, a);
        }
        map<pair<int, int>, long long> num_opposite;
        for (auto [a, b] : opposite)
            ++num_opposite[{action_labels[a], action_labels[b]}];
        labels.has_opposite_pairs = true;
        vector<pair<int, int>> opposite_labels;
        for (const auto &[label_pair, count] : num_opposite) {
            auto [l1, l2] = label_pair;
            if (l1 <= l2 && count == static_cast<long long>(label_size[l1]) * label_size[l2])
                opposite_labels.emplace_back(l1, l2);
        }
        // Both orders are forbidden: a pair is dropped only if the automata rule out both.
        vector<pair<int, int>> both_orders;
        for (auto [l1, l2] : opposite_labels) {
            both_orders.emplace_back(l1, l2);
            both_orders.emplace_back(l2, l1);
        }
        vector<bool> ruled_out = ruled_out_by_automata(automata, both_orders);
        for (size_t i = 0; i < opposite_labels.size(); ++i) {
            if (!ruled_out[2 * i] || !ruled_out[2 * i + 1])
                labels.opposite_pairs.push_back(opposite_labels[i]);
        }
        num_ruled_out_opposite = opposite_labels.size() - labels.opposite_pairs.size();
    }
    cout << "Consecutive actions relations over the labels: " << labels.forbidden_successors.size()
         << " forbidden pair(s) of labels (" << num_ruled_out_successors << " more already ruled out by the automata), "
         << labels.opposite_pairs.size() << " opposite pair(s) of labels (" << num_ruled_out_opposite
         << " more already ruled out by the automata)" << endl;
    return labels;
}

pair<OperatorCountingLP, vector<int>> load_operator_counting(const PipelineOptions &options,
                                                             const ActionReductions &reductions) {
    if (options.opc_lp_path.empty())
        throw runtime_error("the operator-counting label reductions need the operator-counting LP");
    ifstream in(options.opc_lp_path);
    if (!in)
        throw runtime_error("operator-counting LP '" + options.opc_lp_path + "' does not exist, "
                            "it is required by the operator-counting label reductions");
    OperatorCountingLP lp = read_operator_counting_lp(in, cout);
    if (!lp.all_unit_coefficients)
        throw runtime_error("operator-counting LP '" + options.opc_lp_path +
                            "' has non-unit coefficients, which are not supported.");
    if (options.max_op_counting > 0) {
        lp.sort_constraints_by_actions();
        lp.keep_n_constraints(options.max_op_counting);
    }
    return {move(lp), reductions.original_action_to_label()};
}

// Length of a shortest path from the initial state to a goal state; throws if there is none (unsolvable task).
long long shortest_goal_path(const NDAutomaton &automaton) {
    vector<int> distance(automaton.num_states, -1);
    deque<int> queue = {automaton.init_state};
    distance[automaton.init_state] = 0;
    vector<bool> is_goal(automaton.num_states, false);
    for (int goal : automaton.goal_states)
        is_goal[goal] = true;
    while (!queue.empty()) {
        int state = queue.front();
        queue.pop_front();
        if (is_goal[state])
            return distance[state];
        for (const auto &[action, successors] : automaton.transitions[state]) {
            for (int successor : successors) {
                if (distance[successor] == -1) {
                    distance[successor] = distance[state] + 1;
                    queue.push_back(successor);
                }
            }
        }
    }
    throw runtime_error("The task is unsolvable: an automaton has no path to a goal state");
}

void copy_section(ostream &out, const string &name, const string &path) {
    ifstream in(path);
    if (!in)
        throw runtime_error("Cannot open " + path);
    out << "begin_" << name << '\n' << in.rdbuf();
    out << "\nend_" << name << '\n';
}

/*
  The constraints file of the CP model: the constraints over the labels of the
  CP automata, by section. The plan length bounds are appended later, in a
  bounds section (the last one counts).
*/
void write_constraints(ostream &out, int num_labels, const string &lp_path, const string &landmarks_path,
                       const LabelRelations &relations) {
    out << "ccbp_constraints 1\n";
    out << "num_labels " << num_labels << '\n';
    if (!lp_path.empty())
        copy_section(out, "operator_counting", lp_path);
    if (!landmarks_path.empty())
        copy_section(out, "landmarks", landmarks_path);
    if (relations.has_successors) {
        // The pairs of labels that cannot follow each other; the other pairs may.
        out << "begin_successors\n" << relations.num_labels << ' ' << relations.num_possible << ' '
            << relations.forbidden_successors.size() << '\n';
        for (auto [a, b] : relations.forbidden_successors)
            out << a << ' ' << b << '\n';
        out << "end_successors\n";
    }
    if (relations.has_opposite_pairs) {
        out << "begin_opposite_pairs\n" << relations.opposite_pairs.size() << '\n';
        for (auto [a, b] : relations.opposite_pairs)
            out << a << ' ' << b << '\n';
        out << "end_opposite_pairs\n";
    }
}

const char *label_reduction_name(const string &strategy) {
    if (strategy == "greedy") return "GreedyLabelReduction";
    if (strategy == "sss") return "SSSLabelReduction";
    if (strategy == "dss") return "DSSLabelReduction";
    if (strategy == "sss_greedy") return "SSSGreedyLabelReduction";
    if (strategy == "lropc") return "LROPCLabelReduction";
    if (strategy == "lropc_sss_greedy") return "LROPCSSSGreedyLabelReduction";
    throw invalid_argument("Unknown label reduction strategy: " + strategy);
}

const char *nd_label_reduction_name(const string &strategy) {
    if (strategy == "greedy") return "GreedyNDLabelReduction";
    if (strategy == "lropc") return "NDLROPCLabelReduction";
    throw invalid_argument("Unknown non-deterministic label reduction strategy: " + strategy);
}

template<typename Reduction, typename AutomataType>
void run_label_reduction(Reduction &label_reduction, AutomataType &automata, ActionReductions &reductions,
                         const PipelineOptions &options, const string &name) {
    cout << '\n';
    print_info_line("Label Reduction using \"" + name + "\" strategy");
    clock_t start = clock();
    int num_actions = automata.num_actions();
    if (options.lr_fix_point) {
        int num_passes = label_reduction.apply_fix_point(automata);
        print_info_line("Label reduction fix point reached after " + to_string(num_passes) + " reducing pass(es)");
    } else {
        label_reduction.apply_all_automata(automata);
    }
    if (automata.num_actions() < num_actions)
        reductions.apply_mapping(label_reduction.get_mapping(), automata.num_actions());
    print_info_line("Time for label reduction: " + python_float_repr(cpu_seconds_since(start)) + " sec");
}

CPParameters cp_parameters(const vector<int> &costs, int action_cost_flag) {
    CPParameters parameters;
    parameters.action_cost_flag = action_cost_flag;
    if (!all_of(costs.begin(), costs.end(), [](int cost) {return cost == 1;})) {
        parameters.b = 0;
        parameters.c = *min_element(costs.begin(), costs.end());
    }
    return parameters;
}
}

int run_pipeline(const PipelineOptions &options, const AbstractTask &task, Automata atomic_automata,
                 PipelineResult &result) {
    TaskProxy task_proxy(task);
    const vector<OperatorFacts> operators = get_operator_facts(task);
    // Operator.to_solution_string of each operator, as written in reductions.yaml.
    vector<string> solution_strings;
    for (OperatorProxy op : task_proxy.get_operators())
        solution_strings.push_back(solution_string(op.get_name()));
    const size_t num_variables = task_proxy.get_variables().size();

    ConsecutiveRelations consecutive_relations;
    if (!compute_consecutive_actions_relations(task, operators, options, consecutive_relations))
        return EXIT_AFTER_STEP;

    cout << task_proxy.get_goals().size() << " atoms in the goal" << endl;
    long long num_preconditions = 0;
    for (const OperatorFacts &op : operators)
        num_preconditions += op.preconditions.size();
    cout << num_preconditions << " total of summed action preconditions" << endl;

    // The automata written by Scorpion, or the CP automata of --manual-automata (used as they are).
    bool manual = !options.manual_automata_path.empty();
    Automata automata;
    int action_cost_flag;
    if (manual) {
        ifstream in(options.manual_automata_path);
        if (!in)
            throw runtime_error("Cannot open " + options.manual_automata_path);
        automata = read_cp_automata(in, action_cost_flag);
    } else {
        automata = move(atomic_automata);
        action_cost_flag = automata.non_unit_cost;
        // The Automaton constructor reduces the local actions of the automata without a local mapping.
        for (Automaton &automaton : automata.automata)
            reduce_local_equivalent_actions(automaton);

        // Variable and state facts of each automaton: state s of automaton v is value s of variable v.
        for (size_t i = 0; i < automata.automata.size() && i < num_variables; ++i) {
            Automaton &automaton = automata.automata[i];
            int var = i;
            automaton.variables = {var};
            for (int value = 0; value < task_proxy.get_variables()[var].get_domain_size(); ++value)
                automaton.state_facts.push_back({FactPair(var, value)});
        }
    }

    // ----- Mutex pruning (Automata.from_file) -----
    if (options.remove_mutex_transitions && !manual) {
        int total_removed = 0;
        clock_t start = clock();
        cout << "Preprocessing automatons with mutex groups to remove non applicable actions:" << endl;
        size_t num_pruned = min(automata.automata.size(), num_variables);
        for (size_t i = 0; i < num_pruned; ++i) {
            int num_removed = prune_inapplicable_transitions(automata.automata[i], task, operators);
            cout << "-a" << i << ' ' << num_removed << " transitions(s) removed" << endl;
            total_removed += num_removed;
        }
        cout << "Total removed transitions by preprocessing: " << total_removed << endl;
        cout << "Time for removing mutex transitions: " << python_float_repr(cpu_seconds_since(start)) << " sec" << endl;
    } else {
        cout << "Total removed transitions by preprocessing: 0" << endl;
    }

    // Unreachable and dead-end states of the atomic automata (pruned by Scorpion's merge-and-shrink before).
    if (!manual) {
        int num_pruned = 0;
        for (Automaton &automaton : automata.automata)
            num_pruned += prune_and_shrink(automaton, false);
        cout << "Pruned states of the atomic automata: " << num_pruned << endl;
    }

    if (options.unit_cost) {
        for (Automaton &automaton : automata.automata)
            remove_local_mapping(automaton);
        action_cost_flag = 0;
        fill(automata.costs.begin(), automata.costs.end(), 1);
        for (Automaton &automaton : automata.automata) {
            automaton.has_costs = false;
            automaton.local_costs.clear();
        }
        for (Automaton &automaton : automata.automata)
            reduce_local_equivalent_actions(automaton);
    }

    ActionReductions reductions(operators.size());

    // The manual automata are not preprocessed nor refined (generateCPAutomataLocal).
    // The label reduction decides the representation: the automata stay deterministic unless it is non-deterministic.
    bool non_deterministic = !manual && options.nd_label_reduction != "none";
    // Exact bisimulation after each merge and after the label reduction.
    bool shrink = options.shrink;
    NDAutomata nd_automata;
    if (!manual) {
        // ----- Equivalent actions -----
        if (options.remove_equivalent_actions || options.print_equivalent_actions_groups) {
            Partition equivalent(automata.num_actions());
            for (const Automaton &automaton : automata.automata)
                equivalent.refine_by_labels(automaton.local_mapping);
            vector<vector<int>> groups;
            for (vector<int> &group : equivalent.blocks()) {
                if (!group.empty())
                    groups.push_back(move(group));
            }
            if (options.print_equivalent_actions_groups) {
                print_info_block({"Equivalent actions groups"});
                for (const vector<int> &group : equivalent.blocks()) {
                    if (group.size() > 1) {
                        cout << "Group of equivalent actions :" << endl;
                        for (int action : group)
                            cout << "\t- " << solution_strings[action] << endl;
                    }
                }
            }
            if (options.remove_equivalent_actions && equivalent.size() < automata.num_actions()) {
                sort(groups.begin(), groups.end(), [](const vector<int> &a, const vector<int> &b) {return a[0] < b[0];});
                vector<int> representatives;
                for (const vector<int> &group : groups)
                    representatives.push_back(group[0]);

                int old_num_actions = automata.num_actions();
                vector<int> new_costs;
                for (const vector<int> &group : groups) {
                    int cost = automata.costs[group[0]];
                    for (int action : group) {
                        if (automata.costs[action] != cost)
                            throw runtime_error("All equivalent actions should have the same cost");
                    }
                    new_costs.push_back(cost);
                }
                automata.costs = move(new_costs);
                for (Automaton &automaton : automata.automata) {
                    vector<int> new_local_mapping;
                    for (int representative : representatives)
                        new_local_mapping.push_back(automaton.local_mapping[representative]);
                    automaton.local_mapping = move(new_local_mapping);
                }
                cout << "Removed equivalent actions: " << old_num_actions - static_cast<int>(representatives.size()) << endl;
                reductions.remove_equivalent_actions(groups, representatives);
            }
        }
        if (options.exit_after == "equivalent_actions") {
            print_info_line("Exit after equivalent actions detection");
            return EXIT_AFTER_STEP;
        }
        print_info_block({"Parsing and preprocessing completed"}, false, true);

        // ----- Refinement -----
        // The automata used to translate the plans (the driver's layered search works in every mode).
        write_file(options.simulation_automata_path, [&](ostream &out) {write_automata(automata, out);});

        clock_t refinement_start = clock();
        print_info_block({"Refine automata using 'RefinementClassic'"}, true, false);
        // The merges are computed on the deterministic automata, before the conversion to non-deterministic ones.
        if (options.merge_min != -1 || options.merge_objects) {
            FactsRelations relations(task);
            cout << '\n';
            print_info_line(string("Merge using \"") + (options.merge_min != -1 ? "MergeMin" : "MergeInvariantsObjects") +
                            "\" strategy");
            clock_t start = clock();
            if (options.merge_min != -1)
                merge_min(automata, options.merge_min, action_cost_flag, task, relations, shrink,
                          options.merge_max_states);
            else
                merge_objects(automata, action_cost_flag, task, relations, options.keep_unimportant_variables,
                              shrink, options.merge_max_states);
            print_info_line("Time for merging: " + python_float_repr(cpu_seconds_since(start)) + " sec");
        }
        if (non_deterministic) {
            nd_automata.costs = automata.costs;
            for (const Automaton &automaton : automata.automata)
                nd_automata.automata.push_back(to_nd_automaton(automaton));
            automata = Automata();
        }
        /*
          Label reduction, then exact bisimulation (with shrinking). With
          --lr-fix-point, they alternate until neither changes anything: merged
          states can make labels locally equivalent, and reduced labels can make
          more states bisimilar.
        */
        auto shrink_all = [&](auto &all_automata, auto shrink_one) {
            cout << '\n';
            print_info_line("Shrinking using bisimulation");
            clock_t start = clock();
            int num_removed = 0;
            for (auto &automaton : all_automata.automata)
                num_removed += shrink_one(automaton);
            print_info_line("Bisimulation shrinking removed " + to_string(num_removed) + " state(s)");
            print_info_line("Time for shrinking: " + python_float_repr(cpu_seconds_since(start)) + " sec");
            return num_removed;
        };
        if (!non_deterministic) {
            auto reduce = [&]() {
                if (options.label_reduction == "none")
                    return false;
                LabelReduction label_reduction(options.label_reduction);
                if (label_reduction.uses_operator_counting()) {
                    auto [lp, original_to_label] = load_operator_counting(options, reductions);
                    label_reduction.set_operator_counting(move(lp), move(original_to_label));
                }
                int num_actions = automata.num_actions();
                run_label_reduction(label_reduction, automata, reductions, options,
                                    label_reduction_name(options.label_reduction));
                return automata.num_actions() < num_actions;
            };
            auto shrink_one = [](Automaton &automaton) {return prune_and_shrink(automaton, true);};
            reduce();
            if (shrink) {
                while (shrink_all(automata, shrink_one) > 0 && options.lr_fix_point && reduce()) {
                }
            }
        } else {
            auto reduce = [&]() {
                if (options.nd_label_reduction == "none")
                    return false;
                NDLabelReduction label_reduction(options.nd_label_reduction);
                if (label_reduction.uses_operator_counting()) {
                    auto [lp, original_to_label] = load_operator_counting(options, reductions);
                    label_reduction.set_operator_counting(move(lp), move(original_to_label));
                }
                int num_actions = nd_automata.num_actions();
                run_label_reduction(label_reduction, nd_automata, reductions, options,
                                    nd_label_reduction_name(options.nd_label_reduction));
                return nd_automata.num_actions() < num_actions;
            };
            auto shrink_one = [](NDAutomaton &automaton) {return shrink_bisimulation(automaton);};
            reduce();
            if (shrink) {
                while (shrink_all(nd_automata, shrink_one) > 0 && options.lr_fix_point && reduce()) {
                }
            }
        }
        cout << '\n';
        print_info_block({"Refinement finished after " + python_float_repr(cpu_seconds_since(refinement_start)) + " sec"},
                         false, true);
    }

    /*
      The CP automata are non-deterministic automata, posted with CP-SAT's non-deterministic automaton constraint. The
      conversion also merges the locally equivalent actions of the automata with a local mapping.
    */
    if (!non_deterministic) {
        nd_automata.costs = automata.costs;
        for (const Automaton &automaton : automata.automata)
            nd_automata.automata.push_back(to_nd_automaton(automaton));
        automata = Automata();
    }

    // ----- Lower bound on the plan length: the shortest path to a goal in each automaton -----
    result.automata_lower_bound = 0;
    for (const NDAutomaton &automaton : nd_automata.automata)
        result.automata_lower_bound = max(result.automata_lower_bound, shortest_goal_path(automaton));

    // ----- No-op -----
    if (options.no_op || options.no_op_2) {
        print_info_block({"Add no-op"});
        reductions.add_no_op();
        if (options.no_op) {
            if (!options.no_op_right_pad_constraints)
                cout << "Adding no-op automaton for right padding constraints" << endl;
            add_no_op_action(nd_automata, action_cost_flag, !options.no_op_right_pad_constraints);
        } else {
            add_no_op_action2(nd_automata, action_cost_flag);
        }
    }

    // ----- Constraints over the reduced labels -----
    string lp_to_post = options.post_operator_counting ? options.opc_lp_path : "";
    string landmarks_to_post = options.post_landmarks ? options.landmark_graph_path : "";
    if (!lp_to_post.empty() || !landmarks_to_post.empty())
        remap_constraints(reductions.original_action_to_label(), reductions.num_groups(), lp_to_post,
                          landmarks_to_post);

    // ----- Outputs (transform_and_output_automata) -----
    if (options.label_reduction != "none" || options.nd_label_reduction != "none")
        cout << "Label reduction from " << operators.size() << " to " << reductions.num_groups() << endl;
    write_file(options.reductions_path, [&](ostream &out) {reductions.write_yaml(solution_strings, out);});
    if (!options.original_to_label_path.empty()) {
        // The number of labels (with the no-op), then the label of each original operator.
        write_file(options.original_to_label_path, [&](ostream &out) {
            out << reductions.num_groups() << '\n';
            const vector<int> &labels = reductions.original_action_to_label();
            for (size_t i = 0; i < labels.size(); ++i)
                out << (i > 0 ? " " : "") << labels[i];
            out << '\n';
        });
    }

    // Automata.set_CP_parameters: the manual automata already had CP parameters.
    if (manual)
        cout << "WARNING: CP parameters are already set, change old values to new ones" << endl;

    /*
      The CP automata are written with their locally equivalent actions merged (the CP-SAT reader does not merge them):
      the steps above can leave some, e.g. the no-op and the cost-0 actions looping on every state of an automaton
      whose states are all goal states. With no_local_mapping, every action is its own local action instead.
    */
    for (NDAutomaton &automaton : nd_automata.automata) {
        if (options.no_local_mapping)
            remove_local_mapping(automaton);
        else
            reduce_local_equivalent_actions(automaton);
    }

    CPParameters parameters = cp_parameters(nd_automata.costs, action_cost_flag);
    write_file(options.cp_automata_path, [&](ostream &out) {write_cp_automata(nd_automata, parameters, out);});
    vector<int> num_states;
    vector<int> num_local_actions;
    for (const NDAutomaton &automaton : nd_automata.automata) {
        num_states.push_back(automaton.num_states);
        num_local_actions.push_back(automaton.num_local_actions);
    }
    print_automata_info(num_states, num_local_actions, nd_automata.num_actions());

    // The consecutive actions relations are over the original operators (and the no-op, the last label).
    vector<int> action_labels = reductions.original_action_to_label();
    if (options.no_op || options.no_op_2)
        action_labels.push_back(reductions.num_groups() - 1);
    if ((consecutive_relations.has_successors || consecutive_relations.has_opposite_pairs) &&
        reductions.num_groups() != nd_automata.num_actions())
        throw runtime_error("The consecutive actions relations cannot be translated to the labels of the CP automata");
    LabelRelations label_relations = translate_to_labels(consecutive_relations, action_labels, nd_automata);
    write_file(options.constraints_path, [&](ostream &out) {
        write_constraints(out, nd_automata.num_actions(), lp_to_post, landmarks_to_post, label_relations);
    });
    cout.flush();
    return 0;
}
}
