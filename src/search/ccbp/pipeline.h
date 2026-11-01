#ifndef CCBP_PIPELINE_H
#define CCBP_PIPELINE_H

#include <string>
#include <vector>

class AbstractTask;

namespace ccbp {
// Exit code of the tool when a --exit-after step is reached (the driver then stops with code 0).
const int EXIT_AFTER_STEP = 10;

/*
  Options of the refinement stage, named as the arguments of the driver
  (driver/arguments.py).
*/
struct PipelineOptions {
    std::string manual_automata_path; // CP automata used instead, without preprocessing and refinement

    bool unit_cost = false;
    bool remove_mutex_transitions = false;
    bool remove_equivalent_actions = false;
    bool print_equivalent_actions_groups = false;
    std::string exit_after; // equivalent_actions, opposite_actions_detection or empty

    // Consecutive actions relations (generateCPAutomataLocal).
    std::string mutex_actions_constraints = "none";
    std::vector<std::string> consecutive_actions_constraints; // opposite, same, nop
    bool has_consecutive_actions_constraints = false;
    bool print_nbr_sequential_actions_tuples = false;

    int merge_min = -1; // percentage, -1: no merge
    bool merge_objects = false;
    int merge_max_states = -1; // maximum number of states of a merged automaton, -1: no bound
    bool keep_unimportant_variables = false;

    std::string label_reduction = "none";
    std::string nd_label_reduction = "none";
    bool shrink = false;
    bool lr_fix_point = false;
    std::string opc_lp_path; // operator-counting LP over the original operators (lropc, posted constraints)
    std::string landmark_graph_path; // action landmark graph over the original operators
    bool post_operator_counting = false; // write the LP to the constraints file
    bool post_landmarks = false; // write the landmark graph to the constraints file
    int max_op_counting = -1;

    bool no_op = false;
    bool no_op_2 = false;
    bool no_op_right_pad_constraints = false;


    bool no_local_mapping = false;

    std::string cp_automata_path;
    std::string constraints_path;
    std::string reductions_path;
    std::string simulation_automata_path;
    std::string original_to_label_path; // number of labels, then the label of each original operator
};

struct Automata;

struct PipelineResult {
    // The longest shortest path to a goal state over the automata (before the no-op): a plan length lower bound.
    long long automata_lower_bound = 0;
};

/*
  The refinement stage, from the task and its atomic automata to the CP
  automata, reductions.yaml and the automata used to translate the plans.
  Returns the exit code of the tool.
*/
int run_pipeline(const PipelineOptions &options, const AbstractTask &task, Automata atomic_automata,
                 PipelineResult &result);
}

#endif
