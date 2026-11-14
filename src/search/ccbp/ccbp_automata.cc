#include "ccbp_automata.h"

#include "automata.h"
#include "output.h"

#include "../evaluation_context.h"
#include "../evaluation_result.h"
#include "../evaluator.h"
#include "../landmarks/disj_action_landmark_graph.h"
#include "../landmarks/landmark_factory.h"
#include "../merge_and_shrink/factored_transition_system.h"
#include "../merge_and_shrink/fts_factory.h"
#include "../merge_and_shrink/labels.h"
#include "../merge_and_shrink/transition_system.h"
#include "../operator_counting/operator_counting_heuristic.h"
#include "../plan_manager.h"
#include "../plugins/plugin.h"
#include "../task_utils/task_properties.h"
#include "../tasks/cost_adapted_task.h"
#include "../utils/component_errors.h"
#include "../utils/system.h"
#include "../utils/timer.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace std;
using merge_and_shrink::FactoredTransitionSystem;
using merge_and_shrink::LocalLabelInfo;
using merge_and_shrink::TransitionSystem;

namespace ccbp {
namespace {
// Exit code when the pipeline fails (bad input, unsupported task); Scorpion's driver expects codes >= 10.
const int EXIT_FAILURE_CODE = static_cast<int>(utils::ExitCode::SEARCH_CRITICAL_ERROR);

void print_vector(const vector<int> &vec, ostream &out) {
    for (int value : vec)
        out << value << " ";
    out << '\n';
}

/*
  The atomic automata in the automata file format (read_automata), as the
  merge-and-shrink heuristic wrote them for CCBP: one automaton per variable,
  the locally equivalent labels of the same cost grouped into one local action.
*/
void write_atomic_automata(const TaskProxy &task_proxy, const FactoredTransitionSystem &fts, ostream &out) {
    int num_labels = fts.get_labels().get_num_total_labels();
    bool non_unit_cost = !task_properties::is_unit_cost(task_proxy);
    out << "Actions: " << num_labels << '\n';
    out << "NonUnitCost: " << non_unit_cost << '\n';
    out << "Costs:";
    for (int cost : task_properties::get_operator_costs(task_proxy))
        out << " " << cost;
    out << '\n';
    out << "Automata: " << fts.get_num_active_entries() << "\n\n";
    for (int index : fts) {
        const TransitionSystem &ts = fts.get_transition_system(index);
        int num_states = ts.get_size();
        out << num_states << '\n' << ts.get_init_state() << '\n';
        vector<int> goal_states;
        for (int state = 0; state < num_states; ++state) {
            if (ts.is_goal_state(state))
                goal_states.push_back(state);
        }
        out << goal_states.size() << " ";
        for (int state : goal_states)
            out << " " << state;
        out << '\n';

        // Locally equivalent labels with the same cost form one local action, in the order of Scorpion's former
        // automata writer (an unordered_map over the costs).
        vector<int> global_to_local(num_labels, -1);
        vector<int> local_costs;
        vector<vector<int>> transition_function(num_states);
        for (const LocalLabelInfo &local_label_info : ts) {
            unordered_map<int, set<int>> cost_to_labels;
            for (int label : local_label_info.get_label_group())
                cost_to_labels[task_proxy.get_operators()[label].get_cost()].insert(label);
            for (const auto &[cost, labels] : cost_to_labels) {
                int local = local_costs.size();
                for (int label : labels)
                    global_to_local[label] = local;
                local_costs.push_back(cost);
                for (vector<int> &row : transition_function)
                    row.push_back(-1);
                for (const merge_and_shrink::Transition &transition : local_label_info.get_transitions())
                    transition_function[transition.src][local] = transition.target;
            }
        }
        out << local_costs.size() << '\n';
        print_vector(global_to_local, out);
        for (const vector<int> &row : transition_function)
            print_vector(row, out);
        out << non_unit_cost << '\n';
        if (non_unit_cost)
            print_vector(local_costs, out);
        out << '\n';
    }
}

/*
  The atomic automata of the task. The states of the automaton of variable v
  are the values of v, in order: no state is pruned here.
*/
Automata compute_atomic_automata(const TaskProxy &task_proxy) {
    utils::LogProxy silent_log = utils::get_silent_log();
    FactoredTransitionSystem fts = merge_and_shrink::create_factored_transition_system(
        task_proxy, false, false, silent_log);
    stringstream text;
    write_atomic_automata(task_proxy, fts, text);
    return read_automata(text);
}
}

CCBPAutomata::CCBPAutomata(
    const shared_ptr<AbstractTask> &task, const PipelineOptions &options,
    bool unit_cost_automata, const shared_ptr<Evaluator> &operator_counting,
    const shared_ptr<landmarks::TaskIndependentLandmarkFactory> &landmark_factory,
    const shared_ptr<Evaluator> &lower_bound,
    const vector<shared_ptr<TaskIndependentSearchAlgorithm>> &lower_bound_searches,
    const vector<shared_ptr<TaskIndependentSearchAlgorithm>> &upper_bound_searches,
    double max_time_for_bounds,
    const optional<pair<long long, long long>> &fixed_plan_length,
    OperatorCost cost_type, int bound, double max_time,
    const string &description, utils::Verbosity verbosity)
    : SearchAlgorithm(task, cost_type, bound, max_time, description, verbosity),
      options(options),
      unit_cost_automata(unit_cost_automata),
      operator_counting(operator_counting),
      landmark_factory(landmark_factory),
      lower_bound(lower_bound),
      lower_bound_searches(lower_bound_searches),
      upper_bound_searches(upper_bound_searches),
      max_time_for_bounds(max_time_for_bounds),
      fixed_plan_length(fixed_plan_length) {
}

void CCBPAutomata::write_constraint_inputs() {
    State initial_state = state_registry.get_initial_state();
    if (operator_counting) {
        auto heuristic = dynamic_pointer_cast<operator_counting::OperatorCountingHeuristic>(operator_counting);
        if (!heuristic)
            throw runtime_error(
                "operator_counting must be an operatorcounting(...) evaluator, not wrapped in another "
                "evaluator such as eval_modify_costs (with unit_cost=true, it uses unit costs already)");
        if (!heuristic->write_lp(initial_state, options.opc_lp_path))
            throw runtime_error("The task is unsolvable: the operator-counting constraints detect a dead end");
    }
    if (landmark_factory) {
        // Unit costs, as the landmark constraints count actions.
        shared_ptr<AbstractTask> unit_task = make_shared<tasks::CostAdaptedTask>(task, OperatorCost::ONE);
        shared_ptr<landmarks::LandmarkFactory> factory = landmark_factory->bind_task(unit_task);
        shared_ptr<landmarks::LandmarkGraph> graph = factory->compute_landmark_graph(unit_task);
        landmarks::DisjActionLandmarkGraph action_graph(*graph, TaskProxy(*unit_task).get_initial_state());
        action_graph.dump_int_graph(options.landmark_graph_path);
    }
}

void CCBPAutomata::append_bounds(long long plan_min, long long plan_max) const {
    ofstream out(options.constraints_path, ios::app);
    if (!out)
        throw runtime_error("Cannot open " + options.constraints_path);
    out << "begin_bounds\nplan_min " << plan_min << "\nplan_max " << plan_max << "\nend_bounds\n";
}

shared_ptr<SearchAlgorithm> CCBPAutomata::construct_search(
    const shared_ptr<TaskIndependentSearchAlgorithm> &config) const {
    return config->bind_task(task);
}

void CCBPAutomata::compute_bounds(long long automata_lower_bound) {
    // No upper bound: the driver's sys.maxsize.
    const long long no_upper_bound = numeric_limits<long long>::max();
    if (fixed_plan_length) {
        append_bounds(fixed_plan_length->first, fixed_plan_length->second);
        cout << "Plan length bounds (given): " << fixed_plan_length->first << ", " << fixed_plan_length->second
             << endl;
        return;
    }
    utils::Timer timer;
    print_info_block({"Compute plan length bounds"}, true, false);

    // First the bounds without search, written before any search may run out of memory.
    long long plan_min = automata_lower_bound;
    long long plan_max = no_upper_bound;
    cout << "Automata length lower bound: " << automata_lower_bound << endl;
    if (lower_bound) {
        EvaluationContext context(state_registry.get_initial_state());
        int value = context.get_evaluator_value_or_infinity(lower_bound.get());
        if (value == EvaluationResult::INFTY)
            throw runtime_error("The task is unsolvable: the lower bound heuristic detects a dead end");
        cout << "First length lower bound: " << value << endl;
        plan_min = max<long long>(plan_min, value);
    }
    append_bounds(plan_min, plan_max);
    const pair<long long, long long> written_bounds(plan_min, plan_max);

    // Then the searches, within max_time_for_bounds seconds (the first upper bound search has no time limit).
    for (const shared_ptr<TaskIndependentSearchAlgorithm> &config : lower_bound_searches) {
        double remaining_time = max_time_for_bounds - timer();
        if (remaining_time <= 0)
            break;
        print_info_line("Lower bound search, time remaining: " + python_float_repr(remaining_time) + "s");
        shared_ptr<SearchAlgorithm> search = construct_search(config);
        search->set_max_time(remaining_time);
        search->search();
        if (search->found_solution()) {
            // A* with unit costs: the plan has the optimal length.
            long long length = search->get_plan().size();
            cout << "Solved while computing lower bound, plan length: " << length << endl;
            plan_min = max(plan_min, length);
            plan_max = min(plan_max, length);
        } else {
            int f_value = search->get_statistics().get_lastjump_f_value();
            cout << "lower bound length: " << (f_value >= 0 ? to_string(f_value) : "None") << endl;
            plan_min = max<long long>(plan_min, f_value);
        }
    }
    for (size_t i = 0; i < upper_bound_searches.size() && plan_min < plan_max; ++i) {
        double remaining_time = max_time_for_bounds - timer();
        if (i > 0 && (plan_max == no_upper_bound || remaining_time <= 0))
            break;
        shared_ptr<SearchAlgorithm> search = construct_search(upper_bound_searches[i]);
        if (i > 0) {
            // Only plans shorter than the best one so far (a plan length, so it fits an int).
            search->set_bound(static_cast<int>(plan_max));
            search->set_max_time(remaining_time);
        }
        search->search();
        if (search->found_solution()) {
            const Plan &plan = search->get_plan();
            cout << "upper bound cost: " << calculate_plan_cost(plan, task_proxy) << endl;
            cout << "upper bound length: " << plan.size() << endl;
            plan_max = min<long long>(plan_max, plan.size());
        } else if (i == 0) {
            cout << "Failed to find an upper bound" << endl;
        }
    }
    if (plan_min > plan_max)
        throw runtime_error("The plan length lower bound exceeds the upper bound");
    // Only when the searches improved them: the last bounds section counts.
    if (make_pair(plan_min, plan_max) != written_bounds)
        append_bounds(plan_min, plan_max);
    cout << "Plan length bounds : " << plan_min << ", " << plan_max << endl;
    print_info_block({"Compute plan length bounds finished after " + python_float_repr(timer()) + " sec"}, false,
                     true);
}

SearchStatus CCBPAutomata::step() {
    int exit_code;
    try {
        write_constraint_inputs();
        shared_ptr<AbstractTask> automata_task = task;
        if (unit_cost_automata)
            automata_task = make_shared<tasks::CostAdaptedTask>(task, OperatorCost::ONE);
        Automata atomic_automata;
        if (options.manual_automata_path.empty())
            atomic_automata = compute_atomic_automata(TaskProxy(*automata_task));
        PipelineResult result;
        exit_code = run_pipeline(options, *task, move(atomic_automata), result);
        // The LP and the landmark graph are in the constraints file now.
        for (const string &path : {options.opc_lp_path, options.landmark_graph_path}) {
            if (!path.empty())
                filesystem::remove(path);
        }
        if (exit_code == 0)
            compute_bounds(result.automata_lower_bound);
    } catch (const bad_alloc &) {
        utils::exit_with(utils::ExitCode::SEARCH_OUT_OF_MEMORY);
    } catch (const exception &e) {
        cout.flush();
        cerr << "ccbp_automata: " << e.what() << endl;
        exit_code = EXIT_FAILURE_CODE;
    }
    cout.flush();
    // The component writes files and does not search: stop here.
    exit(exit_code);
}

void CCBPAutomata::print_statistics() const {
}

class TaskIndependentCCBPAutomata : public TaskIndependentSearchAlgorithm {
    PipelineOptions options;
    bool unit_cost_automata;
    shared_ptr<TaskIndependentEvaluator> operator_counting;
    shared_ptr<landmarks::TaskIndependentLandmarkFactory> landmark_factory;
    shared_ptr<TaskIndependentEvaluator> lower_bound;
    vector<shared_ptr<TaskIndependentSearchAlgorithm>> lower_bound_searches;
    vector<shared_ptr<TaskIndependentSearchAlgorithm>> upper_bound_searches;
    double max_time_for_bounds;
    optional<pair<long long, long long>> fixed_plan_length;
    OperatorCost cost_type;
    int bound;
    double max_time;
    string description;
    utils::Verbosity verbosity;

    static shared_ptr<Evaluator> bind_optional(
        const shared_ptr<TaskIndependentEvaluator> &evaluator,
        const shared_ptr<AbstractTask> &task) {
        return evaluator ? evaluator->bind_task(task) : nullptr;
    }

protected:
    virtual shared_ptr<SearchAlgorithm> create_task_specific_component(
        const shared_ptr<AbstractTask> &task) const override {
        /*
          With unit costs, the operator-counting constraints count actions, to
          match the objective of the CP model (the plan length).
        */
        shared_ptr<AbstractTask> operator_counting_task = task;
        if (unit_cost_automata)
            operator_counting_task = make_shared<tasks::CostAdaptedTask>(task, OperatorCost::ONE);
        return make_shared<CCBPAutomata>(
            task, options, unit_cost_automata,
            bind_optional(operator_counting, operator_counting_task), landmark_factory,
            bind_optional(lower_bound, task), lower_bound_searches,
            upper_bound_searches, max_time_for_bounds, fixed_plan_length,
            cost_type, bound, max_time, description, verbosity);
    }

public:
    TaskIndependentCCBPAutomata(
        const PipelineOptions &options, bool unit_cost_automata,
        const shared_ptr<TaskIndependentEvaluator> &operator_counting,
        const shared_ptr<landmarks::TaskIndependentLandmarkFactory> &landmark_factory,
        const shared_ptr<TaskIndependentEvaluator> &lower_bound,
        const vector<shared_ptr<TaskIndependentSearchAlgorithm>> &lower_bound_searches,
        const vector<shared_ptr<TaskIndependentSearchAlgorithm>> &upper_bound_searches,
        double max_time_for_bounds,
        const optional<pair<long long, long long>> &fixed_plan_length,
        OperatorCost cost_type, int bound, double max_time,
        const string &description, utils::Verbosity verbosity)
        : options(options),
          unit_cost_automata(unit_cost_automata),
          operator_counting(operator_counting),
          landmark_factory(landmark_factory),
          lower_bound(lower_bound),
          lower_bound_searches(lower_bound_searches),
          upper_bound_searches(upper_bound_searches),
          max_time_for_bounds(max_time_for_bounds),
          fixed_plan_length(fixed_plan_length),
          cost_type(cost_type),
          bound(bound),
          max_time(max_time),
          description(description),
          verbosity(verbosity) {
    }
};

class CCBPAutomataFeature : public plugins::TypedFeature<TaskIndependentSearchAlgorithm> {
public:
    CCBPAutomataFeature() : TypedFeature("ccbp_automata") {
        document_title("CCBP automata");
        document_synopsis(
            "Builds the automata of CCBP (Collaborative Constraint-Based Planning) "
            "from the atomic automata of the task: preprocessing, merges, label "
            "reduction and shrinking, then writes the CP automata. It does not "
            "search and exits once the files are written. The options have the "
            "names of the CCBP driver arguments.");

        add_option<string>("cp_automata", "output: the CP automata", "\"automata_cp\"");
        add_option<string>("constraints", "output: the constraints of the CP model and the plan length bounds",
                           "\"constraints.txt\"");
        add_option<string>("reductions", "output: the original operators of each label", "\"reductions.yaml\"");
        add_option<string>("simulation_automata", "output: the automata used to translate the plans",
                           "\"simulation.automata\"");
        add_option<string>("manual_automata", "CP automata used instead of the atomic automata", "\"\"");

        add_option<bool>("unit_cost", "use unit costs", "false");
        add_option<bool>("remove_mutex_transitions", "", "false");
        add_option<bool>("remove_equivalent_actions", "", "false");
        add_option<bool>("print_equivalent_actions_groups", "", "false");
        add_option<string>("exit_after", "equivalent_actions, opposite_actions_detection or empty", "\"\"");
        add_option<string>("mutex_actions_constraints", "", "\"none\"");
        add_list_option<string>("consecutive_actions_constraints", "opposite, same, nop, commuting", "[]");
        add_option<bool>("print_nbr_sequential_actions_tuples", "", "false");

        add_option<int>("merge_min", "merge percentage, -1 for no merge", "-1");
        add_option<bool>("merge_objects", "", "false");
        add_option<int>("merge_max_states",
                        "maximum number of reachable states of a merged automaton (before its pruning and "
                        "shrinking): merge_min stops at the first larger product, merge_objects skips the merge; "
                        "-1 for no bound",
                        "-1", plugins::Bounds("-1", "infinity"));
        add_option<bool>("keep_unimportant_variables", "", "false");

        add_option<string>("label_reduction", "none, greedy, sss, dss, sss_greedy, lropc, lropc_sss_greedy",
                           "\"none\"");
        add_option<string>("nd_label_reduction", "none, greedy, lropc", "\"none\"");
        add_option<bool>("shrink", "exact bisimulation after each merge and after the label reduction", "false");
        add_option<bool>("lr_fix_point", "", "false");

        add_option<shared_ptr<TaskIndependentEvaluator>>(
            "operator_counting",
            "operatorcounting(...) evaluator whose LP in the initial state gives the operator-counting "
            "constraints (lropc strategies, post_operator_counting). With unit_cost=true, it is evaluated "
            "with unit costs.",
            plugins::ArgumentInfo::NO_DEFAULT);
        add_option<bool>("post_operator_counting", "write the operator-counting constraints", "false");
        add_option<int>("max_op_counting", "", "-1");
        add_option<shared_ptr<landmarks::TaskIndependentLandmarkFactory>>(
            "landmarks", "landmark factory of the action landmark constraints (written if given)",
            plugins::ArgumentInfo::NO_DEFAULT);

        add_option<bool>("no_op", "", "false");
        add_option<bool>("no_op_2", "", "false");
        add_option<bool>("no_op_right_pad_constraints", "", "false");
        add_option<bool>("no_local_mapping", "", "false");

        add_option<string>("plan_length", "given plan length bounds as \"MIN MAX\" (empty: compute them)",
                           "\"\"");
        add_option<shared_ptr<TaskIndependentEvaluator>>(
            "lower_bound", "heuristic whose initial value is a plan length lower bound",
            plugins::ArgumentInfo::NO_DEFAULT);
        add_list_option<shared_ptr<TaskIndependentSearchAlgorithm>>(
            "lower_bound_searches",
            "optimal searches with unit costs, for a better lower bound within max_time_for_bounds",
            "[]");
        add_list_option<shared_ptr<TaskIndependentSearchAlgorithm>>(
            "upper_bound_searches",
            "searches for an upper bound: the first one without time limit, the next ones bounded by the best "
            "plan length, within max_time_for_bounds",
            "[]");
        add_option<double>("max_time_for_bounds", "time for the bounds searches, in seconds", "0");

        add_search_algorithm_options_to_feature(*this, "ccbp_automata");
    }

    virtual shared_ptr<TaskIndependentSearchAlgorithm> create_component(
        const plugins::Options &opts) const override {
        auto landmark_factory = opts.get<shared_ptr<landmarks::TaskIndependentLandmarkFactory>>(
            "landmarks", nullptr);
        PipelineOptions options;
        options.manual_automata_path = opts.get<string>("manual_automata");
        options.unit_cost = opts.get<bool>("unit_cost");
        options.remove_mutex_transitions = opts.get<bool>("remove_mutex_transitions");
        options.remove_equivalent_actions = opts.get<bool>("remove_equivalent_actions");
        options.print_equivalent_actions_groups = opts.get<bool>("print_equivalent_actions_groups");
        options.exit_after = opts.get<string>("exit_after");
        options.mutex_actions_constraints = opts.get<string>("mutex_actions_constraints");
        options.consecutive_actions_constraints = opts.get_list<string>("consecutive_actions_constraints");
        options.has_consecutive_actions_constraints = !options.consecutive_actions_constraints.empty();
        options.print_nbr_sequential_actions_tuples = opts.get<bool>("print_nbr_sequential_actions_tuples");
        options.merge_min = opts.get<int>("merge_min");
        options.merge_objects = opts.get<bool>("merge_objects");
        options.merge_max_states = opts.get<int>("merge_max_states");
        options.keep_unimportant_variables = opts.get<bool>("keep_unimportant_variables");
        options.label_reduction = opts.get<string>("label_reduction");
        options.nd_label_reduction = opts.get<string>("nd_label_reduction");
        options.shrink = opts.get<bool>("shrink");
        options.lr_fix_point = opts.get<bool>("lr_fix_point");
        options.max_op_counting = opts.get<int>("max_op_counting");
        options.post_operator_counting = opts.get<bool>("post_operator_counting");
        options.post_landmarks = landmark_factory != nullptr;
        options.no_op = opts.get<bool>("no_op");
        options.no_op_2 = opts.get<bool>("no_op_2");
        options.no_op_right_pad_constraints = opts.get<bool>("no_op_right_pad_constraints");
        options.no_local_mapping = opts.get<bool>("no_local_mapping");
        options.cp_automata_path = opts.get<string>("cp_automata");
        options.constraints_path = opts.get<string>("constraints");
        options.reductions_path = opts.get<string>("reductions");
        options.simulation_automata_path = opts.get<string>("simulation_automata");
        auto operator_counting = opts.get<shared_ptr<TaskIndependentEvaluator>>(
            "operator_counting", nullptr);
        // The LP and the landmark graph over the original operators, next to the constraints file until they are in it.
        string directory = filesystem::path(options.constraints_path).parent_path().string();
        string prefix = directory.empty() ? "" : directory + "/";
        if (operator_counting)
            options.opc_lp_path = prefix + "operator_counting.lp";
        if (landmark_factory)
            options.landmark_graph_path = prefix + "action_landmark_graph.txt";
        // Two integers that may not fit an int option (the driver's maximum is sys.maxsize); empty: compute them.
        optional<pair<long long, long long>> fixed_plan_length;
        string plan_length = opts.get<string>("plan_length");
        if (!plan_length.empty()) {
            istringstream in(plan_length);
            long long plan_min, plan_max;
            if (!(in >> plan_min >> plan_max))
                throw runtime_error("plan_length must hold the minimum and the maximum plan length");
            fixed_plan_length = make_pair(plan_min, plan_max);
        }
        return components::make_shared_from_arg_tuples<TaskIndependentCCBPAutomata>(
            options, opts.get<bool>("unit_cost"), operator_counting, landmark_factory,
            opts.get<shared_ptr<TaskIndependentEvaluator>>("lower_bound", nullptr),
            opts.get_list<shared_ptr<TaskIndependentSearchAlgorithm>>("lower_bound_searches"),
            opts.get_list<shared_ptr<TaskIndependentSearchAlgorithm>>("upper_bound_searches"),
            opts.get<double>("max_time_for_bounds"), fixed_plan_length,
            get_search_algorithm_arguments_from_options(opts));
    }
};

static plugins::FeaturePlugin<CCBPAutomataFeature> _plugin;
}
