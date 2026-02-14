#include "disj_action_landmark_graph.h"

#include <cassert>
#include <fstream>

using namespace std;

namespace landmarks {
DisjActionLandmarkNode::DisjActionLandmarkNode(std::set<int> actions)
    : actions(std::move(actions)) {
}

bool DisjActionLandmarkNode::overlaps_with(
    DisjActionLandmarkNode &other) const {

    vector<int> intersect;
    set_intersection(actions.begin(), actions.end(),
                     other.actions.begin(), other.actions.end(),
                     back_inserter(intersect));
    return !intersect.empty();
}

bool DisjActionLandmarkNode::satisfied_by(int op_id) const {
    return actions.find(op_id) != actions.end();
}

void DisjActionLandmarkNode::add_strong_dependency(
    int node_id, size_t &num_orderings, size_t &num_weak_orderings) {
    if (dependencies.find(node_id) != dependencies.end()) {
        if (dependencies[node_id] == ActionOrderingType::weak) {
            --num_weak_orderings;
        }
        dependencies[node_id] = ActionOrderingType::strong;
    } else {
        dependencies[node_id] = ActionOrderingType::strong;
        ++num_orderings;
    }
}

void DisjActionLandmarkNode::add_weak_dependency(
    int node_id, size_t &num_orderings, size_t &num_weak_orderings) {
    if (dependencies.find(node_id) == dependencies.end()) {
        dependencies[node_id] = ActionOrderingType::weak;
        ++num_weak_orderings;
        ++num_orderings;
    }
}

ActionOrderingType DisjActionLandmarkNode::get_dependency(int node_id) const {
    assert(depends_on(node_id));
    return dependencies.at(node_id);
}

const map<int, ActionOrderingType> &DisjActionLandmarkNode::get_dependencies() const {
    return dependencies;
}

bool DisjActionLandmarkNode::depends_on(int node_id) const {
    return dependencies.count(node_id);
}

DisjActionLandmarkGraph::DisjActionLandmarkGraph(
    const LandmarkGraph &lm_graph, const State &init_state) {

    vector<size_t> fact_to_action_lm_map = add_nodes(lm_graph, init_state);
    add_edges(lm_graph, init_state, fact_to_action_lm_map);
}

vector<size_t> DisjActionLandmarkGraph::add_nodes(
    const LandmarkGraph &lm_graph, const State &init_state) {
    vector<size_t> fact_to_action_lm_map(lm_graph.get_num_landmarks(), -1);
    for (const std::unique_ptr<LandmarkNode> &node : lm_graph) {
        if (!node->get_landmark().is_true_in_state(init_state) || !node->parents.empty()) {
            std::set<int> sorted_data(node->get_landmark().possible_achievers.begin(), node->get_landmark().possible_achievers.end());
            fact_to_action_lm_map[node->get_id()] = add_node(sorted_data);
        }
    }
    return fact_to_action_lm_map;
}

size_t DisjActionLandmarkGraph::add_node(const set<int> &actions) {
    auto it = ids.find(actions);
    if (it == ids.end()) {
        size_t id = ids.size();
        ids[actions] = id;
        lms.emplace_back(actions);
        return id;
    }
    return it->second;
}

void DisjActionLandmarkGraph::add_edges(const LandmarkGraph &lm_graph,
                                        const State &init_state,
                                        const vector<size_t> &fact_to_action_lm_map) {
    for (const std::unique_ptr<LandmarkNode> &node : lm_graph) {
        if (node->get_landmark().is_true_in_state(init_state)) {
            /* All edges starting in initially true facts are not
               interesting for us since the cycles they possibly induce
               are already resolved initially. */
            continue;
        }
        size_t from_id = fact_to_action_lm_map[node->get_id()];
        assert(utils::in_bounds(from_id, lms));
        for (const auto &child : node->children) {
            LandmarkNode *child_node = child.first;
            OrderingType ord = child.second;
            size_t to_id = fact_to_action_lm_map[child_node->get_id()];
            assert(utils::in_bounds(to_id, lms));
            DisjActionLandmarkNode &dalm_node = lms[to_id];
            /* If there is an action which occurs in both landmarks, applying it
               resolves both landmarks as well as the ordering in one step.
               This special case (which is a consequence of the definition of
               reasonable orderings) makes a lot of things very complicated.
               Ignoring these cases may be desired sometimes which is why we do
               not take them over into our DALM-graph here if the
               *keep_intersecting_orderings* flag is set to false (default). */
            if (ord >= OrderingType::NATURAL) {
                dalm_node.add_strong_dependency(from_id, num_orderings, num_weak_orderings);
            } else if (ord == OrderingType::REASONABLE
                       && !lms[from_id].overlaps_with(dalm_node)) {
                dalm_node.add_weak_dependency(from_id, num_orderings, num_weak_orderings);
            }
        }
    }
}

void DisjActionLandmarkGraph::dump_lm(int id) const {
    cout << "lm" << id << ": <";
    for (int action : lms[id].actions) {
        cout << action << " ";
    }
    cout << ">" << endl;

    if (!lms[id].get_dependencies().empty()) {
        cout << "\tdepends on ";
        for (auto dep : lms[id].get_dependencies()) {
            cout << "lm" << dep.first << "(";
            switch (dep.second) {
            case ActionOrderingType::strong:
                cout << "s";
                break;
            case ActionOrderingType::weak:
                cout << "w";
                break;
            default:
                cout << "?";
                break;
            }
            cout << ") ";
        }
        cout << endl;
    }
}

void DisjActionLandmarkGraph::dump() const {
    cout << "== Disjunctive Action Landmark Graph ==" << endl;
    for (size_t id = 0; id < lms.size(); ++id) {
        dump_lm(id);
    }
    cout << "== End of Graph ==" << endl;
}

void DisjActionLandmarkGraph::dump_dot() const {
    cout << "digraph graphname {\n";
    for (size_t id = 0; id < lms.size(); ++id) {
        cout << "  lm" << id << " [label=\"<";
        for (int a : lms[id].actions) {
            cout << a << " ";
        }
        cout << ">\"];\n";
    }
    cout << "\n";
    for (size_t id = 0; id < lms.size(); ++id) {
        for (pair<int, ActionOrderingType> dep : lms[id].get_dependencies()) {
            cout << "  lm" << dep.first << " -> lm" << id;
            if (dep.second == ActionOrderingType::weak) {
                cout << " [style=dotted]";
            }
            cout << ";\n";
        }
    }
    cout << "}" << endl;
}

void DisjActionLandmarkGraph::dump_graph() const {
    ofstream outFile("action_landmark_graph.txt");
    outFile << "digraph graphname {\n";
    for (size_t id = 0; id < lms.size(); ++id) {
        outFile << "  lm" << id << " [label=\"<";
        for (int a : lms[id].actions) {
            outFile << a << " ";
        }
        outFile << ">\"];\n";
    }
    outFile << "\n";
    for (size_t id = 0; id < lms.size(); ++id) {
        for (pair<int, ActionOrderingType> dep : lms[id].get_dependencies()) {
            outFile << "  lm" << dep.first << " -> lm" << id;
            if (dep.second == ActionOrderingType::weak) {
                outFile << " [style=dotted]";
            }
            outFile << ";\n";
        }
    }
    outFile << "}" << endl;
}

void DisjActionLandmarkGraph::dump_int_graph() const {
    ofstream outFile("action_landmark_graph.txt");
    outFile << lms.size() << endl << endl;

    for (size_t id = 0; id < lms.size(); ++id) {
        outFile << lms[id].actions.size() << "  ";
        for (int a : lms[id].actions) {
            outFile << a << " ";
        }
        outFile << endl;
    }
    outFile << endl;

    size_t total_weak = 0;
    size_t total_strong = 0;

    // Write dependencies with counts
    for (size_t id = 0; id < lms.size(); ++id) {
        outFile << lms[id].get_dependencies().size() << "  ";
        for (pair<int, ActionOrderingType> dep : lms[id].get_dependencies()) {
            outFile << dep.first << " ";
            if (dep.second == ActionOrderingType::weak) {
                outFile << "0  ";
                total_weak++;
            } else {
                outFile << "1  ";
                total_strong++;
            }
        }
        outFile << endl;
    }

    outFile << endl;
    outFile << "Total dependencies - weak: " << total_weak << ", strong: " << total_strong << endl;
    outFile.close();
}

size_t DisjActionLandmarkGraph::get_number_of_landmarks() const {
    return lms.size();
}

ActionOrderingType DisjActionLandmarkGraph::get_ordering_type(int from, int to) {
    assert(0 <= from && from < static_cast<int>(lms.size()));
    assert(0 <= to && to < static_cast<int>(lms.size()));
    assert(lms[to].depends_on(from));
    return lms[to].get_dependency(from);
}

const std::set<int> &DisjActionLandmarkGraph::get_actions(int id) {
    assert(0 <= id && id < static_cast<int>(lms.size()));
    return lms[id].actions;
}

const std::map<int, ActionOrderingType> &DisjActionLandmarkGraph::get_dependencies(int id) {
    assert(0 <= id && id < static_cast<int>(lms.size()));
    return lms[id].get_dependencies();
}

std::vector<std::map<int, bool>> DisjActionLandmarkGraph::to_adj_list() const {
    size_t n = get_number_of_landmarks();
    vector<map<int, bool>> adj(n, map<int, bool>{});
    for (size_t id = 0; id < n; ++id) {
        for (pair<int, ActionOrderingType> dep : lms[id].get_dependencies()) {
            adj[dep.first][id] = dep.second == ActionOrderingType::weak;
        }
    }
    return adj;
}
}
