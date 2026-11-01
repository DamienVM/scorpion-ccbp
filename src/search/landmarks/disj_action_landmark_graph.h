#ifndef CYCLIC_LANDMARKS_DISJ_ACTION_LANDMARK_GRAPH_H
#define CYCLIC_LANDMARKS_DISJ_ACTION_LANDMARK_GRAPH_H

#include "landmark_graph.h"

#include <map>
#include <string>
#include <set>
#include <vector>

namespace landmarks {

enum class ActionOrderingType {
    strong,
    weak,
};

class DisjActionLandmarkNode {
    std::map<int, ActionOrderingType> dependencies;

public:
    const std::set<int> actions;

    DisjActionLandmarkNode(std::set<int> actions);
    bool overlaps_with(DisjActionLandmarkNode &other) const;
    bool satisfied_by(int op_id) const;
    void add_strong_dependency(int node_id, size_t &num_orderings, size_t &num_weak_orderings);
    void add_weak_dependency(int node_id, size_t &num_orderings, size_t &num_weak_orderings);
    ActionOrderingType get_dependency(int id) const;
    const std::map<int, ActionOrderingType> &get_dependencies() const;
    bool depends_on(int id) const;
};

class DisjActionLandmarkGraph {
    std::map<std::set<int>, size_t> ids;
    std::vector<DisjActionLandmarkNode> lms;
    size_t num_orderings = 0;
    size_t num_weak_orderings = 0;

    std::vector<size_t> add_nodes(const LandmarkGraph &lm_graph,
                                  const State &init_state);
    size_t add_node(const std::set<int> &actions);
    void add_edges(const LandmarkGraph &lm_graph,
                   const State &init_state,
                   const std::vector<size_t> &fact_to_action_lm_map);
    void dump_lm(int id) const;
public:
    explicit DisjActionLandmarkGraph(const LandmarkGraph &lm_graph,
                                     const State &init_state);
    size_t get_number_of_landmarks() const;
    size_t get_number_of_orderings() const {
        return num_orderings;
    }
    size_t get_number_of_weak_orderings() const {
        return num_weak_orderings;
    }
    size_t get_number_of_strong_orderings() const {
        return num_orderings - num_weak_orderings;
    }
    const std::set<int> &get_actions(int id);
    const std::map<int, ActionOrderingType> &get_dependencies(int id);
    ActionOrderingType get_ordering_type(int from, int to);
    void dump() const;
    void dump_dot() const;

    std::vector<std::map<int, bool>> to_adj_list() const;

    void dump_graph() const;

    void dump_int_graph(const std::string &path) const;
};
}

#endif
