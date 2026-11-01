#include "remap.h"

#include "operator_counting.h"
#include "output.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>

using namespace std;

namespace ccbp {
namespace {
// Labels of original operators, nothing if one of them is out of range (to_labels of the former driver).
optional<vector<int>> to_labels(const vector<int> &actions, const vector<int> &original_to_label) {
    vector<int> labels;
    for (int action : actions) {
        if (action < 0 || action >= static_cast<int>(original_to_label.size()))
            return nullopt;
        labels.push_back(original_to_label[action]);
    }
    sort(labels.begin(), labels.end());
    labels.erase(unique(labels.begin(), labels.end()), labels.end());
    return labels;
}

struct Entry {
    vector<int> positive_labels;
    vector<int> negative_labels;
    int rhs;
};

void write_operator_counting_lp(const string &path, const vector<Entry> &entries, const string &header) {
    ofstream out(path);
    if (!out)
        throw runtime_error("Cannot open " + path);
    out << "\\ " << header << '\n' << "Subject To\n";
    for (size_t index = 0; index < entries.size(); ++index) {
        // LP variable index is 1-based (the parser reads x_k as action k-1), so emit label + 1.
        string positive;
        for (int label : entries[index].positive_labels)
            positive += (positive.empty() ? "" : " + ") + ("x" + to_string(label + 1));
        string negative;
        for (int label : entries[index].negative_labels)
            negative += (negative.empty() ? "" : " ") + ("- x" + to_string(label + 1));
        string terms = positive.empty() ? negative : (negative.empty() ? positive : positive + " " + negative);
        out << 'c' << index << ": " << terms << " >= " << entries[index].rhs << '\n';
    }
    out << "End\n";
}

bool disjoint(const vector<int> &a, const set<int> &b) {
    return none_of(a.begin(), a.end(), [&](int value) {return b.count(value) > 0;});
}

void remap_operator_counting_lp(const string &path, const vector<int> &original_to_label) {
    ifstream in(path);
    if (!in)
        throw runtime_error("operator-counting LP '" + path + "' does not exist");
    OperatorCountingLP lp = read_operator_counting_lp(in, cout);
    in.close();
    if (!lp.all_unit_coefficients)
        throw runtime_error("operator-counting LP '" + path + "' has non-unit coefficients, which are not supported.");

    bool only_positive = all_of(lp.constraints.begin(), lp.constraints.end(),
                                [](const OpCountConstraint &c) {return c.neg_actions.empty();});
    vector<Entry> entries;
    if (only_positive) {
        // Buckets of the constraints with the same labels, in order of first occurrence.
        map<vector<int>, int> bucket_ids;
        vector<pair<vector<int>, vector<const OpCountConstraint *>>> buckets;
        for (const OpCountConstraint &constraint : lp.constraints) {
            optional<vector<int>> labels = to_labels(constraint.pos_actions, original_to_label);
            if (!labels)
                continue;
            auto [it, inserted] = bucket_ids.try_emplace(*labels, static_cast<int>(buckets.size()));
            if (inserted)
                buckets.emplace_back(*labels, vector<const OpCountConstraint *>());
            buckets[it->second].second.push_back(&constraint);
        }
        for (auto &[labels, constraints] : buckets) {
            // Greedy set packing, largest right-hand side first (stable, as list.sort).
            stable_sort(constraints.begin(), constraints.end(), [](const OpCountConstraint *a, const OpCountConstraint *b) {
                if (a->rhs != b->rhs)
                    return a->rhs > b->rhs;
                return a->pos_actions.size() < b->pos_actions.size();
            });
            set<int> covered;
            int rhs = 0;
            for (const OpCountConstraint *constraint : constraints) {
                if (disjoint(constraint->pos_actions, covered)) {
                    covered.insert(constraint->pos_actions.begin(), constraint->pos_actions.end());
                    rhs += constraint->rhs;
                }
            }
            entries.push_back({labels, {}, rhs});
        }
        write_operator_counting_lp(path, entries, "Positive operator-counting constraints remapped to reduced labels");
    } else {
        for (const OpCountConstraint &constraint : lp.constraints) {
            optional<vector<int>> positive = to_labels(constraint.pos_actions, original_to_label);
            optional<vector<int>> negative = to_labels(constraint.neg_actions, original_to_label);
            if (!positive || !negative)
                continue;
            // Max coefficient per label: a producer in any member outranks a consumer.
            vector<int> negative_only;
            set_difference(negative->begin(), negative->end(), positive->begin(), positive->end(),
                           back_inserter(negative_only));
            if (positive->empty() && negative_only.empty())
                continue;
            if (negative_only.empty() && constraint.rhs <= 0)
                continue;
            entries.push_back({*positive, negative_only, constraint.rhs});
        }
        write_operator_counting_lp(path, entries, "Operator-counting constraints remapped to reduced labels");
    }
}

void remap_action_landmark_graph(const string &path, const vector<int> &original_to_label) {
    ifstream in(path);
    if (!in)
        throw runtime_error("action landmark graph '" + path + "' does not exist");
    int num_landmarks;
    in >> num_landmarks;
    vector<vector<int>> landmark_actions(num_landmarks);
    for (vector<int> &actions : landmark_actions) {
        int num_actions;
        in >> num_actions;
        actions.resize(num_actions);
        for (int &action : actions)
            in >> action;
        sort(actions.begin(), actions.end());
        actions.erase(unique(actions.begin(), actions.end()), actions.end());
    }
    // Precedence edges of each landmark (from, strong), without duplicates.
    vector<set<pair<int, bool>>> precedence(num_landmarks);
    for (int to = 0; to < num_landmarks; ++to) {
        int num_edges;
        in >> num_edges;
        for (int i = 0; i < num_edges; ++i) {
            int from, strength;
            in >> from >> strength;
            precedence[to].emplace(from, strength == 1);
        }
    }
    if (!in)
        throw runtime_error("Cannot read the action landmark graph '" + path + "'");
    in.close();

    // Number of original operators merged into each label.
    map<int, int> label_size;
    for (int label : original_to_label)
        ++label_size[label];

    vector<vector<int>> new_actions;
    vector<bool> is_exact;
    vector<int> old_to_new(num_landmarks, -1);
    for (int landmark = 0; landmark < num_landmarks; ++landmark) {
        optional<vector<int>> labels = to_labels(landmark_actions[landmark], original_to_label);
        if (!labels || labels->empty())
            continue;
        old_to_new[landmark] = new_actions.size();
        size_t covered = 0;
        for (int label : *labels)
            covered += label_size[label];
        is_exact.push_back(covered == landmark_actions[landmark].size());
        new_actions.push_back(*labels);
    }

    // A strong edge needs an exact target, a weak edge always survives.
    vector<vector<pair<int, int>>> new_precedence(new_actions.size());
    int kept_strong = 0;
    int kept_weak = 0;
    for (int to = 0; to < num_landmarks; ++to) {
        for (auto [from, strong] : precedence[to]) {
            int new_from = old_to_new[from];
            int new_to = old_to_new[to];
            if (new_from < 0 || new_to < 0 || (strong && !is_exact[new_to]))
                continue;
            new_precedence[new_to].emplace_back(new_from, strong ? 1 : 0);
            if (strong)
                ++kept_strong;
            else
                ++kept_weak;
        }
    }

    ofstream out(path);
    if (!out)
        throw runtime_error("Cannot open " + path);
    out << new_actions.size() << "\n\n";
    for (const vector<int> &labels : new_actions) {
        out << labels.size() << ' ';
        for (size_t i = 0; i < labels.size(); ++i)
            out << (i > 0 ? " " : "") << labels[i];
        out << '\n';
    }
    out << '\n';
    for (vector<pair<int, int>> &edges : new_precedence) {
        sort(edges.begin(), edges.end());
        out << edges.size();
        for (auto [from, strength] : edges)
            out << ' ' << from << ' ' << strength;
        out << '\n';
    }
    out << '\n' << "Total dependencies - weak: " << kept_weak << ", strong: " << kept_strong << '\n';
}
}

void remap_constraints(const vector<int> &original_to_label, int num_labels,
                       const string &operator_counting_lp_path, const string &action_landmark_graph_path) {
    bool identity = true;
    for (size_t i = 0; i < original_to_label.size(); ++i) {
        if (original_to_label[i] != static_cast<int>(i)) {
            identity = false;
            break;
        }
    }
    if (identity)
        return;

    print_info_block({"Remapping constraints to reduced labels",
                      "original operators: " + to_string(original_to_label.size()) +
                      " -> reduced labels: " + to_string(num_labels)},
                     true, false);
    if (!operator_counting_lp_path.empty())
        remap_operator_counting_lp(operator_counting_lp_path, original_to_label);
    if (!action_landmark_graph_path.empty())
        remap_action_landmark_graph(action_landmark_graph_path, original_to_label);
    print_info_block({"Constraint remapping finished"}, false, true);
}
}
