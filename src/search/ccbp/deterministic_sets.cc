#include "deterministic_sets.h"

#include "automata.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <map>
#include <queue>
#include <tuple>
#include <utility>

using namespace std;

namespace ccbp {
namespace {
/*
  n x n bit matrix of a relation (between local slots, or between vertices).
  Rows are updated with whole words, so the pairwise work is done 64 elements
  at a time.
*/
class BitMatrix {
    int num_words;
    vector<uint64_t> bits;

public:
    explicit BitMatrix(int size)
        : num_words((size + 63) / 64),
          bits(static_cast<size_t>(size) * num_words, 0) {
    }

    int words() const {return num_words;}
    uint64_t *row(int i) {return bits.data() + static_cast<size_t>(i) * num_words;}
    const uint64_t *row(int i) const {return bits.data() + static_cast<size_t>(i) * num_words;}

    bool test(int i, int j) const {
        return (row(i)[j >> 6] >> (j & 63)) & 1;
    }
};

void set_bit(vector<uint64_t> &bitset, int i) {
    bitset[i >> 6] |= uint64_t(1) << (i & 63);
}

/*
  Bit j of row i is set iff local slots i and j are NOT deterministically
  reducible in theta: some state where both are applicable leads to different
  targets, or theta has local costs and theirs differ.
*/
BitMatrix compute_incompatibility(
    const Automaton &theta, const LocalTransitions &local_transitions,
    const vector<int> &slot_locals) {
    int k = slot_locals.size();
    BitMatrix incompat(k);
    int words = incompat.words();
    vector<uint64_t> active(words, 0);
    vector<uint64_t> bucket(words, 0);

    struct Entry {
        int state;
        int target;
        int slot;
        bool operator<(const Entry &other) const {
            return tie(state, target, slot) < tie(other.state, other.target, other.slot);
        }
    };
    vector<Entry> entries;
    for (int slot = 0; slot < k; ++slot) {
        for (auto [state, target] : local_transitions[slot_locals[slot]])
            entries.push_back({state, target, slot});
    }
    sort(entries.begin(), entries.end());

    for (size_t state_begin = 0; state_begin < entries.size();) {
        size_t state_end = state_begin;
        while (state_end < entries.size() && entries[state_end].state == entries[state_begin].state)
            ++state_end;
        // Skip states where all applicable actions go to the same target.
        if (entries[state_begin].target == entries[state_end - 1].target) {
            state_begin = state_end;
            continue;
        }

        for (size_t i = state_begin; i < state_end; ++i)
            set_bit(active, entries[i].slot);
        // An action conflicts with every action applicable here that goes elsewhere.
        for (size_t begin = state_begin; begin < state_end;) {
            size_t end = begin;
            while (end < state_end && entries[end].target == entries[begin].target) {
                set_bit(bucket, entries[end].slot);
                ++end;
            }
            for (size_t i = begin; i < end; ++i) {
                uint64_t *row = incompat.row(entries[i].slot);
                for (int w = 0; w < words; ++w)
                    row[w] |= active[w] & ~bucket[w];
            }
            for (size_t i = begin; i < end; ++i)
                bucket[entries[i].slot >> 6] = 0;
            begin = end;
        }
        for (size_t i = state_begin; i < state_end; ++i)
            active[entries[i].slot >> 6] = 0;
        state_begin = state_end;
    }

    if (theta.has_costs) {
        map<int, vector<uint64_t>> by_cost;
        for (int slot = 0; slot < k; ++slot) {
            auto [it, inserted] = by_cost.try_emplace(
                theta.local_costs[slot_locals[slot]], words, 0);
            set_bit(it->second, slot);
        }
        if (by_cost.size() > 1) {
            for (int slot = 0; slot < k; ++slot) {
                const vector<uint64_t> &same_cost = by_cost[theta.local_costs[slot_locals[slot]]];
                uint64_t *row = incompat.row(slot);
                for (int w = 0; w < words; ++w)
                    row[w] |= ~same_cost[w];
            }
            // Clear the padding bits beyond the last slot.
            if (k % 64 != 0) {
                uint64_t mask = (uint64_t(1) << (k % 64)) - 1;
                for (int slot = 0; slot < k; ++slot)
                    incompat.row(slot)[words - 1] &= mask;
            }
        }
    }
    return incompat;
}

/*
  Bit j of row i is set iff local slots i and j have at least one successor
  state in common. Bit i of row i is set iff slot i has a successor.
*/
BitMatrix compute_shared_successors(
    const LocalTransitions &local_transitions, const vector<int> &slot_locals) {
    int k = slot_locals.size();
    BitMatrix shared(k);
    int words = shared.words();
    vector<pair<int, int>> reaching; // (target, slot)
    for (int slot = 0; slot < k; ++slot) {
        for (auto [state, target] : local_transitions[slot_locals[slot]])
            reaching.emplace_back(target, slot);
    }
    sort(reaching.begin(), reaching.end());
    reaching.erase(unique(reaching.begin(), reaching.end()), reaching.end());

    vector<uint64_t> mask(words, 0);
    for (size_t begin = 0; begin < reaching.size();) {
        size_t end = begin;
        while (end < reaching.size() && reaching[end].first == reaching[begin].first) {
            set_bit(mask, reaching[end].second);
            ++end;
        }
        for (size_t i = begin; i < end; ++i) {
            uint64_t *row = shared.row(reaching[i].second);
            for (int w = 0; w < words; ++w)
                row[w] |= mask[w];
        }
        for (size_t i = begin; i < end; ++i)
            mask[reaching[i].second >> 6] = 0;
        begin = end;
    }
    return shared;
}

/*
  Greedy maximal independent sets, as in GreedyLabelReduction of the Python
  driver: within a round, repeatedly pick the vertex with the smallest virtual
  degree (ties: smallest index), then virtually remove it and its neighbours.
  The picked set is then removed from the graph and the next round starts.
*/
vector<vector<int>> greedy_independent_sets(const vector<vector<int>> &adjacency) {
    int num_vertices = adjacency.size();
    vector<vector<int>> sets;
    vector<bool> removed(num_vertices, false);
    vector<int> degree(num_vertices);
    for (int v = 0; v < num_vertices; ++v)
        degree[v] = adjacency[v].size();
    vector<int> virtual_degree(num_vertices);

    using Entry = pair<int, int>; // (virtual degree, vertex)
    int num_remaining = num_vertices;
    while (num_remaining > 0) {
        priority_queue<Entry, vector<Entry>, greater<Entry>> queue;
        for (int v = 0; v < num_vertices; ++v) {
            if (removed[v]) {
                virtual_degree[v] = -1;
            } else {
                virtual_degree[v] = degree[v];
                queue.emplace(degree[v], v);
            }
        }

        auto virtual_remove = [&](int v) {
            virtual_degree[v] = -1;
            for (int n : adjacency[v]) {
                if (!removed[n] && virtual_degree[n] >= 0) {
                    --virtual_degree[n];
                    queue.emplace(virtual_degree[n], n);
                }
            }
        };

        vector<int> independent_set;
        while (!queue.empty()) {
            auto [d, v] = queue.top();
            queue.pop();
            // Virtual degrees only decrease, so an entry is current iff it matches.
            if (virtual_degree[v] != d)
                continue;
            independent_set.push_back(v);
            virtual_remove(v);
            for (int n : adjacency[v]) {
                if (!removed[n] && virtual_degree[n] >= 0)
                    virtual_remove(n);
            }
        }

        for (int v : independent_set) {
            removed[v] = true;
            --num_remaining;
            for (int n : adjacency[v]) {
                if (!removed[n])
                    --degree[n];
            }
        }
        sets.push_back(move(independent_set));
    }
    return sets;
}

/*
  Same greedy as greedy_independent_sets (same picks, same order), on a bit
  matrix adjacency. Used for dense graphs, where the adjacency lists version
  touches almost every edge for each of its many rounds: here a pick costs
  O(#vertices still available x #vertices / 64) whatever the density.
*/
vector<vector<int>> greedy_independent_sets_dense(const BitMatrix &adjacency, int num_vertices) {
    int words = adjacency.words();
    vector<vector<int>> sets;
    vector<uint64_t> remaining(words, 0);
    for (int v = 0; v < num_vertices; ++v)
        set_bit(remaining, v);
    int num_remaining = num_vertices;
    vector<int> virtual_degree(num_vertices);
    vector<int> available_vertices;
    vector<uint64_t> available(words);
    vector<uint64_t> removed_now(words);

    auto count_common = [words](const uint64_t *a, const uint64_t *b) {
        int count = 0;
        for (int w = 0; w < words; ++w)
            count += __builtin_popcountll(a[w] & b[w]);
        return count;
    };

    while (num_remaining > 0) {
        available = remaining;
        available_vertices.clear();
        for (int v = 0; v < num_vertices; ++v) {
            if ((remaining[v >> 6] >> (v & 63)) & 1) {
                available_vertices.push_back(v);
                virtual_degree[v] = count_common(adjacency.row(v), remaining.data());
            }
        }

        vector<int> independent_set;
        while (!available_vertices.empty()) {
            // Smallest virtual degree, smallest vertex on ties (vertices are increasing).
            int picked = available_vertices[0];
            for (int v : available_vertices) {
                if (virtual_degree[v] < virtual_degree[picked])
                    picked = v;
            }
            independent_set.push_back(picked);

            // Virtually remove the picked vertex and its available neighbours.
            const uint64_t *row = adjacency.row(picked);
            for (int w = 0; w < words; ++w) {
                removed_now[w] = available[w] & row[w];
                available[w] &= ~row[w];
            }
            set_bit(removed_now, picked);
            available[picked >> 6] &= ~(uint64_t(1) << (picked & 63));

            size_t num_kept = 0;
            for (int v : available_vertices) {
                if ((available[v >> 6] >> (v & 63)) & 1) {
                    virtual_degree[v] -= count_common(adjacency.row(v), removed_now.data());
                    available_vertices[num_kept++] = v;
                }
            }
            available_vertices.resize(num_kept);
        }

        for (int v : independent_set) {
            remaining[v >> 6] &= ~(uint64_t(1) << (v & 63));
            --num_remaining;
        }
        sets.push_back(move(independent_set));
    }
    return sets;
}
}

LocalTransitions compute_local_transitions(const Automaton &automaton) {
    LocalTransitions local_transitions(automaton.num_local_actions);
    for (int state = 0; state < automaton.num_states; ++state) {
        for (int local = 0; local < automaton.num_local_actions; ++local) {
            int target = automaton.target(state, local);
            if (target != -1)
                local_transitions[local].emplace_back(state, target);
        }
    }
    return local_transitions;
}

vector<vector<int>> compute_deterministic_sets(
    const Automaton &theta, const LocalTransitions &local_transitions,
    const vector<int> &block, ConflictRule rule) {
    if (block.size() < 2)
        return {block};

    /*
      The tests only depend on the local action of each global action, so
      group the global actions by local action ("slots") and decide each pair
      of slots once. The scratch map is reused across calls because there can
      be many blocks and many local actions.
    */
    static vector<int> slot_of_local;
    if (static_cast<int>(slot_of_local.size()) < theta.num_local_actions)
        slot_of_local.resize(theta.num_local_actions, -1);
    vector<int> slot_locals;
    vector<vector<int>> slot_members; // vertex indices (positions in block)
    for (size_t vertex = 0; vertex < block.size(); ++vertex) {
        int local = theta.local_mapping[block[vertex]];
        int &slot = slot_of_local[local];
        if (slot == -1) {
            slot = slot_locals.size();
            slot_locals.push_back(local);
            slot_members.emplace_back();
        }
        slot_members[slot].push_back(vertex);
    }
    for (int local : slot_locals)
        slot_of_local[local] = -1;

    int k = slot_locals.size();
    BitMatrix conflicts = compute_incompatibility(theta, local_transitions, slot_locals);
    if (rule != ConflictRule::GREEDY) {
        BitMatrix shared = compute_shared_successors(local_transitions, slot_locals);
        for (int i = 0; i < k; ++i) {
            uint64_t *row = conflicts.row(i);
            const uint64_t *shared_row = shared.row(i);
            for (int w = 0; w < conflicts.words(); ++w) {
                if (rule == ConflictRule::SSS)
                    row[w] |= ~shared_row[w];
                else
                    row[w] |= shared_row[w];
            }
            // SSS: a slot never conflicts with itself; DSS: it always does.
            if (rule == ConflictRule::SSS)
                row[i >> 6] &= ~(uint64_t(1) << (i & 63));
            else
                row[i >> 6] |= uint64_t(1) << (i & 63);
            if (k % 64 != 0)
                row[conflicts.words() - 1] &= (uint64_t(1) << (k % 64)) - 1;
        }
    }

    int num_vertices = block.size();
    long long num_edges = 0;
    for (int s1 = 0; s1 < k; ++s1) {
        long long size1 = slot_members[s1].size();
        if (conflicts.test(s1, s1))
            num_edges += size1 * (size1 - 1) / 2;
        for (int s2 = s1 + 1; s2 < k; ++s2) {
            if (conflicts.test(s1, s2))
                num_edges += size1 * static_cast<long long>(slot_members[s2].size());
        }
    }

    vector<vector<int>> sets;
    // Dense: average degree above #vertices / 8.
    if (16 * num_edges > static_cast<long long>(num_vertices) * num_vertices) {
        BitMatrix slot_masks(num_vertices); // rows: slots (k <= #vertices)
        for (int slot = 0; slot < k; ++slot) {
            for (int v : slot_members[slot])
                slot_masks.row(slot)[v >> 6] |= uint64_t(1) << (v & 63);
        }
        BitMatrix adjacency(num_vertices);
        int words = adjacency.words();
        for (int s1 = 0; s1 < k; ++s1) {
            vector<uint64_t> neighbours(words, 0);
            for (int s2 = 0; s2 < k; ++s2) {
                if (conflicts.test(s1, s2)) {
                    const uint64_t *mask = slot_masks.row(s2);
                    for (int w = 0; w < words; ++w)
                        neighbours[w] |= mask[w];
                }
            }
            for (int v : slot_members[s1]) {
                uint64_t *row = adjacency.row(v);
                for (int w = 0; w < words; ++w)
                    row[w] = neighbours[w];
                row[v >> 6] &= ~(uint64_t(1) << (v & 63));
            }
        }
        sets = greedy_independent_sets_dense(adjacency, num_vertices);
    } else {
        vector<vector<int>> adjacency(num_vertices);
        for (int s1 = 0; s1 < k; ++s1) {
            const vector<int> &group1 = slot_members[s1];
            for (int s2 = s1; s2 < k; ++s2) {
                if (!conflicts.test(s1, s2))
                    continue;
                if (s1 == s2) {
                    for (size_t i = 0; i + 1 < group1.size(); ++i) {
                        for (size_t j = i + 1; j < group1.size(); ++j) {
                            adjacency[group1[i]].push_back(group1[j]);
                            adjacency[group1[j]].push_back(group1[i]);
                        }
                    }
                } else {
                    for (int v1 : group1) {
                        for (int v2 : slot_members[s2]) {
                            adjacency[v1].push_back(v2);
                            adjacency[v2].push_back(v1);
                        }
                    }
                }
            }
        }
        sets = greedy_independent_sets(adjacency);
    }

    for (vector<int> &set : sets) {
        for (int &vertex : set)
            vertex = block[vertex];
        sort(set.begin(), set.end());
    }
    return sets;
}
}
