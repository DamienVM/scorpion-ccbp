#include "partition.h"

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <unordered_map>

using namespace std;

namespace ccbp {
Partition::Partition(int num_values)
    : order(num_values), block_of(num_values, 0), num_blocks(num_values > 0 ? 1 : 0) {
    iota(order.begin(), order.end(), 0);
    if (num_values > 0)
        block_sizes.push_back(num_values);
}

void Partition::compute_block_sizes() {
    block_sizes.assign(num_blocks, 0);
    for (int value : order)
        ++block_sizes[block_of[value]];
}

void Partition::refine_by_labels(const vector<int> &labels) {
    unordered_map<uint64_t, int> key_to_new_block;
    key_to_new_block.reserve(2 * static_cast<size_t>(num_blocks) + 16);
    for (int value : order) {
        uint64_t key = (static_cast<uint64_t>(block_of[value]) << 32) |
                       static_cast<uint32_t>(labels[value]);
        auto [it, inserted] = key_to_new_block.try_emplace(
            key, static_cast<int>(key_to_new_block.size()));
        block_of[value] = it->second;
    }
    num_blocks = static_cast<int>(key_to_new_block.size());
    compute_block_sizes();
}

void Partition::refine(const vector<int> &values) {
    // Blocks in the order they are met, with their values from `values`.
    vector<int> met_blocks;
    unordered_map<int, int> block_to_bucket;
    vector<vector<int>> buckets;
    for (int value : values) {
        auto [it, inserted] = block_to_bucket.try_emplace(
            block_of[value], static_cast<int>(buckets.size()));
        if (inserted) {
            met_blocks.push_back(block_of[value]);
            buckets.emplace_back();
        }
        buckets[it->second].push_back(value);
    }
    for (size_t b = 0; b < buckets.size(); ++b) {
        int block = met_blocks[b];
        int bucket_size = buckets[b].size();
        if (bucket_size == block_sizes[block])
            continue;
        int new_block = num_blocks++;
        for (int value : buckets[b])
            block_of[value] = new_block;
        block_sizes[block] -= bucket_size;
        block_sizes.push_back(bucket_size);
    }
}

void Partition::translate(const vector<int> &mapping, int num_new_values) {
    vector<int> new_order;
    new_order.reserve(num_new_values);
    vector<int> new_block_of(num_new_values, -1);
    for (int value : order) {
        int new_value = mapping[value];
        if (new_block_of[new_value] == -1) {
            new_order.push_back(new_value);
        }
        new_block_of[new_value] = block_of[value];
    }
    order = move(new_order);
    block_of = move(new_block_of);
    compute_block_sizes();
}

vector<vector<int>> Partition::blocks() const {
    vector<vector<int>> result(num_blocks);
    for (int value = 0; value < num_values(); ++value) {
        result[block_of[value]].push_back(value);
    }
    return result;
}
}
