#ifndef CCBP_PARTITION_H
#define CCBP_PARTITION_H

#include <vector>

namespace ccbp {
/*
  Partition of the values 0..n-1 into blocks of combinable values.

  Mirrors CombinableElementsSets of the Python driver, including its iteration
  order: block ids are assigned by first occurrence while scanning the values
  in `order`, so the blocks (and hence the reduced action ids) come out in the
  same order as in the Python label reduction.
*/
class Partition {
    std::vector<int> order;    // values in iteration order
    std::vector<int> block_of; // value -> block id
    int num_blocks;
    std::vector<int> block_sizes; // block id -> number of values

    void compute_block_sizes();

public:
    explicit Partition(int num_values);

    int size() const {return num_blocks;}
    int num_values() const {return static_cast<int>(block_of.size());}

    // Two values stay together only if they share the same label.
    void refine_by_labels(const std::vector<int> &labels);

    /*
      Values of `values` and values outside it stay apart, as
      CombinableElementsSets.refine: each block met while scanning `values`
      keeps its id for its values outside `values`, and its values inside get
      a new block id, in the order the blocks are met. The block ids of the
      other values do not change.
    */
    void refine(const std::vector<int> &values);

    // labelling()[value] is the id of the block holding value.
    const std::vector<int> &labelling() const {return block_of;}

    /*
      Rename the values with mapping (old value -> new value). Only valid if
      values mapped together share a block, which label reduction guarantees.
    */
    void translate(const std::vector<int> &mapping, int num_new_values);

    // The blocks in block id order, each sorted increasingly.
    std::vector<std::vector<int>> blocks() const;
};
}

#endif
