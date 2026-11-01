#ifndef CCBP_REMAP_H
#define CCBP_REMAP_H

#include <string>
#include <vector>

namespace ccbp {
/*
  remap_constraints_for_reduction of the former driver/label_reduction/constraint_remap.py:
  rewrite the operator-counting LP and the action landmark graph, written over
  the original operators, over the reduced labels. Nothing is done when no
  action was reduced. Same files and messages as the driver.
*/
void remap_constraints(const std::vector<int> &original_to_label, int num_labels,
                       const std::string &operator_counting_lp_path,
                       const std::string &action_landmark_graph_path);
}

#endif
