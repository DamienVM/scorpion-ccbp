#ifndef CCBP_OPERATOR_COUNTING_H
#define CCBP_OPERATOR_COUNTING_H

#include <iosfwd>
#include <string>
#include <vector>

namespace ccbp {
class Partition;

struct OpCountConstraint {
    std::vector<int> pos_actions; // sorted, original operator ids (0-based)
    std::vector<int> neg_actions; // sorted, original operator ids (0-based)
    int rhs = 0;
};

/*
  Operator-counting LP written by Scorpion, parsed as OperatorCountingLP of the
  driver (driver/landmark/operator_counting_lp.py): the constraints of the
  "Subject To ... End" block, possibly over several lines, with terms such as
  "+ x5", "-x11", "2 x3" or "2*x3" (1-based ids). Constraints with only
  positive terms and a right-hand side <= 0 are always satisfied and skipped,
  with the same message as the driver on `log`.
*/
struct OperatorCountingLP {
    std::vector<OpCountConstraint> constraints;
    bool all_unit_coefficients = true;

    // Stable sort by number of actions, as sort_constraints_by_actions.
    void sort_constraints_by_actions();
    void keep_n_constraints(int n);
};

OperatorCountingLP read_operator_counting_lp(std::istream &in, std::ostream &log);

/*
  Partition of the current labels by the operator-counting constraints they
  occur in (OPCRestriction._build_opc_groups): start from one set and refine
  it by the positive and the negative labels of each constraint, in order.
  original_to_label[o] is the current label of the original operator o.
*/
Partition build_operator_counting_partition(
    const OperatorCountingLP &lp, const std::vector<int> &original_to_label, int num_labels);
}

#endif
