# ccbp_automata

Scorpion component of CCBP: everything between the translation of the task
and the CP solver, in the same Scorpion run. `ccbp.py` builds the call from
its arguments (`driver/generateCPAutomata.py`); the options have the names of
the driver arguments. It replaces the former `tools/refinement` tool and the
separate Scorpion runs for the bounds, the operator-counting LP and the
landmark graph.

## Pipeline

1. Operator-counting LP of the initial state (`operator_counting`, for the
   `lropc` strategies and the posted constraints) and action landmark graph
   (`landmarks`), over the original operators.
2. Atomic automata, one per variable (Scorpion's atomic transition systems,
   the locally equivalent labels of the same cost grouped into one local
   action); their unreachable and dead-end states are removed.
3. Consecutive actions relations, mutex pruning (`remove_mutex_transitions`),
   `unit_cost`, equivalent actions (`remove_equivalent_actions`); the
   simulation automata used to translate the plans are written here.
4. Merges (`merge_min`, `merge_objects`): synchronized products restricted to
   their reachable states whose facts are not mutex, then without their
   dead-end states and, with `shrink`, their bisimilar states
   (exact bisimulation). The facts of a merged state are the facts common to
   its states. With `merge_max_states`, a product is abandoned as soon as it
   has more reachable states: `merge_min` stops merging, `merge_objects`
   skips the merge of that object.
5. Label reduction, deterministic (`label_reduction`) or not
   (`nd_label_reduction`), then shrinking; with `lr_fix_point`, they
   alternate until neither changes anything. A reduced label never groups
   actions of different costs.
6. No-op (`no_op`, `no_op_2`, deterministic or not), then the outputs.
7. Plan length bounds, appended to the constraints file: first without search
   (the `lower_bound` heuristic in the initial state, the shortest path to a
   goal in each automaton), then the `lower_bound_searches` and
   `upper_bound_searches` within `max_time_for_bounds` seconds (the first
   upper bound search has no time limit), appended again only if the searches
   improved them. `plan_length` gives them instead.

## Outputs

- `cp_automata`: the CP automata (`driver/automata/cp_automata_format.txt`),
  without the plan length bounds.
- `constraints`: the constraints of the CP model over the labels of the CP
  automata, read by CP-SAT (`driver/cp_sat/constraints_file.py`):

  ```
  ccbp_constraints 1
  num_labels <number of labels of the CP automata>
  begin_operator_counting      (posted operator-counting constraints, LP format)
  ...
  end_operator_counting
  begin_landmarks              (action landmark graph)
  ...
  end_landmarks
  begin_successors             (--mutex-actions-constraints)
  <#labels> <#pairs that may follow each other> <#forbidden pairs>
  <l1> <l2>                    (l2 may not follow l1)
  end_successors
  begin_opposite_pairs         (--consecutive-actions-constraints opposite)
  <#pairs>
  <l1> <l2>                    (l1 and l2 may not follow each other, in either order)
  end_opposite_pairs
  begin_bounds                 (appended last; the last bounds section counts)
  plan_min <min>
  plan_max <max>
  end_bounds
  ```

  A missing section means that the constraint is not used. The successors and
  opposite pairs are computed over the original operators (and the no-op), then
  translated to the labels: a pair of labels is forbidden only if all the pairs
  of their operators are. The pairs that the CP automata already rule out (in
  some automaton, no state reached by the first label has a transition for the
  second) are left out.
- `reductions`: `reductions.yaml`, the original operators of each label.
- `simulation_automata`: the automata used to translate the plans.

## Exit codes

0 when the files are written; 10 when an `exit_after` step is reached; 22
(Scorpion's out of memory) means the bounds written so far are kept when the
constraints file has a bounds section (a bound search ran out of memory); 32
on errors (bad input, unsolvable task).

## Implementation

- `ccbp_automata.*`: the component, the atomic automata, the LP and landmark
  graph, the bounds.
- `pipeline.*`: the refinement stage and the constraints file.
- `preprocess.*`: the facts of the operators, mutex pruning (with the mutexes
  of Scorpion's task), local actions, no-op, conversions between
  deterministic and non-deterministic automata.
- `merge.*`: facts relations, synchronized products, pruning and shrinking of
  the merged automata, `merge_min`, `merge_objects`.
- `partition.*`, `reduction_loop.h`: combinable sets of actions and the label
  reduction loop (prefix snapshots and a running suffix over the automata).
- `deterministic_sets.*`, `label_reduction.*`: deterministic label reduction
  (conflict graphs and greedy maximal independent sets).
- `nd_automata.*`, `nd_label_reduction.*`: non-deterministic automata, their
  label reduction, pruning and exact bisimulation shrinking.
- `operator_counting.*`, `remap.*`: the operator-counting LP and partition,
  the LP and landmark graph over the reduced labels.
- `output.*`: log helpers, `reductions.yaml`, CP automata writers.

`scripts/validate_pipeline.py` runs CCBP on small instances over every
option, checks the plans with VAL and compares runs.
