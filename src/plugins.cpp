#include "reachability_evaluator.hpp"
#include "benchmark_moveit_ik_solver.hpp"
#include <reach/plugin_utils.h>
EXPORT_IK_SOLVER_PLUGIN(benchmark::MoveItIKSolverFactory, MoveItIKSolver2)
EXPORT_EVALUATOR_PLUGIN(benchmark::ReachabilityEvaluatorFactory, ReachabilityEvaluator)
