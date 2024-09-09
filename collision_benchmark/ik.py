import reach_ros
from ebike.ik import AbstractIK, BioIK, RelaxedIK


class DecouplingIK(AbstractIK):
    name = "DecouplingIK"

    def set_config(self, planning_group):
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.kinematics_solver",
            "decoupling_ik/DecouplingIKPlugin",
        )


class RelaxedIKEmptyCostFn(RelaxedIK):
    name = "RelaxedIK (Empty CostFn)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.empty_cost_fn", True)


class RelaxedIKRCM(RelaxedIK):
    name = "RelaxedIK (RCMGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_rcm", True)


class RelaxedIKRCM2(RelaxedIK):
    name = "RelaxedIK (RCMGoal2)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_rcm2", True)


class RelaxedIKRCM3(RelaxedIK):
    name = "RelaxedIK (RCMGoal3)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_rcm3", True)


class RelaxedIKLine(RelaxedIK):
    name = "RelaxedIK (LineGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_line_goal", True)


class RelaxedIKLineAlignment(RelaxedIK):
    name = "RelaxedIK (LineGoal with AlignmentGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_line_goal", True)
        reach_ros.set_parameter("reach_ros.use_line_alignment", True)


class RelaxedIKAlignment(RelaxedIK):
    name = "RelaxedIK (AlignmentGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_line_alignment", True)


class RelaxedIKDepth(RelaxedIK):
    name = "RelaxedIK (DepthGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_depth", True)


class RelaxedIKDepth2(RelaxedIK):
    name = "RelaxedIK (DepthGoal2)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_depth2", True)


class RelaxedIKCollisionDistance(RelaxedIK):
    name = "RelaxedIK (CollisionDistanceGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_collision_distance", True)


class RelaxedIKCollisionDistanceACM(RelaxedIK):
    name = "RelaxedIK (CollisionDistanceGoalACM)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_collision_distance2", True)


class RelaxedIKScan(RelaxedIK):
    name = "RelaxedIKScan"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.scan_goal", True)


class RelaxedIKScanRCM(RelaxedIK):
    name = "RelaxedIKScanRCM"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.scan_goal", True)
        reach_ros.set_parameter("reach_ros.use_rcm", True)


class BioIKNewParams(BioIK):
    name = "BioIK (new parameters)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.population_size2", 4
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.child_count", 32
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.memetic_opt_gens", 16
        )


class BioIKEmptyCostFn(BioIK):
    name = "BioIK (Empty CostFn)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.empty_cost_fn", True)


class BioIKRCM(BioIK):
    name = "BioIK (RCMGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_rcm", True)


class BioIKRCM3(BioIK):
    name = "BioIK (RCMGoal3)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_rcm3", True)


class BioIKLine(BioIK):
    name = "BioIK (LineGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_line_goal", True)


class BioIKLineAlignment(BioIK):
    name = "BioIK (LineGoal with AlignmentGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_line_goal", True)
        reach_ros.set_parameter("reach_ros.use_line_alignment", True)


class BioIKAlignment(BioIK):
    name = "BioIK (AlignmentGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_line_alignment", True)


class BioIKLookAt(BioIK):
    name = "BioIK (LookAtGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_look_at", True)


class BioIKDepth(BioIK):
    name = "BioIK (DepthGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_depth", True)


class BioIKDepth2(BioIK):
    name = "BioIK (DepthGoal2)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_depth2", True)


class BioIKCollisionDistance(BioIK):
    name = "BioIK (CollisionDistanceGoal)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_collision_distance", True)


class BioIKCollisionDistanceACM(BioIK):
    name = "BioIK (CollisionDistanceGoalACM)"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_collision_distance2", True)


class BioIKKeepSeed(BioIK):
    name = "BioIKKeepSeed"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.keep_seed", True
        )
