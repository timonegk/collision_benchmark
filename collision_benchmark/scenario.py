from ebike.scenario import AbstractScenario


class HydrogenTank(AbstractScenario):
    name = 'hydrogen_tank'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
    hole_position = [0.0, 1.0, 0.5]
    hole_axis = [0, 1, 0]


class HydrogenTankSeed(HydrogenTank):
    name = 'hydrogen_tank_seed'
    def get_config(self, *args):
        config = super().get_config(*args)
        config["optimization"]["seed_state"] = [
                {"name": "shoulder_pan_joint", "position": 1.134},
                {"name": "shoulder_lift_joint", "position": -1.478},
                {"name": "elbow_joint", "position": 2.438},
                {"name": "wrist_1_joint", "position": 2.070},
                {"name": "wrist_2_joint", "position": -1.138},
                {"name": "wrist_3_joint", "position": 0.0},
                {"name": "endo_first_joint", "position": 0.0},
                {"name": "endo_second_joint_first_dof", "position": 0.0},
                {"name": "endo_second_joint_second_dof", "position": 0.0},
        ]
        return config


class HydrogenTankSeed2(HydrogenTank):
    name = 'hydrogen_tank_seed2'
    def get_config(self, *args):
        config = super().get_config(*args)
        config["optimization"]["seed_state"] = [
                {"name": "shoulder_pan_joint", "position": 1.134},
                {"name": "shoulder_lift_joint", "position": -1.478},
                {"name": "elbow_joint", "position": 2.438},
                {"name": "wrist_1_joint", "position": 2.070},
                {"name": "wrist_2_joint", "position": -1.138},
                {"name": "wrist_3_joint", "position": 3.143},
                {"name": "endo_first_joint", "position": -0.288},
                {"name": "endo_second_joint_first_dof", "position": 0.125},
                {"name": "endo_second_joint_second_dof", "position": -1.386},
        ]
        return config


class HydrogenTankSmall(HydrogenTank):
    name = 'hydrogen_tank_small'
    ply_file = f"package://collision_benchmark/scenarios/hydrogen_tank_small.ply"
    pcd_file = f"package://collision_benchmark/scenarios/hydrogen_tank_small.pcd"


class HydrogenTankSmallSeed(HydrogenTankSeed):
    name = 'hydrogen_tank_small_seed'
    ply_file = f"package://collision_benchmark/scenarios/hydrogen_tank_small.ply"
    pcd_file = f"package://collision_benchmark/scenarios/hydrogen_tank_small.pcd"


class HydrogenTankBig(HydrogenTank):
    name = 'hydrogen_tank_big'
    ply_file = f"package://collision_benchmark/scenarios/hydrogen_tank_big.ply"
    pcd_file = f"package://collision_benchmark/scenarios/hydrogen_tank_big_10.pcd"


class HydrogenTankBigSeed(HydrogenTankSeed):
    name = 'hydrogen_tank_big_seed'
    ply_file = f"package://collision_benchmark/scenarios/hydrogen_tank_big.ply"
    pcd_file = f"package://collision_benchmark/scenarios/hydrogen_tank_big.pcd"
