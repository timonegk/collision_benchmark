from ebike.scenario import AbstractScenario


class HydrogenTank(AbstractScenario):
    name = 'HydrogenTank'
    file_name = 'hydrogen_tank'
    ply_file = f"package://collision_benchmark/scenarios/{file_name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{file_name}.pcd"
    hole_position = [0.0, 1.0, 0.5]
    hole_axis = [0, 1, 0]


class HydrogenTankSeed(HydrogenTank):
    name = 'HydrogenTank (Straight Seed)'
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
    name = 'HydrogenTank (Target Seed)'
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
    name = 'HydrogenTankSmall'
    ply_file = f"package://collision_benchmark/scenarios/hydrogen_tank_small.ply"
    pcd_file = f"package://collision_benchmark/scenarios/hydrogen_tank_small.pcd"


class HydrogenTankSmallSeed(HydrogenTankSmall):
    name = 'HydrogenTankSmall (Straight Seed)'

    def get_config(self, *args):
        config = super().get_config(*args)
        config["optimization"]["seed_state"] = [
                {"name": "shoulder_pan_joint", "position": 1.124},
                {"name": "shoulder_lift_joint", "position": -1.471},
                {"name": "elbow_joint", "position": 2.439},
                {"name": "wrist_1_joint", "position": 2.055},
                {"name": "wrist_2_joint", "position": -1.113},
                {"name": "wrist_3_joint", "position": 0.0},
                {"name": "endo_first_joint", "position": 0.0},
                {"name": "endo_second_joint_first_dof", "position": 0.0},
                {"name": "endo_second_joint_second_dof", "position": 0.0},
        ]
        return config


class HydrogenTankSmallSeed2(HydrogenTankSmall):
    name = 'HydrogenTankSmall (Target Seed)'

    def get_config(self, *args):
        config = super().get_config(*args)
        config["optimization"]["seed_state"] = [
                {"name": "shoulder_pan_joint", "position": 1.124},
                {"name": "shoulder_lift_joint", "position": -1.471},
                {"name": "elbow_joint", "position": 2.439},
                {"name": "wrist_1_joint", "position": 2.055},
                {"name": "wrist_2_joint", "position": -1.113},
                {"name": "wrist_3_joint", "position": 2.974},
                {"name": "endo_first_joint", "position": -0.304},
                {"name": "endo_second_joint_first_dof", "position": 0.120},
                {"name": "endo_second_joint_second_dof", "position": -1.375},
        ]
        return config
