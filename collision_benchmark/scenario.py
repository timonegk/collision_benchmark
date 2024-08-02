from ebike.scenario import AbstractScenario


class TestEntry(AbstractScenario):
    name = 'test_entry'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
    hole_position = [0.9, 0.17, 0.8]
    hole_axis = [1, 0, 0]


class TestEntrySeed(TestEntry):
    name = 'test_entry_seed'
    def get_config(self, *args):
        config = super().get_config(*args)
        config["optimization"]["seed_state"] = [
                {"name": "shoulder_pan_joint", "position": 0.02},
                {"name": "shoulder_lift_joint", "position": -0.28},
                {"name": "elbow_joint", "position": -1.43},
                {"name": "wrist_1_joint", "position": 1.70},
                {"name": "wrist_2_joint", "position": 1.59},
                {"name": "wrist_3_joint", "position": -2.07},
                {"name": "endo_first_joint", "position": 0.0},
                {"name": "endo_second_joint_first_dof", "position": 0.0},
                {"name": "endo_second_joint_second_dof", "position": 0.0},
        ]
        return config


class TestEntrySmall(AbstractScenario):
    name = 'test_entry_small'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
    hole_position = [0.0, 1.0, 0.5]
    hole_axis = [0, 1, 0]


class TestEntrySmallSeed(TestEntrySmall):
    name = 'test_entry_small_seed'
    def get_config(self, *args):
        config = super().get_config(*args)
        config["optimization"]["seed_state"] = [
                {"name": "shoulder_pan_joint", "position": 5.03},
                {"name": "shoulder_lift_joint", "position": -1.69},
                {"name": "elbow_joint", "position": -2.22},
                {"name": "wrist_1_joint", "position": -5.54},
                {"name": "wrist_2_joint", "position": -5.0},
                {"name": "wrist_3_joint", "position": -4.05},
                {"name": "endo_first_joint", "position": 0.0},
                {"name": "endo_second_joint_first_dof", "position": 0.0},
                {"name": "endo_second_joint_second_dof", "position": 0.0},
        ]
        return config


class Wing(AbstractScenario):
    name = 'wing'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"


class SquareTank(AbstractScenario):
    name = 'square_tank'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
