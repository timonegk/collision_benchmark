from ebike.scenario import AbstractScenario


class TestEntry(AbstractScenario):
    name = 'test_entry'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
    hole_position = [0.9, 0.17, 0.8]


class TestEntrySmall(AbstractScenario):
    name = 'test_entry_small'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
    hole_position = [0.0, 1.0, 0.5]


class Wing(AbstractScenario):
    name = 'wing'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"


class SquareTank(AbstractScenario):
    name = 'square_tank'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
