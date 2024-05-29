from ebike.scenario import AbstractScenario

class TestEntry(AbstractScenario):
    name = 'test_entry'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"

class Wing(AbstractScenario):
    name = 'wing'
    ply_file = f"package://collision_benchmark/scenarios/{name}.ply"
    pcd_file = f"package://collision_benchmark/scenarios/{name}.pcd"
