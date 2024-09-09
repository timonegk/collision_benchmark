#!/usr/bin/env python
import os

from ebike.benchmark import Benchmark
from ebike.ik import *
from ebike.scenario import *
from collision_benchmark.scenario import *
from collision_benchmark.robot import Elise
from collision_benchmark.ik import *

if __name__ == '__main__':
    os.environ["REACH_PLUGINS"] = "collision_benchmark_reach_plugin"
    benchmark = Benchmark(interactive=True)
    benchmark.add_ik(KDL())
    benchmark.add_scenario(HydrogenTank())
    benchmark.add_scenario(HydrogenTankSmall())
    benchmark.add_robot(Elise())
    benchmark.run()
    benchmark.print()
    benchmark.plot()
