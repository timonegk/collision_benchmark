#!/usr/bin/env python
from ebike.benchmark import Benchmark
from ebike.ik import KDL, TracIK, PickIK, BioIK, RelaxedIK
from ebike.scenario import Table, SmallTable, Kallax, Barrel
from collision_benchmark.scenario import Wing, TestEntry
from collision_benchmark.robot import Elise

if __name__ == '__main__':
    benchmark = Benchmark()
    #benchmark.add_ik(KDL())
    #benchmark.add_ik(TracIK())
    #benchmark.add_ik(PickIK())
    #benchmark.add_ik(BioIK())
    benchmark.add_ik(RelaxedIK())
    benchmark.add_scenario(TestEntry())
    benchmark.add_robot(Elise())
    benchmark.run()
    benchmark.print()
    benchmark.plot()
