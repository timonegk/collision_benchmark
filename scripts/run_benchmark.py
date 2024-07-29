#!/usr/bin/env python
from ebike.benchmark import Benchmark
from ebike.ik import KDL, TracIK, TracIKDistance, PickIK, BioIK, RelaxedIK, BioIK1, BioIK2MemeticL, BioIK2MemeticLBFGS, BioIK2, \
    BioIKGDR, BioIKGDR8, BioIKJac, BioIKJac8, BioIKOptlibBFGS, BioIKOptlibBFGS4, BioIKRCM, BioIKDepth, BioIKCollisionDistance, RelaxedIKRCM, RelaxedIKDepth, RelaxedIKCollisionDistance
from ebike.scenario import Table, SmallTable, Kallax, Barrel, Random
from collision_benchmark.scenario import Wing, TestEntrySmall, TestEntry
from collision_benchmark.robot import Elise

if __name__ == '__main__':
    benchmark = Benchmark(interactive=False)
    #benchmark.add_ik(KDL())
    #benchmark.add_ik(TracIK())
    #benchmark.add_ik(PickIK())
    #benchmark.add_ik(BioIK())
    #benchmark.add_ik(BioIKRCM())
    #benchmark.add_ik(BioIKDepth())
    benchmark.add_ik(BioIKCollisionDistance())
    #benchmark.add_ik(RelaxedIK())
    #benchmark.add_ik(RelaxedIKRCM())
    #benchmark.add_ik(RelaxedIKDepth())
    benchmark.add_ik(RelaxedIKCollisionDistance())
    #benchmark.add_scenario(Random())
    benchmark.add_scenario(TestEntrySmall())
    benchmark.add_robot(Elise())
    benchmark.run()
    benchmark.print()
