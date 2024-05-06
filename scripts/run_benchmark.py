#!/usr/bin/env python
from benchmark.benchmark import Benchmark
from benchmark.ik import KDL, TracIK, PickIK, BioIK
from benchmark.scenario import Table, TestEntry, Wing, Kallax
from benchmark.robot import Elise

if __name__ == '__main__':
    benchmark = Benchmark()
    benchmark.add_ik(KDL())
    benchmark.add_ik(TracIK())
    benchmark.add_ik(PickIK())
    benchmark.add_ik(BioIK())
    benchmark.add_scenario(Table())
    #benchmark.add_scenario(TestEntry())
    #benchmark.add_scenario(Kallax())
    benchmark.add_robot(Elise())
    benchmark.run()
    benchmark.plot()
