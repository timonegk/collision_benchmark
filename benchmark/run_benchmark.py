import reach
import reach_ros
import os
import sys
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import shutil
from benchmark.ik_config import (set_kinematics_config_bio_ik,
                                 set_kinematics_config_kdl,
                                 set_kinematics_config_trac_ik,
                                 set_kinematics_config_pick_ik)
from benchmark.robot_config import set_elise_parameters
from benchmark.visualization import plot_results
from benchmark.reach_config import get_reach_config

IK_SOLVERS = {
    'bio_ik': set_kinematics_config_bio_ik,
    'kdl': set_kinematics_config_kdl,
    'trac_ik': set_kinematics_config_trac_ik,
    # 'trac_ik_distance': set_kinematics_config_trac_ik_distance,
    'pick_ik': set_kinematics_config_pick_ik,
}


def run_benchmark():
    reach_ros.init_ros(sys.argv)
    set_elise_parameters()
    config = get_reach_config('table')
    results_dir = "/tmp/reach_results"
    os.environ["REACH_PLUGINS"] = "reach_ros_plugins"
    results = {}
    for solver, set_config in IK_SOLVERS.items():
        print(f"Running benchmark for {solver}")
        set_config()
        config_name = f"reach_study_{solver}"
        reach.runReachStudy(config, config_name, results_dir, False)
        db = reach.load(f"{results_dir}/{config_name}/reach.db.xml")
        results[solver] = db.results[0]
    plot_results(results)
    shutil.rmtree(results_dir)
