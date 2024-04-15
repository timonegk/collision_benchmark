#!/usr/bin/env python3
import optuna
import reach_ros
from benchmark.robot_config import set_elise_parameters
from benchmark.reach_config import get_reach_config
from benchmark.utils import result_to_df
import reach
import os
import shutil
import numpy as np


def set_pick_ik_config(trial):
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver',
                            'pick_ik/PickIkPlugin')
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_timeout', 1.0)
    reach_ros.set_parameter('robot_description_kinematics.all.mode', 'global')
    reach_ros.set_parameter('robot_description_kinematics.all.stop_optimization_on_valid_solution', True)
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_num_threads', 1)
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_stop_on_first_solution', True)
    population_size = trial.suggest_int('memetic_population_size', 1, 100)
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_population_size', population_size)
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_elite_size',
                            trial.suggest_int('memetic_elite_size', 1, population_size))
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_wipeout_fitness_tol',
                            trial.suggest_float('memetic_wipeout_fitness_tol', 0.000001, 0.1, log=True))
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_max_generations',
                            trial.suggest_int('memetic_max_generations', 1, 100))
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_gd_max_iters',
                            trial.suggest_int('memetic_gd_max_iters', 1, 100))
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_gd_max_time',
                            trial.suggest_float('memetic_gd_max_time', 0.00001, 0.1, log=True))
    reach_ros.set_parameter('robot_description_kinematics.all.gd_step_size',
                            trial.suggest_float('gd_step_size', 0.000001, 0.1, log=True))
    reach_ros.set_parameter('robot_description_kinematics.all.gd_min_cost_delta', 1.0e-12)
    reach_ros.set_parameter('robot_description_kinematics.all.cost_threshold', 10000.0)


def objective(trial):
    set_pick_ik_config(trial)
    config = get_reach_config('test_entry')
    results_dir = "/tmp/reach_results"
    shutil.rmtree(results_dir)
    reach.runReachStudy(config, "reach_study", results_dir, False)
    db = reach.load(f"{results_dir}/reach_study/reach.db.xml")
    result = result_to_df(db.results[0])
    num_reached = np.sum(result.reached == True)
    avg_ik_time = np.mean(result.ik_time[result.reached == True])
    trial.set_user_attr('avg_ik_time', avg_ik_time)
    return num_reached / len(result)


if __name__ == '__main__':
    # General setup
    reach_ros.init_ros([])
    set_elise_parameters()
    results_dir = "/tmp/reach_results"
    os.environ["REACH_PLUGINS"] = "reach_ros_plugins"
    sampler = optuna.samplers.TPESampler()
    storage = optuna.storages.RDBStorage(
        url="sqlite:///optuna.db",
        engine_kwargs={"pool_size": 20, "connect_args": {"timeout": 10}},
    )
    study = optuna.create_study(study_name='pick_ik_test_entry',
                                sampler=sampler,
                                storage=storage,
                                direction="maximize")
    study.enqueue_trial({'memetic_num_threads': 1,
                         'memetic_population_size': 16,
                         'memetic_elite_size': 4,
                         'memetic_wipeout_fitness_tol': 0.00001,
                         'memetic_max_generations': 100,
                         'memetic_gd_max_iters': 25,
                         'memetic_gd_max_time': 0.005,
                         'gd_step_size': 0.0001})
    #study = optuna.load_study(study_name='pick_ik_test_entry', storage=storage)
    study.optimize(objective, n_trials=30)
    print(study.best_params)
    print(study.best_value)
    print(study.best_trial.user_attrs)
    print(optuna.importance.get_param_importances(study))
