#!/usr/bin/env python3
import optuna
import reach_ros
from benchmark.benchmark import Benchmark
from benchmark.robot import Elise
from benchmark.scenario import Table
from benchmark.ik import AbstractIK
from benchmark.utils import result_to_df
import numpy as np


class PickIKOptuna(AbstractIK):
    name = "pick_ik_optuna"
    population_size = None
    elite_size = None
    wipeout_fitness_tol = None
    max_generations = None
    gd_max_iters = None
    gd_max_time = None
    gd_step_size = None

    def set_config(self):
        reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver',
                                'pick_ik/PickIkPlugin')
        reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_timeout', 1.0)
        reach_ros.set_parameter('robot_description_kinematics.all.mode', 'global')
        reach_ros.set_parameter('robot_description_kinematics.all.stop_optimization_on_valid_solution', True)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_num_threads', 1)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_stop_on_first_solution', True)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_population_size', self.population_size)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_elite_size', self.elite_size)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_wipeout_fitness_tol',
                                self.wipeout_fitness_tol)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_max_generations', self.max_generations)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_gd_max_iters', self.gd_max_iters)
        reach_ros.set_parameter('robot_description_kinematics.all.memetic_gd_max_time', self.gd_max_time)
        reach_ros.set_parameter('robot_description_kinematics.all.gd_step_size', self.gd_step_size)
        reach_ros.set_parameter('robot_description_kinematics.all.gd_min_cost_delta', 1.0e-12)
        reach_ros.set_parameter('robot_description_kinematics.all.cost_threshold', 10000.0)

    def update_parameters(self, trial):
        self.population_size = trial.suggest_int('memetic_population_size', 1, 100)
        self.elite_size = trial.suggest_int('memetic_elite_size', 1, self.population_size)
        self.wipeout_fitness_tol = trial.suggest_float('memetic_wipeout_fitness_tol', 0.000001, 0.1, log=True)
        self.max_generations = trial.suggest_int('memetic_max_generations', 1, 100)
        self.gd_max_iters = trial.suggest_int('gd_max_iters', 1, 100)
        self.gd_max_time = trial.suggest_float('gd_max_time', 0.00001, 0.1, log=True)
        self.gd_step_size = trial.suggest_float('gd_step_size', 0.000001, 0.1, log=True)


class BenchmarkOptimization:
    def __init__(self):
        self.benchmark = Benchmark()
        self.benchmark.add_robot(Elise())
        self.benchmark.add_scenario(Table())
        self.pick_ik = PickIKOptuna()
        self.benchmark.add_ik(self.pick_ik)

        sampler = optuna.samplers.TPESampler()
        storage = "sqlite:///optuna.db"

        study_name = 'pick_ik_optuna'
        if study_name in optuna.get_all_study_names(storage):
            self.study = optuna.load_study(study_name=study_name, storage=storage)
        else:
            self.study = optuna.create_study(study_name=study_name,
                                             sampler=sampler,
                                             storage=storage,
                                             direction="maximize")
            self.study.enqueue_trial({'memetic_num_threads': 1,
                                      'memetic_population_size': 16,
                                      'memetic_elite_size': 4,
                                      'memetic_wipeout_fitness_tol': 0.00001,
                                      'memetic_max_generations': 100,
                                      'memetic_gd_max_iters': 25,
                                      'memetic_gd_max_time': 0.005,
                                      'gd_step_size': 0.0001})

    def optimize(self):
        self.study.optimize(self.objective, n_trials=30)

    def objective(self, trial):
        self.pick_ik.update_parameters(trial)
        result = self.benchmark.run()
        result = result_to_df(result['elise']['table']['pick_ik_optuna'][0])
        num_reached = np.sum(result.reached == True)
        avg_ik_time = np.mean(result.ik_time[result.reached == True])
        trial.set_user_attr('avg_ik_time', avg_ik_time)
        return num_reached / len(result)


if __name__ == '__main__':
    bo = BenchmarkOptimization()
    bo.optimize()
    print(bo.study.best_params)
    print(bo.study.best_value)
    print(bo.study.best_trial.user_attrs)
    print(optuna.importance.get_param_importances(bo.study))
