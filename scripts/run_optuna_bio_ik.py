#!/usr/bin/env python3
import numpy as np
import optuna
import reach_ros

from ebike.benchmark import Benchmark
from ebike.ik import AbstractIK
from ebike.robot import UR10
from ebike.scenario import *
from ebike.utils import result_to_df
from collision_benchmark.robot import Elise
from collision_benchmark.scenario import *


class BioIKOptuna(AbstractIK):
    name = "bio_ik_optuna"
    population_size = None
    child_count = None
    elite_count = None
    species_count = None
    memetic_opt_gens = None
    memetic_evolution_gens = None
    threads = None

    def set_config(self, planning_group):
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.kinematics_solver",
            "bio_ik/BioIKKinematicsPlugin",
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.kinematics_solver_timeout",
            1.0,
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.mode", "bio2_memetic"
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.population_size2",
            self.population_size,
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.child_count",
            self.child_count,
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.elite_count2",
            self.elite_count,
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.memetic_opt_gens",
            self.memetic_opt_gens,
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.memetic_evolution_gens",
            self.memetic_evolution_gens,
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.species_count",
            self.species_count,
        )
        reach_ros.set_parameter(
            f"robot_description_kinematics.{planning_group}.threads",
            self.threads,
        )

    def update_parameters(self, trial):
        self.population_size = trial.suggest_int("population_size", 1, 20)
        self.child_count = trial.suggest_int("child_count", 0, 100)
        self.elite_count = trial.suggest_int("elite_count", 0, self.population_size)
        self.species_count = trial.suggest_int("species_count", 1, 20)
        self.memetic_opt_gens = trial.suggest_int("memetic_opt_gens", 0, 50)
        self.memetic_evolution_gens = trial.suggest_int("memetic_evolution_gens", 0, 50)
        self.threads = trial.suggest_int("threads", 1, 8)


class BioIKLineGoalOptuna(BioIKOptuna):
    name = "bio_ik_line_goal_optuna"

    def set_config(self, planning_group):
        super().set_config(planning_group)
        reach_ros.set_parameter("reach_ros.use_line_goal", True)


class BenchmarkOptimization:
    def __init__(self):
        self.benchmark = Benchmark()
        self.robot = Elise()
        self.benchmark.add_robot(self.robot)
        #self.scenario = HydrogenTank()
        #self.scenario = TableObjects()
        #self.scenario = Shelf()
        self.scenario = Random()
        self.benchmark.add_scenario(self.scenario)
        #self.ik = BioIKLineGoalOptuna()
        self.ik = BioIKOptuna()
        self.benchmark.add_ik(self.ik)

        sampler = optuna.samplers.TPESampler()
        storage = "sqlite:///optuna.db"

        study_name = f"{self.ik.name}_{self.robot.name}_{self.scenario.name}"
        if study_name in optuna.get_all_study_names(storage):
            self.study = optuna.load_study(study_name=study_name, storage=storage)
        else:
            self.study = optuna.create_study(
                study_name=study_name,
                sampler=sampler,
                storage=storage,
                direction="minimize",
            )
            self.study.enqueue_trial(
                {

                    "population_size": 2,
                    "child_count": 16,
                    "species_count": 2,
                    "memetic_opt_gens": 8,
                    "memetic_evolution_gens": 8,
                    "threads": 4,
                }
            )

    def optimize(self):
        self.study.optimize(self.objective, n_trials=100)

    def objective(self, trial):
        self.ik.update_parameters(trial)
        result = self.benchmark.run()
        result = result_to_df(
            result[self.robot.name][self.scenario.name][self.ik.name][0]
        )
        num_reached = np.sum(result.reached == 1)
        avg_ik_time = np.mean(result.ik_time[result.reached == 1])
        max_ik_time = np.max(result.ik_time[result.reached == 1])
        trial.set_user_attr("solve_rate", num_reached / len(result))
        trial.set_user_attr("max_ik_time", max_ik_time)
        # use the ik time if it was reached, otherwise set it to 1
        time_metric = np.where(result.reached, result.ik_time, 1)
        # return mean
        return np.mean(time_metric)


if __name__ == "__main__":
    bo = BenchmarkOptimization()
    bo.optimize()
    print(bo.study.best_params)
    print(bo.study.best_value)
    print(bo.study.best_trial.user_attrs)
    print(optuna.importance.get_param_importances(bo.study))
