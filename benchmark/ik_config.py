import reach_ros

def set_kinematics_config_kdl():
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver',
                            'kdl_kinematics_plugin/KDLKinematicsPlugin')
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_search_resolution', 0.001)
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_timeout', 1.0)


def set_kinematics_config_trac_ik():
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver',
                            'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin')
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_search_resolution', 0.001)
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_timeout', 1.0)
    reach_ros.set_parameter('robot_description_kinematics.all.solve_type', 'Speed')


def set_kinematics_config_trac_ik_distance():
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver',
                            'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin')
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_search_resolution', 0.001)
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_timeout', 1.0)
    reach_ros.set_parameter('robot_description_kinematics.all.solve_type', 'Distance')


def set_kinematics_config_pick_ik():
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver',
                            'pick_ik/PickIkPlugin')
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_search_resolution', 0.001)
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_timeout', 1.0)
    reach_ros.set_parameter('robot_description_kinematics.all.mode', 'global')
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_gd_max_iters', 5)
    reach_ros.set_parameter('robot_description_kinematics.all.memetic_population_size', 40)
    reach_ros.set_parameter('robot_description_kinematics.all.stop_optimization_on_valid_solution', True)
    reach_ros.set_parameter('robot_description_kinematics.all.cost_threshold', 10000.0)


def set_kinematics_config_bio_ik():
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver',
                            'bio_ik/BioIKKinematicsPlugin')
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_search_resolution', 0.001)
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_timeout', 1.0)
    reach_ros.set_parameter('robot_description_kinematics.all.kinematics_solver_attempts', 1)
    reach_ros.set_parameter('robot_description_kinematics.all.mode', 'bio2_memetic')
    #reach_ros.set_parameter('robot_description_kinematics.all.memetic_gd_max_iters', 5)
    #reach_ros.set_parameter('robot_description_kinematics.all.memetic_population_size', 40)
    #reach_ros.set_parameter('robot_description_kinematics.all.stop_optimization_on_valid_solution', True)
    #reach_ros.set_parameter('robot_description_kinematics.all.cost_threshold', 10000.0)
