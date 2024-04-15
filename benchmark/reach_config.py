

def get_reach_config(scenario):
    ply_file = f'package://benchmark/scenarios/{scenario}.ply'
    pcd_file = f'package://benchmark/scenarios/{scenario}.pcd'
    reach_config = {
        'optimization': {
            'radius': 0.2,
            'max_steps': 0,
            'step_improvement_threshold': 0.01,
            'max_threads': 1
        },
        'ik_solver': {
            'name': 'MoveItIKSolver',
            'distance_threshold': 0.0,
            'planning_group': 'all',
            'collision_mesh_filename': ply_file,
            'touch_links': ['end_effector', 'endo_third_link']
        },
        'evaluator': {
            'name': 'NoOpEvaluator',
        },
        'display': {
            'name': 'ROSDisplay',
            'collision_mesh_filename': ply_file,
            'kinematic_base_frame': 'base_link',
            'marker_scale': 0.05,
        },
        'target_pose_generator': {
            'name': 'PointCloudTargetPoseGenerator',
            'pcd_file': pcd_file
        },
        'logger': {
            'name': 'BoostProgressConsoleLogger'
        }
    }
    return reach_config
