#include <relaxed_ik/objective.hpp>

namespace relaxed_ik {
    class EnvCollisionDistance : public Objective {
    public:
        explicit EnvCollisionDistance(const planning_scene::PlanningSceneConstPtr &planning_scene) : planning_scene_(planning_scene) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningSceneConstPtr planning_scene_;
    };

    class EnvCollisionDistance2 : public Objective {
    public:
        explicit EnvCollisionDistance2(const planning_scene::PlanningSceneConstPtr &planning_scene) : planning_scene_(planning_scene) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningSceneConstPtr planning_scene_;
    };

    class EnvCollisionDepth : public Objective {
    public:
        explicit EnvCollisionDepth(const planning_scene::PlanningSceneConstPtr &planning_scene) : planning_scene_(planning_scene) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningSceneConstPtr planning_scene_;
    };

    class EnvCollisionDepth2 : public Objective {
    public:
        explicit EnvCollisionDepth2(const planning_scene::PlanningSceneConstPtr &planning_scene) : planning_scene_(planning_scene) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningSceneConstPtr planning_scene_;
    };

    class RCMGoal : public Objective {
    public:
        explicit RCMGoal(Eigen::Vector3d point) : point_(std::move(point)) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        const Eigen::Vector3d point_;
    };

    class RCMGoal2 : public Objective {
    public:
        RCMGoal2(const moveit::core::RobotModelConstPtr &robot_model, const Eigen::Vector3d &point);
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        planning_scene::PlanningScene planning_scene_;
        collision_detection::AllowedCollisionMatrix acm_;
    };

    class RCMGoal3 : public Objective {
    public:
        explicit RCMGoal3(const std::string &link_name, Eigen::Vector3d point) : point_(std::move(point)), link_name_(link_name) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        const Eigen::Vector3d point_;
        const std::string link_name_;
    };

    class LineGoal : public Objective {
    public:
        LineGoal(Eigen::Vector3d point, Eigen::Vector3d axis, std::string link_name) :
                point_(std::move(point)), axis_(std::move(axis)), link_name_(std::move(link_name)) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        const Eigen::Vector3d point_;
        const Eigen::Vector3d axis_;
        const std::string link_name_;
    };

    class AlignmentGoal : public Objective {
    public:
        AlignmentGoal(Eigen::Vector3d axis, std::string link_name) :
                axis_(std::move(axis)), link_name_(std::move(link_name)) {};
        double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    private:
        const Eigen::Vector3d axis_;
        const std::string link_name_;
    };

    class ScanGoal : public Objective {
        public:
            double call(const std::vector<double> &joints, const Variables &v, const moveit::core::RobotState &state) override;
    };
}
