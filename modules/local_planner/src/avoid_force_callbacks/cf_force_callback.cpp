#include "cf_force_callback.hpp"

#include "local_planner/skills/abstract_feature.hpp"

#include "local_planner/avoidance_manager.hpp"

#include "calc_cf_force.hpp"
namespace luhsoccer::local_planner {
AvoidForceResult getCFForce(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                            const RobotIdentifier& robot, AvoidanceManager& am, const time::TimePoint time,
                            const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                            const Eigen::Vector2d& target_force, const Eigen::Vector2d& max_target,
                            const Eigen::Vector2d& mean_weighted_target) {
    constexpr double SMALL_VALUE = 0.00001;

    AvoidForceResult result;

    double angle_goal = std::atan2(mean_weighted_target.y(), mean_weighted_target.x());

    // determine which obstacle could have an influence based on distance and weight
    std::vector<std::shared_ptr<const AbstractCFObstacle>> active_obstacles;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> active_obstacle_results;
    for (const auto& cf_feature : obstacles) {
        auto res = cf_feature->getVecAndVelocity(wm, td, robot, time);

        // rotation_vector_map[cf_feature->getUid()] = rotation_vectors[i];
        if (res.has_value() &&
            res->first.norm() < std::max(std::min(cf_feature->getInfluenceDistance(wm, td), max_target.norm()),
                                         localPlannerConfig().robot_radius.val()) &&
            cf_feature->getWeight(wm, td) > SMALL_VALUE) {
            active_obstacles.push_back(cf_feature);
            active_obstacle_results.push_back(res.value());
        } else {
            result.influence_map[cf_feature->getUid()] = false;
        }
    }

    // request rotation vectors for active obstacles
    auto rotation_vectors = am.getRotationVectors(active_obstacles, mean_weighted_target, wm, td, robot, time);
    if (rotation_vectors.size() != active_obstacles.size())
        throw std::range_error(
            "The number of rotation vectors provided by the avoidance manager should equal the "
            "number of rotation features.");

    // calc cf force based on obstacles and rotation vectors
    auto result_it = active_obstacle_results.begin();
    int i = 0;
    std::optional<Eigen::Vector2d> vec_to_closest_obstacle;
    Eigen::Vector2d cf_feature_force = Eigen::Vector2d::Zero();
    for (const auto& cf_feature : active_obstacles) {
        // switch vector if closest point is on other side than goal
        double angle_feature = std::atan2(result_it->first.y(), result_it->first.x());

        double angle_group_goal = angle_goal - angle_feature;

        angle_group_goal = cropAngle(angle_group_goal);
        if (std::abs(angle_group_goal) > localPlannerConfig().angle_for_inverse)
            rotation_vectors[i] = !rotation_vectors[i];

        auto cf_force = calcCFForce(result_it->first, result_it->second, rotation_vectors[i]);
        result.magnetic_field_vec_map[cf_feature->getUid()] = rotation_vectors[i];
        cf_feature_force +=
            cf_force * localPlannerConfig().feature_robot_obstacle_k_cf.val() * cf_feature->getWeight(wm, td);
        // determine closes cf obstacle vector
        if (!vec_to_closest_obstacle.has_value() || result_it->first.norm() < vec_to_closest_obstacle->norm())
            vec_to_closest_obstacle = result_it->second;

        if (result_it->first.norm() < cf_feature->getCriticalDistance(wm, td)) {
            result.critical_vectors.emplace_back(result_it->first, rotation_vectors[i]);
        }
        result.influence_map[cf_feature->getUid()] = cf_force.norm() > SMALL_VALUE;
        result_it++;
        i++;
    }

    double goal_relaxation_factors = calcTargetRelaxationFactors(vec_to_closest_obstacle, max_target);

    result.total_force = target_force * goal_relaxation_factors + cf_feature_force;

    return result;
}
}  // namespace luhsoccer::local_planner