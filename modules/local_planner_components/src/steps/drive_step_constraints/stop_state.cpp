#include "stop_state.hpp"

#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "transform/transform.hpp"

namespace luhsoccer::local_planner {

bool StopStateConstraint::conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                            const TaskData& /*td*/) const {
    if (localPlannerConfig().stop_state) return true;
    auto game_state = wm->getGameState();
    static const std::vector<transform::GameState> STOP_STATES{transform::GameState::STOP,
                                                               transform::GameState::FREE_KICK_ENEMY,
                                                               transform::GameState::PENALTY_PREP_ENEMY,
                                                               transform::GameState::KICKOFF_ENEMY,
                                                               transform::GameState::KICKOFF_PREP_ENEMY,
                                                               transform::GameState::KICKOFF_PREP};
    if (game_state.has_value())
        return std::find(STOP_STATES.begin(), STOP_STATES.end(), game_state.value()) != STOP_STATES.end();
    else
        return false;
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>>
StopStateConstraint::getAdditionalTargetFeaturesImpl() const {
    const auto& config = localPlannerConfig();

    std::vector<std::shared_ptr<const AbstractTargetFeature>> target_features;

    DoubleComponentParam w_border(
        CALLBACK, [&config](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto ball_pos = wm->getTransform("ball");
            auto robot_to_ball = wm->getTransform("ball", td.robot.getFrame());
            const auto field_data = wm->getFieldData();
            double active_distance = config.stop_state_min_ball_distance + config.robot_radius * 2;
            if (!ball_pos.has_value() || !robot_to_ball.has_value()) return 0.0;
            bool close_to_field_lines =
                (std::abs(ball_pos->transform.translation().x()) >
                     field_data.size.x() / 2 + field_data.field_runoff_width - active_distance ||
                 std::abs(ball_pos->transform.translation().y()) >
                     field_data.size.y() / 2 + field_data.field_runoff_width - active_distance);
            if (close_to_field_lines && robot_to_ball->transform.translation().norm() <
                                            config.stop_state_min_ball_distance + config.robot_radius) {
                return 100.0;
            } else {
                return 0.0;
            }
        });
    DoubleComponentParam w_ball(
        CALLBACK, [w_border](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w_border.val(wm, td) > 1.0) {
                return 10.0;
            } else {
                return 10000.0;
            }
        });
    DoubleComponentParam k_ball(
        CALLBACK, [w_border](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w_border.val(wm, td) > 1.0) {
                return 1.0;
            } else {
                return 20.0;
            }
        });
    DoubleComponentParam ball_anti_target_influence_distance(
        CALLBACK, [&config]() { return config.stop_state_min_ball_distance + config.robot_radius; });
    target_features.push_back(std::make_shared<AntiTargetFeature>(
        PointShape("ball"), ball_anti_target_influence_distance, w_ball, std::nullopt, std::nullopt, k_ball));

    DoubleComponentParam border_anti_target_influence_distance(
        CALLBACK, [&config]() { return config.stop_state_min_ball_distance * 2 + config.robot_radius * 3; });
    target_features.push_back(std::make_shared<AntiTargetFeature>(
        RectangleShape({transform::field::CENTER},
                       {CALLBACK,
                        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                            auto field_data = wm->getFieldData();
                            return field_data.size.y() + 2 * field_data.field_runoff_width;
                        }},
                       {CALLBACK,
                        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                            auto field_data = wm->getFieldData();
                            return field_data.size.x() + 2 * field_data.field_runoff_width;
                        }},
                       false),
        border_anti_target_influence_distance, w_border));

    for (const auto& other_robot : transform::RobotDataStorage::generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)) {
        DoubleComponentParam w(
            CALLBACK,
            [other_robot](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
                auto other_robot_data = wm->getRobotData(other_robot);
                auto ball_pos = wm->getTransform(other_robot.getFrame(), "ball");
                if (!ball_pos.has_value()) return 0.0;

                if (other_robot != td.robot && other_robot_data.has_value() && other_robot_data->on_field &&
                    ball_pos->transform.translation().norm() < 1.5) {
                    return 1.0;
                } else {
                    return 0.0;
                }
            });
        // target_features.emplace_back(std::make_shared<AntiTargetFeature>(PointShape({other_robot.getFrame()}), 0.3,
        // w,
        //                                                                  std::nullopt, std::nullopt, 3.0));
    }
    // constexpr double FIELD_RUNOFF_WIDTH = 0.3;
    // RectangleShape r1({{transform::field::GOAL_ENEMY_CENTER, FIELD_RUNOFF_WIDTH}},
    //                   {CALLBACK,
    //                    [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
    //                        return wm->getFieldData().penalty_area_width + localPlannerConfig().defense_area_margin *
    //                        2;
    //                    }},
    //                   {CALLBACK,
    //                    [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
    //                        return 2 * (wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH +
    //                                    localPlannerConfig().defense_area_margin);
    //                    }},
    //                   false);

    // RectangleShape r2({{transform::field::GOAL_ALLY_CENTER, -FIELD_RUNOFF_WIDTH}},
    //                   {CALLBACK,
    //                    [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
    //                        return wm->getFieldData().penalty_area_width + localPlannerConfig().defense_area_margin *
    //                        2;
    //                    }},
    //                   {CALLBACK,
    //                    [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
    //                        return 2 * (wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH +
    //                                    localPlannerConfig().defense_area_margin);
    //                    }},
    //                   false);

    // target_features.push_back(
    //     std::make_shared<AntiTargetFeature>(std::move(r1), border_anti_target_influence_distance, w_border));
    // target_features.push_back(
    //     std::make_shared<AntiTargetFeature>(std::move(r2), border_anti_target_influence_distance, w_border));

    return target_features;
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>>
StopStateConstraint::getAdditionalObstacleFeaturesImpl() const {
    const auto& config = localPlannerConfig();
    std::vector<std::shared_ptr<const AbstractCFObstacle>> obstacle_features;
    DoubleComponentParam w_b(
        CALLBACK, [&config](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto robot_pos = wm->getTransform("ball", td.robot.getFrame());
            if (robot_pos.has_value() &&
                robot_pos->transform.translation().norm() > config.stop_state_min_ball_distance + config.robot_radius) {
                return 1.0;
            } else {
                return 0.0;
            }
        });

    obstacle_features.push_back(std::make_shared<const RobotCFObstacle>(
        CircleShape({"ball"}, config.stop_state_min_ball_distance, false), w_b));
    return obstacle_features;
}

}  // namespace luhsoccer::local_planner