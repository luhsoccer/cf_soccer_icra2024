#include "defense_area.hpp"

#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "transform/transform.hpp"

namespace luhsoccer::local_planner {

DefenseAreaConstraint::DefenseAreaConstraint()
    : DriveStepConstraint(),
      w1(CALLBACK,
         [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto pos_robot = wm->getTransform(td.robot.getFrame());
             auto game_state = wm->getGameState();

             if (!pos_robot.has_value() || !game_state.has_value()) return 0.0;
             auto field_data = wm->getFieldData();
             double margin = 0.0;
             if (game_state.value() == transform::GameState::STOP ||
                 game_state.value() == transform::GameState::FREE_KICK_ENEMY) {
                 margin = localPlannerConfig().defense_area_margin_stop;
             }
             if (pos_robot->transform.translation().x() > field_data.size.x() / 2 - field_data.penalty_area_depth -
                                                              localPlannerConfig().defense_area_margin - margin &&
                 std::abs(pos_robot->transform.translation().y()) <
                     field_data.penalty_area_width / 2 + localPlannerConfig().defense_area_margin + margin) {
                 return 0;
             } else {
                 return 1.0;
             }
         }),
      w2(CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
          auto pos_robot = wm->getTransform(td.robot.getFrame());

          if (!pos_robot.has_value()) return 0.0;
          auto field_data = wm->getFieldData();
          if (pos_robot->transform.translation().x() < -(field_data.size.x() / 2 - field_data.penalty_area_depth -
                                                         localPlannerConfig().defense_area_margin) &&
              std::abs(pos_robot->transform.translation().y()) <
                  field_data.penalty_area_width / 2 + localPlannerConfig().defense_area_margin) {
              return 0;
          } else {
              return 1.0;
          }
      }) {
    this->initFeatures();
};

bool DefenseAreaConstraint::conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                              const TaskData& td) const {
    auto goalie = wm->getGoalieID();
    auto game_state = wm->getGameState();

    bool stop_state_and_close = game_state.value_or(transform::GameState::NORMAL) == transform::GameState::STOP;
    if (stop_state_and_close) {
        auto ball_pos = wm->getTransform("ball");
        if (ball_pos.has_value()) {
            double active_distance =
                localPlannerConfig().stop_state_min_ball_distance + localPlannerConfig().robot_radius * 2;
            auto field_data = wm->getFieldData();
            stop_state_and_close =
                ball_pos->transform.translation().x() <
                    -(field_data.size.x() / 2 - field_data.penalty_area_depth - active_distance) &&
                std::abs(ball_pos->transform.translation().y()) < field_data.penalty_area_width / 2 + active_distance;
        }
    }
    bool in_ball_placement =
        game_state.has_value() && (game_state.value() == transform::GameState::BALL_PLACEMENT_ENEMY ||
                                   game_state.value() == transform::GameState::BALL_PLACEMENT_FREE_KICK ||
                                   game_state.value() == transform::GameState::BALL_PLACEMENT_FORCE_START);

    return (!goalie.has_value() || goalie.value() != td.robot) && !stop_state_and_close && !in_ball_placement;
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>>
DefenseAreaConstraint::getAdditionalTargetFeaturesImpl() const {
    DoubleComponentParam w1 = this->w1;
    DoubleComponentParam w1_anti(
        CALLBACK, [w1](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w1.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 10000.0;
            }
        });

    DoubleComponentParam w2 = this->w2;
    DoubleComponentParam w2_anti(
        CALLBACK, [w2](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w2.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 1000.0;
            }
        });

    DoubleComponentParam k1_anti(
        CALLBACK, [w1](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w1.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 2.0;
            }
        });

    DoubleComponentParam k2_anti(
        CALLBACK, [w2](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w2.val(wm, td) > 0.5) {
                return 0.0;
            } else {
                return 2.0;
            }
        });
    return {std::make_shared<AntiTargetFeature>(PointShape({{transform::field::GOAL_ENEMY_CENTER, FIELD_RUNOFF_WIDTH}}),
                                                2.5, w1_anti, 1.0, 1.0, k1_anti),
            std::make_shared<AntiTargetFeature>(PointShape({{transform::field::GOAL_ALLY_CENTER, -FIELD_RUNOFF_WIDTH}}),
                                                2.5, w2_anti, 1.0, 1.0, k2_anti)};
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>>
DefenseAreaConstraint::getAdditionalObstacleFeaturesImpl() const {
    RectangleShape r1(
        {{transform::field::GOAL_ENEMY_CENTER, FIELD_RUNOFF_WIDTH}},
        {CALLBACK,
         [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
             auto game_state = wm->getGameState();
             double margin = 0.0;
             if (game_state.has_value() && (game_state.value() == transform::GameState::STOP ||
                                            game_state.value() == transform::GameState::FREE_KICK_ENEMY)) {
                 margin = localPlannerConfig().defense_area_margin_stop;
             }
             return wm->getFieldData().penalty_area_width + localPlannerConfig().defense_area_margin * 2 + margin * 2;
         }},
        {CALLBACK,
         [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
             auto game_state = wm->getGameState();
             double margin = 0.0;
             if (game_state.has_value() && (game_state.value() == transform::GameState::STOP ||
                                            game_state.value() == transform::GameState::FREE_KICK_ENEMY)) {
                 margin = localPlannerConfig().defense_area_margin_stop;
             }
             return 2 * (wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH +
                         localPlannerConfig().defense_area_margin + margin);
         }},
        false);

    RectangleShape r2({{transform::field::GOAL_ALLY_CENTER, -FIELD_RUNOFF_WIDTH}},
                      {CALLBACK,
                       [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                           return wm->getFieldData().penalty_area_width + localPlannerConfig().defense_area_margin * 2;
                       }},
                      {CALLBACK,
                       [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                           return 2 * (wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH +
                                       localPlannerConfig().defense_area_margin);
                       }},
                      false);

    return {std::make_shared<RobotCFObstacle>(std::move(r1), this->w1, std::nullopt, std::nullopt,
                                              localPlannerConfig().defense_area_collision_distance),
            std::make_shared<RobotCFObstacle>(std::move(r2), this->w2, std::nullopt, std::nullopt,
                                              localPlannerConfig().defense_area_collision_distance)};
}

}  // namespace luhsoccer::local_planner