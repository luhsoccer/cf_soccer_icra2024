#include "skill_books/bod_skill_book/move_to_penalty_line.hpp"
#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
// include components here

namespace luhsoccer::skills {

MoveToPenaltyLineBuild::MoveToPenaltyLineBuild()
    : SkillBuilder("MoveToPenaltyLine",         //
                   {},                          //
                   {},                          //
                   {},                          //
                   {},                          //
                   {"PenaltyShootOnAllyGoal"},  //
                   {}){};

void MoveToPenaltyLineBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    auto move_to_penalty = DriveStep();

    // define parameters
    const double penalty_line_distance = 7.5;  // 6.0DivB + 1.0m behind ball + 0.5m puffer
    BoolComponentParam ally_side(TD, 0);

    // define drive steps
    DriveStep line_shoot_on_ally_goal;
    DriveStep line_shoot_on_enemy_goal;

    // loop through all allies and enemies for target features
    for (const auto other_robot : transform::RobotDataStorage::generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)) {
        DoubleComponentParam w(
            CALLBACK,
            [other_robot](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
                auto other_robot_data = wm->getRobotData(other_robot);

                if (other_robot != td.robot && other_robot_data.has_value() && other_robot_data->on_field) {
                    return 1.0;
                } else {
                    return 0.0;
                }
            });
        line_shoot_on_ally_goal.addFeature(
            RobotCFObstacle(CircleShape(other_robot, cs.skills_config.antigial_rad, true), w));
        line_shoot_on_enemy_goal.addFeature(
            RobotCFObstacle(CircleShape(other_robot, cs.skills_config.antigial_rad, true), w));
    }

    ComponentPosition penalty_line_ally_left({transform::field::CORNER_ALLY_LEFT, penalty_line_distance, 0.0, 0.0});
    ComponentPosition penalty_line_ally_right({transform::field::CORNER_ALLY_RIGHT, penalty_line_distance, 0.0, 0.0});
    line_shoot_on_ally_goal.addFeature(TargetFeature(LineShape(penalty_line_ally_left, penalty_line_ally_right)));
    line_shoot_on_ally_goal.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    line_shoot_on_ally_goal.setRotationControl(HeadingRotationControl(L_PI, ""));

    ComponentPosition penalty_line_enemy_left({transform::field::CORNER_ENEMY_LEFT, -penalty_line_distance, 0.0, 0.0});
    ComponentPosition penalty_line_enemy_right(
        {transform::field::CORNER_ENEMY_RIGHT, -penalty_line_distance, 0.0, 0.0});
    line_shoot_on_enemy_goal.addFeature(TargetFeature(LineShape(penalty_line_enemy_left, penalty_line_enemy_right)));
    line_shoot_on_enemy_goal.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    line_shoot_on_enemy_goal.setRotationControl(HeadingRotationControl(0.0, ""));

    // add condition step
    ConditionStep cond({TD, 0});
    cond.addIfStep(std::move(line_shoot_on_ally_goal));
    cond.addElseStep(std::move(line_shoot_on_enemy_goal));

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    addStep(std::move(cond));
}
}  // namespace luhsoccer::skills