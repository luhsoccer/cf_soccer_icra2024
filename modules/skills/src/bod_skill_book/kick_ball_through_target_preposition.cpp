#include "skill_books/bod_skill_book/kick_ball_through_target_preposition.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "local_planner_components/steps/kick_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/arc_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"

namespace luhsoccer::skills {

KickBallThroughTargetPrepositionBuild::KickBallThroughTargetPrepositionBuild()
    : SkillBuilder("KickBallThroughTargetPreposition",  //
                   {},                                  //
                   {"TargetPoint"},                     //
                   {"KickVelocity"},                    //
                   {},                                  //
                   {},                                  //
                   {}){};

void KickBallThroughTargetPrepositionBuild::buildImpl(const config_provider::ConfigStore& cs) {
    ComponentPosition target(TD_Pos::POINT, 0);
    ComponentPosition ball_rotated_to_target(
        CALLBACK,
        [target](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            std::optional<Eigen::Affine2d> target_pos = target.positionObject(wm, td).getCurrentPosition(wm, "ball");
            if (target_pos.has_value()) {
                double target_rot = atan2(target_pos->translation().y(), target_pos->translation().x());
                return {"ball", 0.0, 0.0, target_rot + L_PI};
            }
            return {"ball"};
        });

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    DriveStep go_to_get_ball;
    go_to_get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_get_ball.setAvoidOtherRobots(true);
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball", false, L_PI / 2));
    go_to_get_ball.addFeature(TargetFeature(ArcShape(ball_rotated_to_target, cs.skills_config.move_to_ball_pre_radius,
                                                     cs.skills_config.kick_ball_through_target_preposition_angle),
                                            0.1));
    go_to_get_ball.addFeature(RobotCFObstacle(PointShape("ball"), cs.skills_config.ball_obstacle_weight));
    addStep(std::move(go_to_get_ball));

    addStep(TurnAroundBallStep({TD_Pos::POINT, 0}));

    addStep(KickStep({TD, 0}, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep drive_in_ball;
    drive_in_ball.addFeature(BallTargetFeature(CircleShape("ball", 0.03, true), 1.0, true));
    drive_in_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_in_ball.setCancelCondition(
        {CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto ball_pose = transform::Position(td.robot.getFrame()).getCurrentPosition(wm, "ball");
             return ball_pose.has_value() &&
                    ball_pose->translation().norm() > cs.skills_config.move_to_ball_pre_radius * 3;
         }});
    addStep(std::move(drive_in_ball));

    addStep(WaitStep(WAIT_DURATION, 0.2));
}
}  // namespace luhsoccer::skills
