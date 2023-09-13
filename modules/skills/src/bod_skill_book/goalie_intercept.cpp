#include "skill_books/bod_skill_book/goalie_intercept.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/kick_step.hpp"
// include components here

namespace luhsoccer::skills {

GoalieInterceptBuild::GoalieInterceptBuild()
    : SkillBuilder("GoalieIntercept",  //
                   {},                 //
                   {},                 //
                   {},                 //
                   {},                 //
                   {"Reflect"},        //
                   {}){};

void GoalieInterceptBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    // velocity extrapolation
    ComponentPosition past_ball_pos(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            ComponentPosition ball_pos("ball");
            auto past_ball_pos = ball_pos.positionObject(wm, td).getVelocity(wm);

            // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
            if (past_ball_pos.has_value() && past_ball_pos->norm() > 0.5) {
                transform::Position ball_vel(
                    "", ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().x() + past_ball_pos->x(),
                    ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().y() + past_ball_pos->y());
                return ball_vel;
            }

            return ComponentPosition(td.robot.getFrame()).positionObject(wm, td);
        });

    auto calc_y_on_goal = [past_ball_pos](const std::shared_ptr<const transform::WorldModel>& wm,
                                          const TaskData& td) -> std::optional<double> {
        ComponentPosition b_comp("ball");

        auto b_pos = b_comp.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        auto e_pos = past_ball_pos.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);

        if (!b_pos.has_value() || !e_pos.has_value()) return std::nullopt;

        double dx = b_pos->translation().x() - e_pos->translation().x();
        double dy = b_pos->translation().y() - e_pos->translation().y();

        return b_pos->translation().y() - ((dy / dx) * b_pos->translation().x());
    };

    ComponentPosition second_line_point(
        CALLBACK,
        [&cs, calc_y_on_goal, past_ball_pos](const std::shared_ptr<const transform::WorldModel>& wm,
                                             const TaskData& td) -> transform::Position {
            auto y_on_goal_line = calc_y_on_goal(wm, td);
            if (y_on_goal_line.has_value() &&
                std::abs(y_on_goal_line.value()) > cs.skills_config.goalie_intercept_goal_distance) {
                ComponentPosition ally_goal(transform::field::GOAL_ALLY_CENTER);
                auto ally_goal_pos = ally_goal.positionObject(wm, td).getCurrentPosition(wm);
                int factor = y_on_goal_line.value() > 0 ? 1 : -1;
                if (ally_goal_pos.has_value())
                    return {"", ally_goal_pos->translation().x(),
                            factor * cs.skills_config.goalie_intercept_goal_distance};
            }

            return past_ball_pos.positionObject(wm, td);
        });

    // Wait for the ball with the correct rotation
    // DriveStep wait_for_kick;
    // wait_for_kick.setRotationControl(HeadingRotationControl("ball"));
    // wait_for_kick.setCancelCondition(
    //     {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
    //          auto ball_vel = wm->getVelocity("ball");
    //          auto ball_info = wm->getBallInfo();
    //          bool ally_has_ball = false;
    //          bool ball_moves = false;

    //          if (ball_vel.has_value()) {
    //              ball_moves = ball_vel->velocity.norm() > 0.5;
    //          }
    //          if (ball_info.has_value()) {
    //              if (ball_info->robot.has_value()) {
    //                  ally_has_ball = ball_info->robot->isAlly();
    //              }
    //          }

    //          return (ball_moves && !ally_has_ball);
    //      }});
    // wait_for_kick.setReachCondition(DriveStep::ReachCondition::NEVER);
    // wait_for_kick.setAvoidDefenseArea(false);
    // addStep(std::move(wait_for_kick));

    ConditionStep cond(
        {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
             return td.required_bools[0];
         }});
    cond.addIfStep(KickStep(6.5, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));
    cond.addElseStep(DribblerStep(robot_interface::DribblerMode::LOW));

    addStep(std::move(cond));

    // DriveStep for future ball position
    DriveStep receive_ball;
    receive_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    receive_ball.addFeature(BallTargetFeature(LineShape(second_line_point, "ball", -10.0, 0.0), 1.0, true,
                                              cs.skills_config.receive_ball_kg, cs.skills_config.receive_ball_kv));
    receive_ball.setAvoidDefenseArea(false);
    receive_ball.setAvoidOtherRobots(false);
    receive_ball.setRotationControl(HeadingRotationControl("ball"));
    receive_ball.setCancelCondition(
        {CALLBACK,
         [&cs, calc_y_on_goal](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
             auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);

             if (ball_pos.has_value()) {
                 if (ball_pos->translation().x() > 4.5 || ball_pos->translation().x() < -4.5 ||
                     ball_pos->translation().y() > 3.0 || ball_pos->translation().y() < -3.0) {
                     return true;
                 }
             }

             //  auto y_on_goal_line = calc_y_on_goal(wm, td);
             //  if (y_on_goal_line.has_value() &&
             //      std::abs(y_on_goal_line.value()) > cs.skills_config.goalie_intercept_goal_distance)
             //      return true;

             return false;
         }});

    addStep(std::move(receive_ball));
    // end of skill
}
}  // namespace luhsoccer::skills