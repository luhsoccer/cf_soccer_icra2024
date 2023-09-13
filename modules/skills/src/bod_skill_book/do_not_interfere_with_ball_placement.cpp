#include "skill_books/bod_skill_book/do_not_interfere_with_ball_placement.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
// include components here

namespace luhsoccer::skills {

DoNotInterfereWithBallPlacementBuild::DoNotInterfereWithBallPlacementBuild()
    : SkillBuilder("DoNotInterfereWithBallPlacement",  //
                   {},                                 //
                   {},                                 //
                   {},                                 //
                   {},                                 //
                   {},                                 //
                   {}){};

void DoNotInterfereWithBallPlacementBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step

    DriveStep drive_away;
    drive_away.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_away.setRotationControl(HeadingRotationControl("ball"));
    drive_away.setAvoidDefenseArea(false);
    drive_away.setMaxVelX(cs.skills_config.ball_placement_avoid_speed);
    drive_away.setMaxVelY(cs.skills_config.ball_placement_avoid_speed);
    drive_away.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, true);
    drive_away.addFeature(RobotCFObstacle(PointShape("ball"), 1.0));
    addStep(std::move(drive_away));
    // end of skill
}
}  // namespace luhsoccer::skills