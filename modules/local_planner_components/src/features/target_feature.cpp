

#include "local_planner_components/features/target_feature.hpp"

#include "local_planner/skills/abstract_shape.hpp"
#include "logger/logger.hpp"
namespace luhsoccer::local_planner {

Eigen::Vector2d TargetFeature::calcArtificialDesiredVelocity(const std::shared_ptr<const transform::WorldModel>& wm,
                                                             const TaskData& td, const RobotIdentifier& robot,
                                                             const time::TimePoint& time,
                                                             const ComponentPosition& observe_position) const {
    auto vec_and_vel_to_shape = this->shape->getTransformToClosestPoint(wm, td, robot, time, observe_position);
    auto robot_pos = ComponentPosition(td.robot.getFrame())
                         .positionObject(wm, td)
                         .getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero();
    if (vec_and_vel_to_shape.vec && robot_pos.has_value()) {
        auto field_data = wm->getFieldData();
        Eigen::Vector2d vec = vec_and_vel_to_shape.vec.value();
        vec.x() = std::max(-(field_data.size.x() / 2 + robot_pos->translation().x() - field_data.max_robot_radius +
                             field_data.field_runoff_width),
                           std::min(field_data.size.x() / 2 - robot_pos->translation().x() -
                                        field_data.max_robot_radius + field_data.field_runoff_width,
                                    vec.x()));
        vec.y() = std::max(-(field_data.size.y() / 2 + robot_pos->translation().y() - field_data.max_robot_radius +
                             field_data.field_runoff_width),
                           std::min(field_data.size.y() / 2 - robot_pos->translation().y() -
                                        field_data.max_robot_radius + field_data.field_runoff_width,
                                    vec.y()));
        desired_velocity = this->weight.val(wm, td) * this->k_g.val(wm, td) / this->k_v.val(wm, td) * vec;
    }

    auto robot_vel = wm->getVelocity(robot.getFrame(), observe_position.positionObject(wm, td).getFrame(),
                                     observe_position.positionObject(wm, td).getFrame(), time);
    if (!ignore_velocity.val(wm, td) && vec_and_vel_to_shape.velocity.has_value() && robot_vel.has_value()) {
        Eigen::Vector2d target_vel = vec_and_vel_to_shape.velocity.value() + robot_vel.value().velocity.head(2);
        desired_velocity += this->weight.val(wm, td) * target_vel;
    }

    return desired_velocity;
};

bool TargetFeature::isReached(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                              const RobotIdentifier& robot, const time::TimePoint& time) const {
    VectorWithVelocityStamped transform_to_shape = this->shape->getTransformToClosestPoint(wm, td, robot, time);
    if (!transform_to_shape.vec.has_value()) return false;

    bool velocity_zero = true;
    if (this->velocity_zero_for_reach.val(wm, td) && transform_to_shape.velocity.has_value()) {
        velocity_zero = transform_to_shape.velocity->norm() < ZERO_VELOCITY_TOLERANCE;
    }
    return transform_to_shape.vec->norm() < this->translational_tolerance.val(wm, td) && velocity_zero;
};

}  // namespace luhsoccer::local_planner