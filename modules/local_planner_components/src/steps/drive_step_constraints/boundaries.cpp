#include "boundaries.hpp"

#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "transform/transform.hpp"

namespace luhsoccer::local_planner {

BoundariesConstraint::BoundariesConstraint() : DriveStepConstraint() { this->initFeatures(); };

bool BoundariesConstraint::conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& /*wm*/,
                                             const TaskData& /*td*/) const {
    return false;
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>>
BoundariesConstraint::getAdditionalObstacleFeaturesImpl() const {
    // RectangleShape r1({{transform::field::CENTER}},
    //                   {CALLBACK,
    //                    [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
    //                        return wm->getFieldData().size.y() + wm->getFieldData().field_runoff_width * 2;
    //                    }},
    //                   {CALLBACK,
    //                    [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
    //                        return wm->getFieldData().size.x() + wm->getFieldData().field_runoff_width * 2;
    //                    }},
    //                   false);

    LineShape l1({{transform::field::CORNER_ALLY_LEFT, -this->FIELD_RUNOFF_WIDTH, this->FIELD_RUNOFF_WIDTH}},
                 {{transform::field::CORNER_ENEMY_LEFT, this->FIELD_RUNOFF_WIDTH, this->FIELD_RUNOFF_WIDTH}});
    LineShape l2({{transform::field::CORNER_ALLY_RIGHT, -this->FIELD_RUNOFF_WIDTH, -this->FIELD_RUNOFF_WIDTH}},
                 {{transform::field::CORNER_ENEMY_RIGHT, this->FIELD_RUNOFF_WIDTH, -this->FIELD_RUNOFF_WIDTH}});
    LineShape l3({{transform::field::CORNER_ALLY_LEFT, -this->FIELD_RUNOFF_WIDTH, this->FIELD_RUNOFF_WIDTH}},
                 {{transform::field::CORNER_ALLY_RIGHT, -this->FIELD_RUNOFF_WIDTH, -this->FIELD_RUNOFF_WIDTH}});
    LineShape l4({{transform::field::CORNER_ENEMY_LEFT, this->FIELD_RUNOFF_WIDTH, this->FIELD_RUNOFF_WIDTH}},
                 {{transform::field::CORNER_ENEMY_RIGHT, this->FIELD_RUNOFF_WIDTH, -this->FIELD_RUNOFF_WIDTH}});
    return {std::make_shared<RobotCFObstacle>(std::move(l1), 1.0, 0.3, std::nullopt, 0.3),
            std::make_shared<RobotCFObstacle>(std::move(l2), 1.0, 0.3, std::nullopt, 0.3),
            std::make_shared<RobotCFObstacle>(std::move(l3), 1.0, 0.3, std::nullopt, 0.3),
            std::make_shared<RobotCFObstacle>(std::move(l4), 1.0, 0.3, std::nullopt, 0.3)};
}

}  // namespace luhsoccer::local_planner