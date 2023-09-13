#pragma once

#include "local_planner_components/steps/drive_step_constraint.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

class BoundariesConstraint : public DriveStepConstraint {
   public:
    BoundariesConstraint();

   private:
    [[nodiscard]] bool conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                         const TaskData& td) const override;
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>> getAdditionalObstacleFeaturesImpl()
        const override;
    static constexpr double FIELD_RUNOFF_WIDTH = 0.3;
};
}  // namespace luhsoccer::local_planner