#pragma once

#include "local_planner/avoidance_manager_unit.hpp"
#include "local_planner/simulation_manager.hpp"

namespace luhsoccer::local_planner {

class MultiAgent : public AbstractAvoidanceManagerUnit, public AbstractComponent {
   public:
    MultiAgent(SimulationManager& simulation_manager, const std::shared_ptr<const transform::WorldModel>& real_wm,
               marker::MarkerService& ms);

    std::vector<bool> getRotationVectors(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
                                         const Eigen::Vector2d& goal_vec,
                                         const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                         const RobotIdentifier& robot, time::TimePoint time) override;

   private:
    void startNewSim(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                     const RobotIdentifier& robot, time::TimePoint time);

    static size_t getScore(const SimulationResult& result, const RobotIdentifier& robot);

    void showResult(const SimulationResult& result, const RobotIdentifier& robot, size_t score, bool reachable,
                    bool take_result, unsigned long id);

    SimulationManager& simulation_manager;

    struct Result {
        std::map<size_t, bool> field_vectors;
        size_t score;
    };
    mutable std::shared_mutex result_mtx;
    std::map<RobotIdentifier, Result> results;
    std::shared_ptr<const transform::WorldModel> real_wm;
    marker::MarkerService& ms;
};

}  // namespace luhsoccer::local_planner