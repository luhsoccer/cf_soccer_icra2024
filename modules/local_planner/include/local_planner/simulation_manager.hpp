#pragma once

#include <utility>

#include "local_planner/skills/abstract_component.hpp"
#include "local_planner/skills/abstract_feature.hpp"
#include "local_planner/simulator.hpp"

namespace luhsoccer {
namespace marker {
class MarkerService;
}
namespace local_planner {
class ThreadPool;
class Skill;
class AvoidanceManager;

class SimulationManager : public AbstractComponent {
   public:
    SimulationManager(const SimulationManager&) = delete;
    SimulationManager(SimulationManager&&) = delete;
    SimulationManager& operator=(const SimulationManager&) = delete;
    SimulationManager& operator=(SimulationManager&&) = delete;
    SimulationManager(ThreadPool& thread_pool, marker::MarkerService& ms,
                      const std::shared_ptr<const transform::WorldModel>& real_wm);
    ~SimulationManager() override;

    void setNewTask(const RobotIdentifier& robot, const Skill* skill, const TaskData& td);

    void startSkillSimulation(const RobotIdentifier& robot, const Skill& skill, const TaskData& td,
                              const ResultCallback& result_callback, const time::TimePoint& start_time = time::now(),
                              std::shared_ptr<const transform::WorldModel> wm = nullptr);

    void startTaskDataSimulation(const RobotIdentifier& robot, const TaskData& td,
                                 const ResultCallback& result_callback, std::shared_ptr<const transform::WorldModel> wm,
                                 const std::optional<time::TimePoint>& start_from);

    SimulationResult runSyncSimulation(const RobotIdentifier& robot, const Skill& skill, const TaskData& td,
                                       const time::TimePoint& start_time = time::now(),
                                       std::shared_ptr<const transform::WorldModel> wm = nullptr);

    void stopSimulations() { this->should_run = false; }

    void linkAvoidanceManager(std::shared_ptr<AvoidanceManager> am) { this->am = std::move(am); }

    const Skill* getCurrentSkill(const RobotIdentifier& robot) const;
    std::optional<TaskData> getCurrentTaskData(const RobotIdentifier& robot) const;
    std::vector<std::shared_ptr<Simulator>> getSimulators(const RobotIdentifier& robot) const;

   private:
    struct RotationVectorData {
        std::vector<bool> rotation_vectors;
        int score;
    };

    void startNewSimulation(const RobotIdentifier& robot, const std::shared_ptr<const transform::WorldModel>& wm,
                            const TaskData& td, const time::TimePoint& time);

    std::unordered_map<RobotIdentifier, std::vector<std::shared_ptr<Simulator>>> simulators;
    std::unordered_map<RobotIdentifier, std::tuple<const Skill*, TaskData, time::TimePoint>> current_skills;
    std::vector<std::shared_ptr<Simulator>> independent_simulators;

    mutable std::shared_mutex simulator_mtx;
    std::atomic_bool should_run{true};
    ThreadPool& thread_pool;
    marker::MarkerService& ms;
    std::shared_ptr<const transform::WorldModel> real_wm;
    std::shared_ptr<AvoidanceManager> am;
    logger::Logger logger{"SimulationManager"};
};

}  // namespace local_planner
}  // namespace luhsoccer