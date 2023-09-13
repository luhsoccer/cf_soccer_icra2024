#include "local_planner/simulation_manager.hpp"
#include "local_planner/thread_pool.hpp"
#include "marker_service/marker_service.hpp"
#include "local_planner/local_planner.hpp"

#include "local_planner/skills/abstract_shape.hpp"

#define SHOW_COLLIED_SIMULATIONS
namespace luhsoccer::local_planner {

SimulationManager::SimulationManager(ThreadPool& thread_pool, marker::MarkerService& ms,
                                     const std::shared_ptr<const transform::WorldModel>& real_wm)
    : thread_pool(thread_pool), ms(ms), real_wm(real_wm), am(nullptr) {}

SimulationManager::~SimulationManager() {
    this->should_run = false;
    for (const auto& simulator : simulators) {
        for (const auto& sim : simulator.second) {
            sim->stopSimulation();
        }
    }
}

void SimulationManager::setNewTask(const RobotIdentifier& robot, const Skill* skill, const TaskData& td) {
    std::unique_lock lock(this->simulator_mtx);
    auto simulator_it = this->simulators.find(robot);

    if (simulator_it != this->simulators.end()) {
        // stop old simulation
        for (auto& simulator : simulator_it->second) {
            simulator->stopSimulation();
        }
        simulator_it->second.clear();
    } else {
        this->simulators.emplace(robot, std::vector<std::shared_ptr<Simulator>>());
    }

    this->current_skills.insert_or_assign(robot, std::make_tuple(skill, td, time::now()));
}

const Skill* SimulationManager::getCurrentSkill(const RobotIdentifier& robot) const {
    std::shared_lock lock(this->simulator_mtx);
    auto current_skill_it = this->current_skills.find(robot);
    if (current_skill_it == this->current_skills.end()) return nullptr;
    return std::get<0>(current_skill_it->second);
}

std::optional<TaskData> SimulationManager::getCurrentTaskData(const RobotIdentifier& robot) const {
    std::shared_lock lock(this->simulator_mtx);
    auto current_skill_it = this->current_skills.find(robot);
    if (current_skill_it == this->current_skills.end()) return std::nullopt;
    return std::get<1>(current_skill_it->second);
}

std::vector<std::shared_ptr<Simulator>> SimulationManager::getSimulators(const RobotIdentifier& robot) const {
    std::shared_lock lock(this->simulator_mtx);
    auto simulator_it = this->simulators.find(robot);
    if (simulator_it == this->simulators.end()) return {};
    return simulator_it->second;
}

void SimulationManager::startTaskDataSimulation(const RobotIdentifier& robot, const TaskData& td,
                                                const ResultCallback& result_callback,
                                                std::shared_ptr<const transform::WorldModel> wm,
                                                const std::optional<time::TimePoint>& start_from) {
    std::unique_lock lock(this->simulator_mtx);
    auto simulators_it = this->simulators.find(robot);
    auto current_skill_data_it = this->current_skills.find(robot);
    if (simulators_it == this->simulators.end()) return;
    if (current_skill_data_it == this->current_skills.end()) return;

    if (wm == nullptr) {
        wm = this->real_wm;
    }
    auto simulator = std::make_shared<Simulator>(this->thread_pool, wm, this->am);
    simulator->setTasks({{robot, {std::get<0>(current_skill_data_it->second), td, {}}}});
    simulator->setResultCallback(result_callback);
    Simulator::startSimulation(simulator, this->should_run, std::get<2>(current_skill_data_it->second), start_from);
    simulators_it->second.push_back(simulator);
}

void SimulationManager::startSkillSimulation(const RobotIdentifier& robot, const Skill& skill, const TaskData& td,
                                             const ResultCallback& result_callback, const time::TimePoint& start_time,
                                             std::shared_ptr<const transform::WorldModel> wm) {
    if (wm == nullptr) {
        wm = this->real_wm;
    }
    auto simulator = std::make_shared<Simulator>(this->thread_pool, wm, this->am);
    simulator->setTasks({{robot, {&skill, td, {}}}});
    simulator->setResultCallback(result_callback);
    Simulator::startSimulation(simulator, this->should_run, start_time, std::nullopt);
    std::lock_guard lock(this->simulator_mtx);
    this->independent_simulators.push_back(simulator);
}

SimulationResult SimulationManager::runSyncSimulation(const RobotIdentifier& robot, const Skill& skill,
                                                      const TaskData& td, const time::TimePoint& start_time,
                                                      std::shared_ptr<const transform::WorldModel> wm) {
    if (wm == nullptr) {
        wm = this->real_wm;
    }
    auto simulator = std::make_shared<Simulator>(this->thread_pool, wm, this->am);
    simulator->setTasks({{robot, {&skill, td, {}}}});
    return simulator->runSyncSimulation(this->should_run, start_time);
}

}  // namespace luhsoccer::local_planner