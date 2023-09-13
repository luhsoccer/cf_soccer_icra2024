#include "multi_agent.hpp"
#include <numeric>
#include "local_planner/skills/abstract_shape.hpp"
#include "marker_service/marker_service.hpp"
#include "utils/utils.hpp"
#define SHOW_COLLIED_SIMULATIONS
namespace luhsoccer::local_planner {

MultiAgent::MultiAgent(SimulationManager& simulation_manager,
                       const std::shared_ptr<const transform::WorldModel>& real_wm, marker::MarkerService& ms)
    : simulation_manager(simulation_manager), real_wm(real_wm), ms(ms) {}

std::vector<bool> MultiAgent::getRotationVectors(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
                                                 const Eigen::Vector2d& /*goal_vec*/,
                                                 const std::shared_ptr<const transform::WorldModel>& wm,
                                                 const TaskData& td, const RobotIdentifier& robot,
                                                 time::TimePoint time) {
    // start main simulation is non is running
    if (this->simulation_manager.getSimulators(robot).size() == 0) {
        {
            // clear results
            std::unique_lock result_write_lock(this->result_mtx);
            results.erase(robot);
        }
        // NOLINTNEXTLINE(performance-unnecessary-copy-initialization) - taskdata is changed
        TaskData new_task_data = td;
        this->setCookie(new_task_data, "reached_obstacles", std::map<size_t, bool>());
        this->startNewSim(wm, new_task_data, robot, time::now());
    }

    // check if new obstacle is reached, and start_new simulation if needed
    auto reached_obstacles = this->getCookie<std::map<size_t, bool>>(td, "reached_obstacles");
    if (reached_obstacles.has_value()) {
        // this is a simulation check if new simulation has to be started
        std::stringstream ss;
        for (const auto& feature : features) {
            if (reached_obstacles->find(feature->getUid()) == reached_obstacles->end()) {
                auto new_reached_obstacles = reached_obstacles.value();
                reached_obstacles->insert_or_assign(feature->getUid(), false);
                new_reached_obstacles.insert_or_assign(feature->getUid(), true);
                LOG_DEBUG(logger::Logger("MultiAgent"),
                          "Starting new simulation for robot {} because feature {:d} was reached.", robot,
                          feature->getUid());
                // NOLINTNEXTLINE(performance-unnecessary-copy-initialization) - taskdata is changed
                TaskData new_task_data = td;
                this->setCookie(new_task_data, "reached_obstacles", new_reached_obstacles);
                this->startNewSim(wm, new_task_data, robot, time);
            }
            ss << feature->getUid() << ",";
        }
        this->setCookie(td, "reached_obstacles", reached_obstacles.value());

        // LOG_TRACE(logger::Logger("MultiAgent"), ss.str());
    } else {
        // this is the original real agent
        // take the best simulation result
        reached_obstacles = std::map<size_t, bool>();
        std::shared_lock read_lock(this->result_mtx);
        auto result_it = this->results.find(robot);
        if (result_it != this->results.end()) {
            reached_obstacles = result_it->second.field_vectors;
        }
    }

    // use field vectors from reached obstacle map
    std::vector<bool> magnetic_field_vectors(features.size());
    size_t i = 0;
    for (const auto& feature : features) {
        auto magentic_field_vector_it = reached_obstacles->find(feature->getUid());
        if (magentic_field_vector_it != reached_obstacles->end()) {
            magnetic_field_vectors[i] = magentic_field_vector_it->second;
        }
        i++;
    }
    return magnetic_field_vectors;
}

void MultiAgent::startNewSim(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                             const RobotIdentifier& robot, time::TimePoint time) {
    time::TimePoint sim_start_time = time::now();
    auto result_callback = [this, sim_start_time](unsigned long id, const SimulationResult& result) {
        LOG_DEBUG(logger::Logger("MultiAgent"), "Simulation with id {:d} finished after {:.0f} ms ", id,
                  time::Duration(time::now() - sim_start_time).asSec() * 1000);
        std::shared_lock read_lock(this->result_mtx);
        for (const auto& task : result.tasks) {
            size_t score = MultiAgent::getScore(result, task.first);
            auto result_it = this->results.find(task.first);

            // check if path is reachable
            bool reachable = false;
            constexpr time::Duration SCANNING_INTERVAL = 0.1;
            auto robot_pos = this->real_wm->getTransform(task.first.getFrame(), "");
            for (time::TimePoint time = result.start_time; time < result.end_time; time += SCANNING_INTERVAL) {
                auto simulation_pos = result.wm->getTransform(task.first.getFrame(), "", time);
                if (simulation_pos.has_value() &&
                    (robot_pos->transform.translation() - simulation_pos->transform.translation()).norm() <
                        localPlannerConfig().simulation_max_deviation) {
                    reachable = true;
                    break;
                }
            }
            bool take_result = false;
            if (reachable && (result_it == this->results.end() || result_it->second.score < score)) {
                // replace current field vector set
                read_lock.unlock();
                std::unique_lock write_lock(this->result_mtx);
                Result result;
                result.score = score;
                auto reached_obstacles =
                    this->getCookie<std::map<size_t, bool>>(task.second.task_data, "reached_obstacles");
                result.field_vectors = reached_obstacles.value_or(std::map<size_t, bool>());
                results.insert_or_assign(task.first, result);
                take_result = true;
            }
            this->showResult(result, task.first, score, reachable, take_result, id);
        }
    };
    this->simulation_manager.startTaskDataSimulation(robot, td, result_callback, wm, time);
}

size_t MultiAgent::getScore(const SimulationResult& result, const RobotIdentifier& robot) {
    const time::Duration scanning_interval = 0.03;
    std::optional<double> min_robot_distance;
    // 1 - check for collision
    for (time::TimePoint time = result.start_time; time < result.end_time; time += scanning_interval) {
        std::vector<RobotIdentifier> visible_robots = result.wm->getVisibleRobots(time);
        auto robot_pose = result.wm->getTransform(robot.getFrame(), "", time);
        for (const auto& other_robot : visible_robots) {
            if (robot == other_robot) continue;
            auto other_robot_pose = result.wm->getTransform(other_robot.getFrame(), "", time);
            if (robot_pose.has_value() && other_robot_pose.has_value()) {
                double distance =
                    (robot_pose->transform.translation() - other_robot_pose->transform.translation()).norm();
                if (!min_robot_distance.has_value() || min_robot_distance.value() > distance)
                    min_robot_distance = distance;
            }
        }
    }
    if (min_robot_distance < localPlannerConfig().simulation_scoring_collision_distance.val()) {
        return 0;
    }
    int score = 0;

    // 3 - duration of path

    double duration = time::Duration(result.end_time - result.start_time).asSec();

    score += static_cast<int>(localPlannerConfig().simulation_scoring_duration_k / duration);

    // 5 -  minimal distance to obstacle
    if (min_robot_distance.has_value()) {
        score += static_cast<int>(localPlannerConfig().simulation_scoring_obstacle_distance_k *
                                  (1 - std::exp(localPlannerConfig().robot_radius * 2 - min_robot_distance.value())));
    } else {
        score += static_cast<int>(localPlannerConfig().simulation_scoring_obstacle_distance_k);
    }
    return score;
}

void MultiAgent::showResult(const SimulationResult& result, const RobotIdentifier& robot, size_t score, bool reachable,
                            bool take_result, unsigned long id) {
    constexpr double VISUALIZATION_INTERVAL = 1.0 / 30.0;
    std::vector<marker::Point> points;
    for (auto time = result.start_time; time < result.end_time; time += time::Duration(VISUALIZATION_INTERVAL)) {
        auto pos = result.wm->getTransform(robot.getFrame(), "", time);
        if (pos.has_value()) {
            points.emplace_back(pos->transform.translation().x(), pos->transform.translation().y());
        }
    }
#ifndef SHOW_COLLIED_SIMULATIONS
    if (score == 0) return;
#endif
    marker::LineStrip strip("", fmt::format("{}SimulationResult", robot), id);
    strip.setPoints(points);
    marker::Color c;
    size_t marker_id = id;
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
    double lifetime = 5.0;
    double height = 0.02;
    if (score == 0) {
        c = marker::Color::YELLOW();
    } else if (take_result) {
        lifetime = 30.0;
        c = marker::Color::BLUE();
        marker_id = 0;
        strip.setId(0);
        height = 0.05;
        // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
    } else if (reachable) {
        c = marker::Color::GREEN();
    } else {
        c = marker::Color::GREY();
    }
    if (points.size() > 1) {
        strip.setLifetime(lifetime);
        strip.setColor(c);
        strip.setHeight(height);
        this->ms.displayMarker(strip);
        marker::Text text({"", points[points.size() / 2].x, points[points.size() / 2].y},
                          fmt::format("{}SimulationResultText", robot), marker_id);
        text.setText(fmt::format("Score: {:d}", score));
        text.setScale(marker::ScaleVec3(2));
        text.setColor(c);
        text.setLifetime(lifetime);
        text.setHeight(height);
        this->ms.displayMarker(text);
    }
}

}  // namespace luhsoccer::local_planner