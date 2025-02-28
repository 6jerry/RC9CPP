
/*
 * @author loopgad
 * @contact 3280646246@qq.com
 * @license MIT License
 *
 * Copyright (c) 2024 loopgad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <limits>
#include <algorithm>
#include <iomanip>
using namespace std;

// 自定义输出函数
std::ostream& operator<<(std::ostream& os, const std::vector<Eigen::Matrix<double, 2, 1>>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        if (i > 0) os << ", ";
        os << vec[i].transpose();  // 输出矩阵的转置（方便查看）
    }
    os << "]";
    return os;
}

pair<vector<Eigen::Vector2d>, vector<double>> position_control_mpc(
    const Eigen::Vector2d& state, // [x, y]
    const Eigen::Vector2d& target_pos, // 目标位置 [x, y]
    double time_step, // 时间步长
    int prediction_horizon = 10, // 预测时域长度
    double max_vx = 2.7, // 最大线速度
    double max_vy = 2.7, // 最大 y 方向速度
    double max_acc = 10.0   // 最大加速度
) {
    // 预测轨迹和控制输入的初始化
    vector<Eigen::Vector2d> predicted_trajectory;

    // 计算目标位置误差
    Eigen::Vector2d position_error = target_pos - state;
    double distance = position_error.norm() / 0.2;
    double distance_ = position_error.norm();
    double threshold = 0.10; // 误差阈值

    // 如果目标位置非常接近，则启用加速度受限的速度衰减
    static Eigen::Vector2d previous_velocity = Eigen::Vector2d::Zero(); // 静态变量，保存前一个时间步长的速度
    Eigen::Vector2d current_velocity = Eigen::Vector2d::Zero();

    if (distance_ < threshold) {
        Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero(); // 目标速度为零

        // 计算速度变化（加速度）
        Eigen::Vector2d velocity_change = desired_velocity - previous_velocity;
        // 限制加速度不超过最大加速度
        double acceleration_norm = velocity_change.norm();
        if (acceleration_norm > max_acc * time_step) {
            // 缩放加速度以满足最大加速度约束
            velocity_change = (velocity_change / acceleration_norm) * max_acc * time_step;
        }

        current_velocity = previous_velocity + velocity_change;

        // 更新前一个速度
        previous_velocity = current_velocity;
    } else {
        // 合成目标速度矢量 v_target
        Eigen::Vector2d v_target = position_error.normalized() * std::min(distance, max_vx);  // 限制最大速度

        // 计算速度变化（加速度）
        Eigen::Vector2d velocity_change = v_target - previous_velocity;
        // 限制加速度不超过最大加速度
        double acceleration_norm = velocity_change.norm();
        if (acceleration_norm > max_acc * time_step) {
            // 缩放加速度以满足最大加速度约束
            velocity_change = (velocity_change / acceleration_norm) * max_acc * time_step;
        }

        current_velocity = previous_velocity + velocity_change;

        // 更新前一个速度
        previous_velocity = current_velocity;
    }

    // 分解合成速度矢量为 vx 和 vy
    double vx = current_velocity(0);
    double vy = current_velocity(1);

    // 预测轨迹
    vector<Eigen::Vector2d> trajectory;
    Eigen::Vector2d predicted_state = state;
    for (int step = 0; step < prediction_horizon; ++step) {
        predicted_state(0) += vx * time_step;
        predicted_state(1) += vy * time_step;
        trajectory.push_back(predicted_state);
    }

    // 计算代价函数：位置误差的平方和
    double cost = 0.0;
    for (int i = 0; i < prediction_horizon; ++i) {
        Eigen::Vector2d position_error = target_pos - trajectory[i];
        cost += position_error.squaredNorm();  // 只考虑位置误差的平方和
    }

    // 选择最小代价对应的控制输入
    double best_vx = vx;
    double best_vy = vy;
    if (cost < std::numeric_limits<double>::infinity()) {
        best_vx = vx;
        best_vy = vy;
    }

    // 返回控制输入
    vector<double> control = {best_vx, best_vy};

    // 返回轨迹和控制输入
    return {trajectory, control};
}

// RRT* 节点定义
struct Node {
    Eigen::Vector2d position;
    Node* parent;
    double cost;

    Node(const Eigen::Vector2d& pos, Node* p = nullptr, double c = 0.0)
        : position(pos), parent(p), cost(c) {}
};

// RRT* 算法实现
class RRTStarPlanner {
public:
    RRTStarPlanner(const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double max_dist = 1.0, double max_iter = 2000)
        : start_(start), goal_(goal), max_dist_(max_dist), max_iter_(max_iter) {
        start_bound_ = min(min(start(0), goal(0)), min(start(1), goal(1))) - 1.0;
        end_bound_ = max(max(start(0), goal(0)), max(start(1), goal(1))) + 1.0;
        tree_.emplace_back(new Node(start));
    }

    vector<Eigen::Vector2d> plan() {
        for (int i = 0; i < max_iter_; ++i) {
            Eigen::Vector2d random_point = sample();
            Node* nearest_node = findNearest(random_point);
            Node* new_node = generateNewNode(nearest_node, random_point);

            if (new_node != nullptr) {
                addNode(new_node);
            }
        }

        return findPath();
    }

private:
    Eigen::Vector2d sample() {
        double r = rand() / static_cast<double>(RAND_MAX);
        if (r < 0.1) {
            return goal_;  // 以10%的概率采样目标点
        } else {
            return Eigen::Vector2d(randUniform(start_bound_, end_bound_), randUniform(start_bound_, end_bound_));
        }
    }

    Node* findNearest(const Eigen::Vector2d& point) {
        Node* nearest = tree_[0];
        double min_dist = (point - nearest->position).norm();
        for (size_t i = 1; i < tree_.size(); ++i) {
            double dist = (point - tree_[i]->position).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest = tree_[i];
            }
        }
        return nearest;
    }

    Node* generateNewNode(Node* nearest, const Eigen::Vector2d& point) {
        Eigen::Vector2d dir = point - nearest->position;
        double dist = dir.norm();
        if (dist < 1e-6) {
            return nullptr;
        }
        dir /= dist;

        Eigen::Vector2d new_pos = nearest->position + dir * min(max_dist_, dist);
        if (nearGoal(new_pos)) {
            bool found = false;
            for (Node* node : tree_) {
                if ((node->position - goal_).norm() < 1e-6) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                return new Node(goal_, nearest, nearest->cost + (new_pos - nearest->position).norm());
            } else {
                return nullptr;
            }
        }
        return new Node(new_pos, nearest, nearest->cost + (new_pos - nearest->position).norm());
    }

    void addNode(Node* new_node) {
        tree_.push_back(new_node);
        optimzeNearNodes(new_node);
    }

    bool nearGoal(const Eigen::Vector2d& pos) {
        return (pos - goal_).norm() < 0.2; // 目标点附近
    }

    void optimzeNearNodes(Node* new_node) {
        const double radius = 0.5;
        vector<Node*> near_nodes;
        for (Node* node : tree_) {
            if (node != new_node && (node->position - new_node->position).norm() < radius) {
                near_nodes.push_back(node);
            }
        }

        for (Node* near_node : near_nodes) {
            double cost = new_node->cost + (near_node->position - new_node->position).norm();
            if (cost < near_node->cost) {
                near_node->parent = new_node;
                near_node->cost = cost;
            }
        }
    }

    vector<Eigen::Vector2d> findPath() {
        Node* best_node = tree_[0];
        for (Node* node : tree_) {
            if (nearGoal(node->position) && node->cost < best_node->cost) {
                best_node = node;
            }
        }

        vector<Eigen::Vector2d> path;
        Node* current = best_node;
        while (current != nullptr) {
            path.push_back(current->position);
            current = current->parent;
        }

        // 反转路径，得到从起点到终点的路径
        reverse(path.begin(), path.end());
        return path;
    }

    double randUniform(double min_val, double max_val) {
        return min_val + static_cast<double>(rand()) / RAND_MAX * (max_val - min_val);
    }

    Eigen::Vector2d start_;
    Eigen::Vector2d goal_;
    double max_dist_;
    int max_iter_;
    vector<Node*> tree_;
    double start_bound_, end_bound_;
};

// 路径规划函数（RTT* 实现）
vector<Eigen::Vector2d> rtt_planner(const Eigen::Vector2d& current_pos, const Eigen::Vector2d& target_pos) {
    double distance = (current_pos - target_pos).norm();
    if (distance < 0.5) { // 当当前位置靠近目标点时，直接返回目标点作为路径
        return {target_pos};
    } else {
        RRTStarPlanner planner(current_pos, target_pos, 0.2, 1000);
        auto path = planner.plan();
        if (path.size() < 2) {
            return {target_pos};
        } else {
            vector<Eigen::Vector2d> waypoints;
            for (size_t i = 1; i < path.size(); ++i) {
                waypoints.push_back(path[i]);
            }
            return waypoints;
        }
    }
}

vector<double> compute_control_velocities(
    const Eigen::Vector2d& state,
    const Eigen::Vector2d& original_target_pos,
    double time_step,
    bool& is_path_planned,
    vector<Eigen::Vector2d>& path_points,
    int& current_path_index
) {
    // 控制逻辑
    Eigen::Vector2d current_target;

    if (!is_path_planned) {
        double distance_to_original = (original_target_pos - state).norm();
        if (distance_to_original > 1.5) {
            path_points = rtt_planner(state, original_target_pos);
            if (!path_points.empty()) {
                current_path_index = 0;
                is_path_planned = true;
            }
        }
    }

    if (is_path_planned) {
        current_target = path_points[current_path_index];
    } else {
        current_target = original_target_pos;
    }

    auto [trajectory, control] = position_control_mpc(state, current_target, time_step);

    // 检查是否到达当前路径点（路径规划模式）
    if (is_path_planned) {
        double distance_to_current_target = (current_target - state).norm();
        if (distance_to_current_target < 0.1) {
            current_path_index++;
            if (current_path_index >= path_points.size()) {
                is_path_planned = false;
            }
        }
    }

    // 如果到达原始目标点
    if (!is_path_planned && (original_target_pos - state).norm() < 0.05) {
        cout << "Target reached." << endl;
        control = {0.0, 0.0}; // 停止运动
    }

    return control;
}

// 测试函数
int main() {
    // 输入参数
    Eigen::Vector2d state(0.0, 0.0);          // 起始状态
    Eigen::Vector2d original_target_pos(3.0, 4.5); // 原始目标位置
    double time_step = 0.01;                  // 时间步长
    bool is_path_planned = false;             // 路径规划是否已执行
    vector<Eigen::Vector2d> path_points;      // 路径点
    int current_path_index = 0;               // 当前路径点索引

    // 控制循环
    int max_steps = 300; // 最大步数
    for (int step = 0; step < max_steps; ++step) {
        vector<double> control = compute_control_velocities(
            state,
            original_target_pos,
            time_step,
            is_path_planned,
            path_points,
            current_path_index
        );

        // 输出控制速度（vx, vy）
        cout << "Step " << step << ": velocities ("
             << control[0] << ", " << control[1] << ")" << endl;

        // **状态更新**：根据控制输入更新状态
        state(0) += control[0] * time_step; // 更新x位置
        state(1) += control[1] * time_step; // 更新y位置
    }

    // 输出最终位置
    cout << "Final Position: ("
         << state(0) << ", " << state(1) << ")" << endl;

    return 0;
}