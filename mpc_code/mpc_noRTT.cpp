
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
    const Eigen::Vector2d& state,
    const Eigen::Vector2d& target_pos,
    double time_step,
    int prediction_horizon = 10,
    double max_vx = 2.7,
    double max_vy = 2.7
) {
    // 增加静态变量保存历史状态
    static Eigen::Vector2d prev_velocity(0, 0);

    vector<Eigen::Vector2d> predicted_trajectory;

    // 计算目标误差
    Eigen::Vector2d error = target_pos - state;
    double error_norm = error.norm();
    const double STOP_THRESHOLD = 0.1; // 停止阈值

    // 动态速度衰减因子
    double dynamic_decay = 2.5;
    if(error_norm < 0.5) {
        dynamic_decay = std::max(0.2, error_norm-0.1); // 距离越近速度衰减越大
    }

    // 计算理想速度
    Eigen::Vector2d ideal_velocity = error.normalized() * dynamic_decay * max_vx;

    // 增加加速度约束 (最大加速度 10 m/s²)
    const double MAX_ACCEL = 10.0;
    Eigen::Vector2d allowed_delta = ideal_velocity - prev_velocity;
    if(allowed_delta.norm() > MAX_ACCEL * time_step) {
        allowed_delta = allowed_delta.normalized() * MAX_ACCEL * time_step;
    }

    Eigen::Vector2d new_velocity = prev_velocity + allowed_delta;

    // 最终速度限幅
    new_velocity.x() = std::clamp(new_velocity.x(), 0.0, max_vx);
    new_velocity.y() = std::clamp(new_velocity.y(), 0.0, max_vy);

    // 更新历史速度
    prev_velocity = new_velocity;

    // 生成预测轨迹
    Eigen::Vector2d predicted_state = state;
    for(int i=0; i<prediction_horizon; ++i){
        predicted_state += new_velocity * time_step;
        predicted_trajectory.push_back(predicted_state);
    }

    // 提前停止判断
    if(error_norm < STOP_THRESHOLD){
        return {predicted_trajectory, {0, 0}};
    }

    return {predicted_trajectory, {new_velocity.x(), new_velocity.y()}};
}

// 主函数
int main() {
    // 输入参数
    Eigen::Vector2d state(0.0, 0.0); // [x, y]
    Eigen::Vector2d target_pos(3.1, 2.5); // 目标位置
    double time_step = 0.02;              // 时间步长

    // 控制循环
    int max_steps = 200; // 最大步数
    for (int step = 0; step < max_steps; ++step) {
        cout << "Step " << step << ":\n";

        // 计算控制输入和新的状态
        auto [trajectory, control] = position_control_mpc(state, target_pos, time_step);

        // 输出控制速度（vx, vy）
        cout << "Control Velocities: ("
             << control[0] << ", "  // vx
             << control[1] << ")\n"; // vy

        // **状态更新**：根据控制输入更新状态
        state(0) += control[0] * time_step; // 更新x位置
        state(1) += control[1] * time_step; // 更新y位置

        // 如果位置足够接近目标位置，就结束
        if ((target_pos - state).norm() < 0.05) {
            cout << "Target reached.\n";
            break;
        }
        target_pos += Eigen::Vector2d(0.01,0.01);

    }

    // 输出最终位置
    cout << "Final Position: ("
         << state(0) << ", "  // x
         << state(1) << ")\n"; // y
    // 输出最终位置
    cout << "Final Target: ("
         << target_pos(0) << ", "  // x
         << target_pos(1) << ")\n"; // y

    return 0;
}
