clc;
clear all;

% 四轮X型全向底盘的运动学参数
wheel_radius = 0.1;  % 轮子半径
wheel_base = 0.5;    % 轮子间距
time_step = 0.01;     % 时间步长
prediction_horizon = 5;  % 预测范围
control_horizon = 5;      % 控制范围
num_wheels = 4;           % 轮子数量
dead_zone = 0.1;          % 死区半径

% 初始状态
state_mpc = [0.0, 0.0, 0.0];  % [x, y, theta]，theta为朝向角度

% 初始化路径数据
mpc_path_x = [state_mpc(1)];
mpc_path_y = [state_mpc(2)];

% 设置目标位置更新频率（每多少时间步更新一次目标位置）
track_time = 1000; % 设置仿真时长

% 设置正弦路径的参数
amplitude = 3.0;  % 正弦波幅度
frequency = 1;  % 正弦波频率
offset = 5.0;      % 正弦波的偏移量（控制路径的垂直位置）

% 初始目标位置
t = 0;  % 时间变量，用于生成正弦路径
target_pos = [t, amplitude * sin(frequency * t) + offset];

% 动态更新
figure; % 创建图形窗口
hold on;
plot(mpc_path_x, mpc_path_y, 'b', 'LineWidth', 2);  % 绘制完整路径
scatter(target_pos(1), target_pos(2), 70, 'r', 'filled');  % 绘制目标位置
axis equal;
xlim([-10, 10]);
ylim([-10, 10]);
grid on;

% 动态更新
for t = 1:track_time
    % 更新MPC状态
    u_mpc = mpc_control(state_mpc, target_pos, prediction_horizon, control_horizon, time_step);
    for i = 1:control_horizon
        vx = u_mpc(i);  % 线速度
        wz = u_mpc(i + control_horizon);  % 角速度
        state_mpc(1) = state_mpc(1) + vx * cos(state_mpc(3)) * time_step;
        state_mpc(2) = state_mpc(2) + vx * sin(state_mpc(3)) * time_step;
        state_mpc(3) = state_mpc(3) + wz * time_step;
    end
    mpc_path_x = [mpc_path_x, state_mpc(1)];
    mpc_path_y = [mpc_path_y, state_mpc(2)];
    
    % 绘制当前动点
    plot(state_mpc(1), state_mpc(2), 'bo', 'MarkerFaceColor','b');  % 动态绘制当前位置
    scatter(target_pos(1), target_pos(2), 70,'r', 'filled');  % 绘制目标位置
    target_pos = [t * time_step, amplitude * sin(frequency * t * time_step) + offset];  % 新目标点
    % 检查是否到达目标位置，如果到达目标则更新目标位置
    if norm(state_mpc(1:2) - target_pos) < dead_zone
        disp('Reached the target position. Updating target along the sine path.');
        
        % 生成新的目标点沿着正弦路径
        t = t + 1;  % 更新时间
        target_pos = [t * time_step, amplitude * sin(frequency * t * time_step) + offset];  % 新目标点
        amplitude = 3.0 + t/10000;  % 正弦波幅度
        frequency = 1 + t/500;  % 正弦波频率）
    end
end

% 完整的仿真路径
simulation_path = [mpc_path_x; mpc_path_y];

% 显示仿真路径
disp('Complete Simulation Path:');
disp(simulation_path);

% MPC优化目标函数
function cost = cost_function(u, state, target_pos, prediction_horizon, time_step)
    cost = 0;
    current_state = state;
    for i = 1:prediction_horizon
        % 运动学模型更新
        vx = u(i);  % 线速度
        wz = u(i + prediction_horizon);  % 角速度
        current_state(1) = current_state(1) + vx * cos(current_state(3)) * time_step;
        current_state(2) = current_state(2) + vx * sin(current_state(3)) * time_step;
        current_state(3) = current_state(3) + wz * time_step;
        % 目标位置误差
        cost = cost + norm(current_state(1:2) - target_pos)^2;
    end
end

% MPC控制器
function u = mpc_control(state, target_pos, prediction_horizon, control_horizon, time_step)
    u_initial = zeros(1, prediction_horizon * 2);  % 初始控制输入
    lb = -1 * ones(1, prediction_horizon * 2);  % 下界
    ub = 1 * ones(1, prediction_horizon * 2);   % 上界
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    u = fmincon(@(u) cost_function(u, state, target_pos, prediction_horizon, time_step), u_initial, [], [], [], [], lb, ub, [], options);
    u = u(1:control_horizon * 2);  % 确保返回的控制输入长度正确
end
