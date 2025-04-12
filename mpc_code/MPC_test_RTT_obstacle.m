% @author loopgad
% @contact 3280646246@qq.com
% @license MIT License
%
% Copyright (c) 2024 loopgad
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

function main()
    % 初始化参数
    state = [0, 0]; % [x, y]
    original_target_pos = [3, 4.5]; % 原始目标位置
    time_step = 0.1; % 时间步长
    max_steps = 100; % 最大步数
    max_acc = 10; % 最大加速度
    max_v = 2.7; % 最大线速度
    prediction_horizon = 10; % 预测时域长度
    threshold = 0.1; % 误差阈值

    % 障碍物参数 [x, y, radius]
    obstacles = [1.5, 2.0, 0.4;
                 2.5, 3.8, 0.3;
                 3.0, 2.0, 0.2]; 

    % 绘图初始化
    figure;
    hold on;
    grid on;
    axis([-1, 5, -1, 6]);
    axis equal;
    
    % 绘制障碍物
    for i = 1:size(obstacles,1)
        drawCircle(obstacles(i,1), obstacles(i,2), obstacles(i,3));
    end

    % 存储轨迹
    trajectory = [];

    % 路径规划相关变量
    is_path_planned = false;
    path_points = [];
    current_path_index = 0;
    previous_velocity = [0, 0]; 

    % 控制循环
    for step = 1:max_steps
        % 绘制目标点
        plot(original_target_pos(1), original_target_pos(2), 'k*', 'MarkerSize', 10, 'LineWidth', 2);

        % 调用路径规划和MPC控制
        [current_target, path_points, current_path_index, is_path_planned] = ...
            updateControl(state, original_target_pos, is_path_planned, path_points, current_path_index, obstacles);

        % 计算控制速度
        [vx, vy, previous_velocity] = computeMPC(state, current_target, time_step, max_v, max_acc, previous_velocity, prediction_horizon, threshold);

        % 状态更新
        state = state + [vx, vy] * time_step;

        % 存储轨迹
        trajectory = [trajectory; state];

        % 绘制当前状态
        scatter(state(1), state(2), 'filled');
        drawnow;
        pause(0.01);

        % 检查是否到达目标点
        if norm(state - original_target_pos) < 0.05
            disp('Target reached!');
            break;
        end
    end

    % 绘制最终轨迹
    plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
    title('Dynamic Path Tracking with Obstacle Avoidance');
    legend('Target', 'Robot Position', 'Trajectory');
end

function drawCircle(x, y, r)
    % 绘制圆形障碍物
    theta = 0:0.1:2*pi;
    x_pos = x + r*cos(theta);
    y_pos = y + r*sin(theta);
    fill(x_pos, y_pos, 'r', 'EdgeColor', 'none');
end

function [current_target, path_points, current_path_index, is_path_planned] = ...
    updateControl(state, original_target_pos, is_path_planned, path_points, current_path_index, obstacles)
    % 更新路径规划状态
    if ~is_path_planned
        distance_to_original = norm(original_target_pos - state);
        if distance_to_original > 1.5
            [path_points, success] = rrt_planner(state, original_target_pos, obstacles);
            if success
                current_path_index = 1;
                current_target = path_points(current_path_index, :);
                is_path_planned = true;
            else
                current_target = original_target_pos;
                is_path_planned = false;
            end
        else
            current_target = original_target_pos;
            is_path_planned = false;
        end
    else
        current_target = path_points(current_path_index, :);
        distance_to_current = norm(state - current_target);
        if distance_to_current < 0.1
            current_path_index = current_path_index + 1;
            if current_path_index > size(path_points, 1)
                is_path_planned = false;
                current_target = original_target_pos;
            else
                current_target = path_points(current_path_index, :);
            end
        end
    end
end

function [vx, vy, previous_velocity] = computeMPC(state, current_target, time_step, max_v, max_acc, previous_velocity, prediction_horizon, threshold)
    % MPC速度控制
    position_error = current_target - state;
    distance = norm(position_error);

    if distance < threshold
        desired_velocity = [0, 0];
        velocity_change = desired_velocity - previous_velocity;
        acceleration_norm = norm(velocity_change);
        if acceleration_norm > max_acc * time_step
            velocity_change = (velocity_change / acceleration_norm) * max_acc * time_step;
        end
        current_velocity = previous_velocity + velocity_change;
    else
        v_target = (position_error / norm(position_error)) * min(distance / 0.2, max_v);
        velocity_change = v_target - previous_velocity;
        acceleration_norm = norm(velocity_change);
        if acceleration_norm > max_acc * time_step
            velocity_change = (velocity_change / acceleration_norm) * max_acc * time_step;
        end
        current_velocity = previous_velocity + velocity_change;
    end

    previous_velocity = current_velocity;
    vx = current_velocity(1);
    vy = current_velocity(2);
end

function [path_points, success] = rrt_planner(start_pos, target_pos, obstacles)
    % RRT*路径规划核心
    max_dist = 0.1;
    max_iter = 200;
    success = false;
    tree = struct('position', {}, 'parent', {}, 'cost', {});
    tree(1).position = start_pos;
    tree(1).parent = [];
    tree(1).cost = 0;

    for iter = 1:max_iter
        % 安全采样
        if rand() < 0.1
            random_point = target_pos;
        else
            while true
                random_point = [rand()*5, rand()*5];
                if ~checkCollisionPoint(random_point, obstacles)
                    break;
                end
            end
        end

        % 最近节点搜索
        nearest_node_idx = findNearest(tree, random_point);
        nearest_node = tree(nearest_node_idx);

        % 节点扩展
        [new_node, is_goal] = generateNewNode(nearest_node, nearest_node_idx, random_point, max_dist, target_pos, obstacles);

        if ~isempty(new_node)
            % 添加新节点
            new_node.parent = nearest_node_idx;
            new_node.cost = nearest_node.cost + norm(new_node.position - nearest_node.position);
            tree(end+1) = new_node;
            
            % 优化树结构
            tree = optimizeNearNodes(tree, new_node, max_dist, obstacles);
            
            if is_goal
                success = true;
                break;
            end
        end
    end

    % 路径提取
    if success
        path_points = extractPath(tree, target_pos);
    else
        path_points = target_pos;
    end
end

function idx = findNearest(tree, point)
    % 最近节点查找
    min_dist = inf;
    idx = 1;
    for i = 1:length(tree)
        node = tree(i);
        dist = norm(node.position - point);
        if dist < min_dist
            min_dist = dist;
            idx = i;
        end
    end
end

function collision = checkCollisionLineSeg(start_pt, end_pt, obstacles)
    % 线段碰撞检测
    collision = false;
    num_checks = 10;
    for t = linspace(0, 1, num_checks)
        check_pt = start_pt + t*(end_pt - start_pt);
        if checkCollisionPoint(check_pt, obstacles)
            collision = true;
            return;
        end
    end
end

function collision = checkCollisionPoint(point, obstacles)
    % 点碰撞检测
    collision = false;
    for i = 1:size(obstacles,1)
        dist = norm(point - obstacles(i,1:2));
        if dist < obstacles(i,3) + 0.1
            collision = true;
            return;
        end
    end
end

function [new_node, is_goal] = generateNewNode(nearest_node, nearest_node_idx, random_point, max_dist, target_pos, obstacles)
    % 安全节点生成
    new_node = [];
    is_goal = false;
    dir = random_point - nearest_node.position;
    dist = norm(dir);
    
    if dist < 1e-6 || checkCollisionPoint(nearest_node.position, obstacles)
        return;
    end
    
    dir = dir / dist;
    new_pos = nearest_node.position + dir * min(max_dist, dist);
    
    % 路径安全性检查
    if checkCollisionLineSeg(nearest_node.position, new_pos, obstacles)
        return;
    end
    
    % 目标检测
    if norm(new_pos - target_pos) < 0.2
        new_node.position = target_pos;
        new_node.parent = nearest_node_idx;
        new_node.cost = nearest_node.cost + norm(new_pos - nearest_node.position);
        is_goal = true;
    else
        new_node.position = new_pos;
        new_node.parent = nearest_node_idx;
        new_node.cost = nearest_node.cost + norm(new_pos - nearest_node.position);
    end
end

function tree = optimizeNearNodes(tree, new_node, max_dist, obstacles)
    % 树结构优化
    radius = 0.5;
    new_idx = length(tree);
    
    for i = 1:new_idx-1
        node = tree(i);
        if norm(node.position - new_node.position) < radius && ...
           ~checkCollisionLineSeg(node.position, new_node.position, obstacles)
           
            tentative_cost = new_node.cost + norm(node.position - new_node.position);
            if tentative_cost < node.cost
                node.parent = new_idx;
                node.cost = tentative_cost;
                tree(i) = node;
            end
        end
    end
end

function path_points = extractPath(tree, target_pos)
    % 路径提取
    goal_idx = find(arrayfun(@(n) norm(n.position - target_pos) < 0.2, tree), 1, 'last');
    if isempty(goal_idx)
        path_points = target_pos;
        return;
    end
    
    path = [];
    current_idx = goal_idx;
    while ~isempty(current_idx) && ~isempty(tree(current_idx).parent)
        path = [tree(current_idx).position; path];
        current_idx = tree(current_idx).parent;
    end
    path = [tree(1).position; path];
    
    path_points = path;
end