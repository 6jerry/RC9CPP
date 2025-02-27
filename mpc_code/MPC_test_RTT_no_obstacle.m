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

    % 绘图初始化
    figure;
    hold on;
    grid on;
    axis([-1, 5, -1, 6]); % 设置坐标范围
    axis equal;

    % 存储轨迹
    trajectory = [];

    % 路径规划相关变量
    is_path_planned = false;
    path_points = [];
    current_path_index = 0;
    previous_velocity = [0, 0]; % 前一个时间步的速度

    % 控制循环
    for step = 1:max_steps
        % 绘制目标点
        plot(original_target_pos(1), original_target_pos(2), 'k*', 'MarkerSize', 10, 'LineWidth', 2);

        % 调用路径规划和MPC控制
        [current_target, path_points, current_path_index, is_path_planned] = ...
            updateControl(state, original_target_pos, is_path_planned, path_points, current_path_index);

        % 计算控制速度
        [vx, vy, previous_velocity] = computeMPC(state, current_target, time_step, max_v, max_acc, previous_velocity, prediction_horizon, threshold);

        % 状态更新
        state = state + [vx, vy] * time_step;

        % 存储轨迹
        trajectory = [trajectory; state];

        % 绘制当前状态
        scatter(state(1), state(2), 'filled');
        drawnow; % 更新图形
        pause(0.01); % 动画延时

        % 检查是否到达目标点
        if norm(state - original_target_pos) < 0.05
            disp('Target reached!');
            break;
        end
    end

    % 绘制最终轨迹
    plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
    title('Dynamic Path Tracking and Velocity Control');
    legend('Target', 'Robot Position', 'Trajectory');
end

function [current_target, path_points, current_path_index, is_path_planned] = ...
    updateControl(state, original_target_pos, is_path_planned, path_points, current_path_index)
    % 路径规划逻辑
    if ~is_path_planned
        distance_to_original = norm(original_target_pos - state);
        if distance_to_original > 1.5
            [path_points, success] = rrt_planner(state, original_target_pos);
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
        % 检查是否到达当前路径点
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
    % MPC 控制逻辑
    position_error = current_target - state;
    distance = norm(position_error);

    % 目标位置非常接近时的处理
    if distance < threshold
        desired_velocity = [0, 0];
        velocity_change = desired_velocity - previous_velocity;
        acceleration_norm = norm(velocity_change);
        if acceleration_norm > max_acc * time_step
            velocity_change = (velocity_change / acceleration_norm) * max_acc * time_step;
        end
        current_velocity = previous_velocity + velocity_change;
    else
        % 合成目标速度矢量
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

function [path_points, success] = rrt_planner(start_pos, target_pos)
    % RRT* 路径规划
    max_dist = 0.05;
    max_iter = 10;
    success = false;
    tree = struct('position', {}, 'parent', {}, 'cost', {}); % 初始化空结构体数组
    tree(1).position = start_pos; % 添加初始节点
    tree(1).parent = [];
    tree(1).cost = 0;

    for iter = 1:max_iter
        % 随机采样
        if rand() < 0.1
            random_point = target_pos;
        else
            random_point = [rand()*5, rand()*5]; % 假设地图范围为0-5
        end

        % 查找最近节点
        nearest_node_idx = findNearest(tree, random_point);
        nearest_node = tree(nearest_node_idx);

        % 生成新节点（传递 nearest_node_idx 参数）
        [new_node, is_goal] = generateNewNode(nearest_node, nearest_node_idx, random_point, max_dist, target_pos);

        if ~isempty(new_node)
            % 添加新节点到树
            new_node.parent = nearest_node_idx; % 存储父节点索引
            new_node.cost = nearest_node.cost + norm(new_node.position - nearest_node.position);
            tree(end+1) = new_node;
            
            % 优化附近节点
            tree = optimizeNearNodes(tree, new_node, max_dist);
            
            % 检查是否到达目标
            if is_goal
                success = true;
                break;
            end
        end
    end

    % 提取路径
    if success
        path_points = extractPath(tree, target_pos);
    else
        path_points = target_pos; % 直接返回目标点
    end
end

function idx = findNearest(tree, point)
    % 查找最近节点
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

function [new_node, is_goal] = generateNewNode(nearest_node, nearest_node_idx, random_point, max_dist, target_pos) % 添加参数
    % 生成新节点
    new_node = [];
    is_goal = false;
    dir = random_point - nearest_node.position;
    dist = norm(dir);
    
    if dist < 1e-6
        return;
    end
    
    dir = dir / dist;
    new_pos = nearest_node.position + dir * min(max_dist, dist);
    
    % 检查是否接近目标（使用传递的 nearest_node_idx）
    if norm(new_pos - target_pos) < 0.2
        new_node.position = target_pos;
        new_node.parent = nearest_node_idx; % 使用参数中的索引
        new_node.cost = nearest_node.cost + norm(new_pos - nearest_node.position);
        is_goal = true;
    else
        new_node.position = new_pos;
        new_node.parent = nearest_node_idx; % 使用参数中的索引
        new_node.cost = nearest_node.cost + norm(new_pos - nearest_node.position);
    end
end

function tree = optimizeNearNodes(tree, new_node, max_dist)
    % 优化附近节点（修正索引错误）
    radius = 0.5;
    new_idx = length(tree); % 新节点的正确索引
    
    for i = 1:new_idx-1 % 遍历除新节点外的所有节点
        node = tree(i);
        if norm(node.position - new_node.position) < radius
            tentative_cost = new_node.cost + norm(node.position - new_node.position);
            if tentative_cost < node.cost
                node.parent = new_idx; % 设置为新节点的索引
                node.cost = tentative_cost;
                tree(i) = node;
            end
        end
    end
end

function path_points = extractPath(tree, target_pos)
    % 提取路径（添加越界保护）
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
    path = [tree(1).position; path]; % 确保包含起始点
    
    path_points = path;
end