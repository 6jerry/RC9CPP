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

%% 交互式卡尔曼滤波仿真平台（修正版）
clear; clc; close all;
rng(20250225);

% 创建图形界面
fig = figure('Color','w', 'Position', [100,100,1000,700]);
ax1 = subplot(2,1,1); hold(ax1, 'on');
ax2 = subplot(2,1,2); hold(ax2, 'off');

% 初始化控件
noise_panel = uipanel(fig, 'Title','噪声强度调节', 'Position',[0.75 0.4 0.2 0.5]);
uicontrol(noise_panel, 'Style','text', 'String','传感器1:',...
          'Position',[10 160 80 20]);
uicontrol(noise_panel, 'Style','text', 'String','传感器2:',...
          'Position',[10 110 80 20]);
uicontrol(noise_panel, 'Style','text', 'String','传感器3:',...
          'Position',[10 60 80 20]);

% 定义全局变量以便回调函数访问
global sld1 sld2 sld3 h_obs h_filt ax2 mse_text t true_signal n_samples A Q;

sld1 = uicontrol(noise_panel, 'Style','slider', 'Min',0.1,'Max',3,'Value',0.3,...
                'Position',[80 160 120 20], 'Callback',@updateSim);
sld2 = uicontrol(noise_panel, 'Style','slider', 'Min',0.1,'Max',3,'Value',1.0,...
                'Position',[80 110 120 20], 'Callback',@updateSim);
sld3 = uicontrol(noise_panel, 'Style','slider', 'Min',0.1,'Max',3,'Value',2.0,...
                'Position',[80 60 120 20], 'Callback',@updateSim);

% 系统参数
t = 0:0.1:10;
true_signal = sin(8*t) + 3.5*cos(3*t) + 0.6*sin(0.8*t) - 0.9*cos(20*t);
n_samples = length(t);
A = 1;  Q = 0.8;

% 图形句柄初始化
h_true = plot(ax1, t, true_signal, 'b', 'LineWidth', 2);
h_obs = plot(ax1, t, zeros(3,n_samples), 'LineStyle','--');
h_filt = plot(ax1, t, zeros(1,n_samples), 'k', 'LineWidth', 2);
legend(ax1, {'原始信号','观测1','观测2','观测3','滤波结果'}, 'Location','northwest');
title(ax1, '实时滤波效果');
xlabel('时间 (s)'); ylabel('幅值');

% 初始化性能评估
mse_text = uicontrol(fig, 'Style','text', 'Position',[760 50 200 30],...
                    'String','当前MSE: -', 'FontSize',12);

% 首次运行
updateSim();

%% 回调函数实现动态更新
function updateSim(~,~)
    global sld1 sld2 sld3 h_obs h_filt ax2 mse_text t true_signal n_samples A Q;
    
    % 获取当前滑动条值
    noise_strength = [sld1.Value, sld2.Value, sld3.Value];
    R = diag(noise_strength.^2);
    H = ones(3,1);
    
    % 生成带噪声的观测
    noise = noise_strength' .* randn(3, n_samples);
    Z = true_signal + noise;
    
    % 执行卡尔曼滤波
    x_est = 0;  P_est = 1;
    filtered_signal = zeros(1, n_samples);
    for k = 1:n_samples
        x_pred = A * x_est;
        P_pred = A * P_est * A' + Q;
        S = H*P_pred*H' + R;
        K = P_pred*H' / S;
        x_est = x_pred + K*(Z(:,k) - H*x_pred);
        P_est = (1 - K*H)*P_pred;
        filtered_signal(k) = x_est;
    end
    
    % 更新图形
    set(h_obs(1), 'YData', Z(1,:));
    set(h_obs(2), 'YData', Z(2,:));
    set(h_obs(3), 'YData', Z(3,:));
    set(h_filt, 'YData', filtered_signal);
    
    % 计算性能指标
    mse = mean((true_signal - filtered_signal).^2);
    set(mse_text, 'String', sprintf('当前MSE: %.4f', mse));
    
    % 显示噪声强度分布
    cla(ax2);
    bar(ax2, noise_strength, 'FaceColor',[0.7 0.7 0.9]);
    title(ax2, '当前噪声强度分布');
    xlabel(ax2, '传感器编号'); ylabel(ax2, '噪声标准差');
    set(ax2, 'XTick',1:3, 'XLim',[0.5 3.5]);
end