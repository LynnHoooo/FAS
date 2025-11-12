%% ===================================================================
%  单独轨迹优化测试脚本
%  目标：不运行完整AO循环，仅测试轨迹优化子问题
%  步骤：
%   1) 运行 initial.m 获得基础参数和初始轨迹
%   2) 加载波束优化结果 (W_test, R_test) 
%   3) 调用 optimize_trajectory_SCA_TR 优化轨迹
%   4) 对比优化前后性能，绘制轨迹变化
%% ===================================================================

clear; clc; close all;

%% 0. 设置路径
setup_paths; % 添加所有必要路径

%% 1. 基础数据准备
fprintf('🚀 单独轨迹优化测试\n');
fprintf('步骤1: 运行 initial.m 准备基础数据...\n');
initial; % 生成所有基础参数
q_init = q_traj; % 保存初始轨迹
fprintf('✅ 初始化完成。\n');

%% 2. 加载波束优化结果  
mat_file = 'data/beamforming_feasibility_results.mat';
fprintf('\n步骤2: 加载波束优化结果 %s ...\n', mat_file);
if ~isfile(mat_file)
    error('找不到 %s，请先运行 test_beamforming_feasibility.m', mat_file);
end

data_beam = load(mat_file);
W_used = data_beam.W_test;
R_used = data_beam.R_test; 
alpha_used = alpha_init; % 使用初始关联
fprintf('✅ 波束数据加载完成。\n');

%% 3. 构造轨迹优化参数
fprintf('\n步骤3: 准备轨迹优化参数...\n');

% 参数结构体
params = struct();
params.M = M; params.K = K; params.N = N;
params.Na = Na; params.d = d; params.kappa = kappa;
params.sigma2 = sigma2; params.dt = dt; params.Vmax = Vmax;
params.v = v; params.H_sense = H_sense;
params.Dmin = Dmin; params.min_trust_region = 1;

% 信道增益
h_mkn_gain = cell(M, K, N);
for m = 1:M
    for k = 1:K
        for n = 1:N
            h_mkn_gain{m,k,n} = norm(h_mkn{m,k,n})^2;
        end
    end
end

% 从协方差矩阵恢复波束向量
w_mkn = cell(M, K, N);
for m = 1:M
    for k = 1:K
        for n = 1:N
            W_tmp = W_used{m,k,n};
            if ~isempty(W_tmp) && any(W_tmp(:))
                W_tmp = (W_tmp + W_tmp')/2;
                [V, D] = eig(W_tmp);
                [lambda_max, idx_max] = max(real(diag(D)));
                if lambda_max > 1e-8
                    w_mkn{m,k,n} = V(:,idx_max) * sqrt(lambda_max);
                else
                    w_mkn{m,k,n} = zeros(Na,1);
                end
            else
                w_mkn{m,k,n} = zeros(Na,1);
            end
        end
    end
end

% 轨迹优化参数
gamma_min_SINR = db2pow(5);   
trust_region0 = 20;           % 修改为20m，确保 > Vmax*dt = 15m
max_iter_traj = 15;
tol_traj = 1e-3;

% 自适应调整Gamma (与AO版本保持一致)
min_sensing_power_current = inf;
for n = 1:N
    for q_idx = 1:size(v,1)
        total_power = 0;
        for m = 1:M
            X_cov = R_used{m,1,n};
            for k = 1:K
                if ~isempty(W_used{m,k,n})
                    X_cov = X_cov + W_used{m,k,n};
                end
            end
            dx = v(q_idx,1) - u(m,1);
            dy = v(q_idx,2) - u(m,2);
            dist_sq = dx^2 + dy^2 + H_sense^2;
            if dist_sq > 1e-6
                dist = sqrt(dist_sq);
                cos_theta = H_sense / dist;
                phi = 2*pi*d * cos_theta;
                a_vec = exp(1j * phi * (0:Na-1)');
                total_power = total_power + real(a_vec' * X_cov * a_vec) / dist_sq;
            end
        end
        min_sensing_power_current = min(min_sensing_power_current, total_power);
    end
end

Gamma_adaptive = min_sensing_power_current * 0.8;
fprintf('  📋 使用自适应 Gamma: %.4e W (%.1f dBW)\n', Gamma_adaptive, 10*log10(Gamma_adaptive));
fprintf('  📋 轨迹优化参数: Vmax=%.1f m/s, 单步限制=%.1f m, 信任域=%.1f m, 最大迭代=%d\n', ...
    Vmax, Vmax*dt, trust_region0, max_iter_traj);

%% 4. 单独调用轨迹优化
fprintf('\n步骤4: 调用单次轨迹优化...\n');
tic;
[q_opt, trust_region_final] = optimize_trajectory_SCA_TR( ...
    q_init, h_mkn_gain, alpha_used, w_mkn, R_used, u, H, params, ...
    gamma_min_SINR, Gamma_adaptive, max_iter_traj, tol_traj, trust_region0, 1); % verbose=1
optimization_time = toc;

fprintf('✅ 轨迹优化完成，耗时: %.2f 秒\n', optimization_time);
fprintf('   最终信任域半径: %.2f m\n', trust_region_final);

%% 5. 性能评估
fprintf('\n步骤5: 性能评估...\n');

% 5.1 计算优化前性能 
sum_rate_before = compute_sum_rate(h_mkn, W_used, R_used, alpha_used, sigma2, M, K, N);

% 5.2 重新计算优化后的信道
h_mkn_new = cell(M, K, N);
for m = 1:M
    for k = 1:K
        for n = 1:N
            h_mkn_new{m,k,n} = get_channel(m, k, n, u, q_opt, H, kappa, d, Na);
        end
    end
end

% 5.3 计算优化后性能
sum_rate_after = compute_sum_rate(h_mkn_new, W_used, R_used, alpha_used, sigma2, M, K, N);
min_sense_power = compute_min_sensing_power(W_used, R_used, u, v, H_sense, Na);

% 轨迹变化统计
trajectory_change = norm(q_opt(:) - q_init(:)) / norm(q_init(:) + 1e-8);
max_position_change = 0;
for k = 1:K
    for n = 1:N
        pos_change = norm(squeeze(q_opt(k,:,n) - q_init(k,:,n)));
        max_position_change = max(max_position_change, pos_change);
    end
end

% 输出结果
fprintf('\n📊 === 轨迹优化结果 ===\n');
fprintf('通信性能:\n');
fprintf('  和速率 (优化前): %.4f bps/Hz\n', sum_rate_before);
fprintf('  和速率 (优化后): %.4f bps/Hz\n', sum_rate_after);
fprintf('  性能提升: %.4f bps/Hz (%.2f%%)\n', ...
    sum_rate_after - sum_rate_before, ...
    (sum_rate_after - sum_rate_before) / sum_rate_before * 100);

fprintf('\n感知性能:\n');
fprintf('  最小感知功率: %.4e W (%.1f dBW)\n', min_sense_power, 10*log10(min_sense_power));
fprintf('  感知阈值 Gamma: %.4e W (%.1f dBW)\n', Gamma_adaptive, 10*log10(Gamma_adaptive));
if min_sense_power >= Gamma_adaptive
    constraint_status = '✅ 是';
else
    constraint_status = '❌ 否';
end
fprintf('  约束满足: %s\n', constraint_status);

fprintf('\n轨迹变化:\n');
fprintf('  轨迹总体相对变化: %.4f%%\n', trajectory_change * 100);
fprintf('  单点最大位移: %.2f m\n', max_position_change);

%% 6. 可视化对比
fprintf('\n步骤6: 绘制轨迹对比图...\n');
figure('Position', [100, 100, 1200, 500]);

% 子图1: 水平轨迹对比
subplot(1,2,1); hold on; grid on; axis equal; axis([0 400 0 400]);
plot(u(:,1), u(:,2), 'ks', 'MarkerFaceColor','y', 'MarkerSize',10, 'DisplayName','GBS');

% 初始轨迹
plot(squeeze(q_init(1,1,:)), squeeze(q_init(1,2,:)), 'b--o', 'LineWidth',1.5, 'DisplayName','UAV1 初始');
plot(squeeze(q_init(2,1,:)), squeeze(q_init(2,2,:)), 'r--s', 'LineWidth',1.5, 'DisplayName','UAV2 初始');

% 优化后轨迹
plot(squeeze(q_opt(1,1,:)), squeeze(q_opt(1,2,:)), 'b-', 'LineWidth',3, 'DisplayName','UAV1 优化');
plot(squeeze(q_opt(2,1,:)), squeeze(q_opt(2,2,:)), 'r-', 'LineWidth',3, 'DisplayName','UAV2 优化');

% 感知区域
scatter(v(:,1), v(:,2), 60, 'g', 'filled', 'DisplayName','感知点');

% 起点和终点标记
plot(q_init(1,1,1), q_init(1,2,1), 'bo', 'MarkerSize',10, 'MarkerFaceColor','b', 'DisplayName','起点');
plot(q_init(1,1,N), q_init(1,2,N), 'bs', 'MarkerSize',10, 'MarkerFaceColor','b', 'DisplayName','终点');

title('单次轨迹优化结果 (水平面)', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)');
legend('Location', 'bestoutside');

% 子图2: 位移变化
subplot(1,2,2); hold on; grid on;
displacement_uav1 = zeros(1, N);
displacement_uav2 = zeros(1, N);
for n = 1:N
    displacement_uav1(n) = norm(squeeze(q_opt(1,:,n) - q_init(1,:,n)));
    displacement_uav2(n) = norm(squeeze(q_opt(2,:,n) - q_init(2,:,n)));
end

plot(1:N, displacement_uav1, 'b-o', 'LineWidth',2, 'DisplayName','UAV1 位移');
plot(1:N, displacement_uav2, 'r-s', 'LineWidth',2, 'DisplayName','UAV2 位移');
plot([1,N], [trust_region0, trust_region0], 'k--', 'DisplayName','信任域限制');

title('各时隙的位置变化', 'FontSize', 14);
xlabel('时隙 n'); ylabel('位移 (m)');
legend('Location', 'best');

fprintf('✅ 可视化完成。\n');

%% 7. 保存结果
save('data/single_trajectory_results.mat', 'q_init', 'q_opt', 'sum_rate_before', 'sum_rate_after', ...
     'min_sense_power', 'trajectory_change', 'optimization_time', 'trust_region_final');
fprintf('✅ 结果已保存到 data/single_trajectory_results.mat\n');

fprintf('\n🎯 单独轨迹优化测试完成！\n');

%% ====================== 辅助函数 ======================
function sum_rate = compute_sum_rate(h_mkn_cell, W_cell, R_cell, alpha_mkn, sigma2, M, K, N)
    sum_rate = 0;
    for n = 1:N
        for k = 1:K
            m_serv = find(alpha_mkn(:,k,n) == 1, 1);
            if isempty(m_serv)
                continue;
            end
            h_serv = h_mkn_cell{m_serv,k,n};
            signal_power = real(h_serv' * W_cell{m_serv,k,n} * h_serv);
            interference = 0;
            for m = 1:M
                for i = 1:K
                    if m == m_serv && i == k
                        continue;
                    end
                    interference = interference + real(h_mkn_cell{m,k,n}' * W_cell{m,i,n} * h_mkn_cell{m,k,n});
                end
                if ~isempty(R_cell{m,1,n})
                    interference = interference + real(h_mkn_cell{m,k,n}' * R_cell{m,1,n} * h_mkn_cell{m,k,n});
                end
            end
            SINR = signal_power / (interference + sigma2);
            sum_rate = sum_rate + log2(1 + max(SINR, 0));
        end
    end
    sum_rate = sum_rate / N; % 平均和速率
end

function min_power = compute_min_sensing_power(W_cell, R_cell, u, v, H_sense, Na)
    M = size(W_cell,1);
    Q = size(v,1);
    N = size(W_cell,3);
    min_power = inf;
    for n = 1:N
        for q_idx = 1:Q
            total = 0;
            for m = 1:M
                X_cov = R_cell{m,1,n};
                for k = 1:size(W_cell,2)
                    if ~isempty(W_cell{m,k,n})
                        X_cov = X_cov + W_cell{m,k,n};
                    end
                end
                dx = v(q_idx,1) - u(m,1);
                dy = v(q_idx,2) - u(m,2);
                dist_sq = dx^2 + dy^2 + H_sense^2;
                if dist_sq <= 1e-6
                    continue;
                end
                dist = sqrt(dist_sq);
                cos_theta = H_sense / dist;
                a_vec = exp(1j * 2 * pi * 0.5 * cos_theta * (0:Na-1)');
                total = total + real(a_vec' * X_cov * a_vec) / (dist_sq + 1e-12);
            end
            min_power = min(min_power, total);
        end
    end
end
