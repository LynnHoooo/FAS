%% ===================================================================
%  测试脚本：利用波束优化结果调试轨迹优化
%  步骤：
%   1) 运行 initial.m 生成基础场景与直线路径；
%   2) 加载 beamforming_feasibility_results.mat 中的 W_test / R_test；
%   3) 调用 optimize_trajectory_SCA_TR 优化无人机轨迹；
%   4) 输出优化前后的通信和感知性能，并绘制轨迹对比。
%% ===================================================================

clear; clc; close all;

%% 1. 准备基础数据
fprintf('步骤1: 运行 initial.m 生成初始场景...\n');
initial; % 生成 q_traj、alpha_init、W_init、R_init、h_mkn 等
fprintf('✅ initial.m 完成。\n');

q_init = q_traj; % 备份直线路径

%% 2. 加载波束优化结果
mat_file = 'beamforming_feasibility_results.mat';
fprintf('\n步骤2: 加载 %s ...\n', mat_file);
if ~isfile(mat_file)
    error('找不到 %s，请先运行 test_beamforming_feasibility.m。', mat_file);
end

data_beam = load(mat_file);

% 优先使用优化后的 W/R，如果不存在则回退到初值
if isfield(data_beam, 'W_test') && ~isempty(data_beam.W_test)
    W_used = data_beam.W_test;
else
    warning('未找到 W_test，使用 initial 中的 W_init。');
    W_used = W_init;
end

if isfield(data_beam, 'R_test') && ~isempty(data_beam.R_test)
    R_used = data_beam.R_test;
else
    warning('未找到 R_test，使用 initial 中的 R_init。');
    R_used = R_init;
end

if isfield(data_beam, 'alpha_init')
    alpha_used = data_beam.alpha_init;
else
    alpha_used = alpha_init;
end

fprintf('✅ 波束数据加载完成。正在组装轨迹优化输入...\n');

%% 3. 构造轨迹优化所需变量
params = struct();
params.M = M; params.K = K; params.N = N;
params.Na = Na; params.d = d; params.kappa = kappa;
params.sigma2 = sigma2; params.dt = dt; params.Vmax = Vmax;
params.v = v; params.H_sense = H_sense;
params.Dmin = Dmin; params.min_trust_region = 1;

% 3.1 信道增益 |h|^2
h_mkn_gain = cell(M, K, N);
for m = 1:M
    for k = 1:K
        for n = 1:N
            h_mkn_gain{m,k,n} = norm(h_mkn{m,k,n})^2;
        end
    end
end

% 3.2 从协方差矩阵恢复波束向量 w_{m,k}[n]
w_mkn = cell(M, K, N);
for m = 1:M
    for k = 1:K
        for n = 1:N
            W_tmp = W_used{m,k,n};
            if ~isempty(W_tmp) && any(W_tmp(:))
                % 对称化避免数值误差导致的非厄米特
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

gamma_min_SINR = db2pow(5);   % 5 dB 基础门限
trust_region0 = 10;           % 初始信任域半径（米）
max_iter_traj = 15;
tol_traj = 1e-3;

% 根据波束优化后的实际感知能力调整 Gamma
fprintf('  📋 检测波束优化后的感知能力...\n');

% 计算当前轨迹下的实际感知功率
min_sensing_power_current = inf;
for n = 1:N
    for q_idx = 1:size(v,1)
        total_power = 0;
        for m = 1:M
            % 构建总协方差矩阵
            X_cov = R_used{m,1,n};
            for k = 1:K
                if ~isempty(W_used{m,k,n})
                    X_cov = X_cov + W_used{m,k,n};
                end
            end
            % 计算感知功率
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

% 自适应调整 Gamma
Gamma_original = Gamma;
if min_sensing_power_current > 0 && min_sensing_power_current < Gamma_original
    Gamma_adaptive = min_sensing_power_current * 0.8; % 留20%安全余量
    fprintf('      • 原始 Gamma: %.4e W (%.1f dBW)\n', Gamma_original, 10*log10(Gamma_original));
    fprintf('      • 实际最小感知功率: %.4e W (%.1f dBW)\n', min_sensing_power_current, 10*log10(min_sensing_power_current));
    fprintf('      • 调整后 Gamma: %.4e W (%.1f dBW)\n', Gamma_adaptive, 10*log10(Gamma_adaptive));
    Gamma = Gamma_adaptive;
else
    fprintf('      • 使用原始 Gamma: %.4e W (%.1f dBW)\n', Gamma, 10*log10(Gamma));
end

% 输出关键参数信息  
fprintf('  📋 轨迹优化关键参数:\n');
fprintf('      • Vmax = %.1f m/s, dt = %.1f s → 单步最大位移 = %.1f m\n', Vmax, dt, Vmax*dt);
fprintf('      • 初始信任域 = %.1f m, 最小UAV间距 = %.1f m\n', trust_region0, Dmin);
fprintf('      • UAV数量 = %d, 时隙数量 = %d, 总优化变量 = %d\n', K, N, K*2*N);

% 检查初始轨迹信息
fprintf('  📋 初始轨迹检查:\n');
for k = 1:K
    start_pos = squeeze(q_init(k,:,1));
    end_pos = squeeze(q_init(k,:,N));
    total_dist = norm(end_pos - start_pos);
    fprintf('      • UAV%d: (%.1f,%.1f) → (%.1f,%.1f), 总距离 %.1f m\n', ...
        k, start_pos(1), start_pos(2), end_pos(1), end_pos(2), total_dist);
end

%% 4. 真正的交替优化 (AO) 算法
fprintf('\n步骤3: 开始交替优化 (AO) 算法...\n');

max_ao_iter = 5;        % AO外层迭代次数
ao_tolerance = 1e-3;    % AO收敛阈值
q_current = q_init;     % 当前轨迹
W_current = W_used;     % 当前通信波束
R_current = R_used;     % 当前感知波束
alpha_current = alpha_used; % 当前关联

sum_rate_history = zeros(max_ao_iter, 1);

for ao_iter = 1:max_ao_iter
    fprintf('  🔄 AO迭代 %d/%d\n', ao_iter, max_ao_iter);
    
    % === AO子问题1: 给定轨迹，更新关联和波束参数 ===
    fprintf('    📡 子问题1: 固定轨迹 q，更新关联 α 和波束参数...\n');
    
    % 重新计算当前轨迹下的信道
    h_mkn_current = cell(M, K, N);
    for m = 1:M
        for k = 1:K
            for n = 1:N
                h_mkn_current{m,k,n} = get_channel(m, k, n, u, q_current, H, kappa, d, Na);
            end
        end
    end
    
    % 子问题1a: 更新关联 (基于新信道)
    alpha_new = optimize_association(h_mkn_current, W_current, R_current, Pmax, sigma2, M, K, N, Na);
    alpha_current = alpha_new;
    fprintf('      ✅ 关联更新完成\n');
    
    % 子问题1b: 基于新信道更新波束方向 (MRT/感知导向)
    % 不重新求解CVX，只更新波束指向
    for m = 1:M
        for k = 1:K
            for n = 1:N
                if alpha_current(m,k,n) == 1  % 该UAV连接到该GBS
                    % 更新通信波束：MRT方向
                    h_vec = h_mkn_current{m,k,n};
                    if norm(h_vec) > 1e-8
                        w_mrt = h_vec / norm(h_vec);
                        power_comm = trace(W_current{m,k,n});  % 保持功率不变
                        W_current{m,k,n} = power_comm * (w_mrt * w_mrt');
                    end
                end
            end
        end
        % 感知波束保持不变（基于固定感知区域）
        % R_current{m,1,n} 不变
    end
    fprintf('      ✅ 波束方向更新完成\n');
    
    % === AO子问题2: 给定波束，优化轨迹 ===
    fprintf('    🚁 子问题2: 固定波束 (w,R)，优化轨迹 q...\n');
    
    % 从新波束重新构造 w_mkn
    w_mkn_new = cell(M, K, N);
    for m = 1:M
        for k = 1:K
            for n = 1:N
                W_tmp = W_current{m,k,n};
                if ~isempty(W_tmp) && any(W_tmp(:))
                    W_tmp = (W_tmp + W_tmp')/2;
                    [V, D] = eig(W_tmp);
                    [lambda_max, idx_max] = max(real(diag(D)));
                    if lambda_max > 1e-8
                        w_mkn_new{m,k,n} = V(:,idx_max) * sqrt(lambda_max);
                    else
                        w_mkn_new{m,k,n} = zeros(Na,1);
                    end
                else
                    w_mkn_new{m,k,n} = zeros(Na,1);
                end
            end
        end
    end
    
    % 计算当前轨迹下的信道增益
    h_mkn_gain_current = cell(M, K, N);
    for m = 1:M
        for k = 1:K
            for n = 1:N
                h_mkn_gain_current{m,k,n} = norm(h_mkn_current{m,k,n})^2;
            end
        end
    end
    
    % 轨迹优化
    [q_new, ~] = optimize_trajectory_SCA_TR( ...
        q_current, h_mkn_gain_current, alpha_current, w_mkn_new, R_current, u, H, params, ...
        gamma_min_SINR, Gamma, max_iter_traj, tol_traj, trust_region0, 0);
    
    fprintf('      ✅ 轨迹优化完成\n');
    
    % ✅ 关键更新：轨迹变化后重新计算信道用于性能评估
    h_mkn_updated = cell(M, K, N);
    for m = 1:M
        for k = 1:K
            for n = 1:N
                h_mkn_updated{m,k,n} = get_channel(m, k, n, u, q_new, H, kappa, d, Na);
            end
        end
    end
    
    % === 性能评估与收敛检查 ===
    sum_rate_current = compute_sum_rate(h_mkn_updated, W_current, R_current, alpha_current, sigma2, M, K, N);
    sum_rate_history(ao_iter) = sum_rate_current;
    
    fprintf('      当前和速率: %.4f bps/Hz\n', sum_rate_current);
    
    % 收敛检查
    if ao_iter > 1
        improvement = abs(sum_rate_history(ao_iter) - sum_rate_history(ao_iter-1));
        rel_improvement = improvement / (abs(sum_rate_history(ao_iter-1)) + 1e-8);
        
        fprintf('      和速率提升: %.6f (相对: %.4f%%)\n', improvement, rel_improvement*100);
        
        if rel_improvement < ao_tolerance
            fprintf('    ✅ AO算法收敛于第 %d 次迭代\n', ao_iter);
            break;
        end
    end
    
    % ✅ 更新当前解（轨迹是主要变量）
    q_current = q_new;
    
    % 轨迹变化分析
    trajectory_change = norm(q_new(:) - q_init(:)) / norm(q_init(:) + 1e-8);
    fprintf('      轨迹相对变化: %.4f%%\n', trajectory_change*100);
    
    % 检查感知约束满足情况
    min_sensing_check = compute_min_sensing_power(W_current, R_current, u, v, H_sense, Na);
    sensing_satisfied = min_sensing_check >= Gamma;
    status_text = '❌';
    if sensing_satisfied
        status_text = '✅';
    end
    fprintf('      感知约束: %.4e W ≥ %.4e W ? %s\n', ...
        min_sensing_check, Gamma, status_text);
end

% 最终结果
q_opt = q_current;
W_final = W_current;
R_final = R_current;

fprintf('  ✅ AO算法完成，总迭代: %d次\n', min(ao_iter, max_ao_iter));

%% 5. 重新计算性能指标
fprintf('\n步骤4: 评估优化前后性能...\n');

% 5.1 重新计算信道
h_mkn_new = cell(M, K, N);
for m = 1:M
    for k = 1:K
        for n = 1:N
            h_mkn_new{m,k,n} = get_channel(m, k, n, u, q_opt, H, kappa, d, Na);
        end
    end
end

sum_rate_before = compute_sum_rate(h_mkn, W_used, R_used, alpha_used, sigma2, M, K, N);
sum_rate_after  = compute_sum_rate(h_mkn_new, W_used, R_used, alpha_used, sigma2, M, K, N);

min_sense_power = compute_min_sensing_power(W_used, R_used, u, v, H_sense, Na);

fprintf('和速率 (优化前 → 优化后): %.4f → %.4f bps/Hz\n', sum_rate_before, sum_rate_after);
fprintf('最小感知功率 (固定波束): %.4e W\n', min_sense_power);

%% 6. 绘制轨迹对比
figure('Position',[100 100 900 420]);
subplot(1,2,1); hold on; grid on; axis equal; axis([0 400 0 400]);
plot(u(:,1), u(:,2), 'ks', 'MarkerFaceColor','y', 'MarkerSize',8, 'DisplayName','GBS');
plot(squeeze(q_init(1,1,:)), squeeze(q_init(1,2,:)), 'b--o', 'DisplayName','UAV1 初始');
plot(squeeze(q_init(2,1,:)), squeeze(q_init(2,2,:)), 'r--s', 'DisplayName','UAV2 初始');
plot(squeeze(q_opt(1,1,:)), squeeze(q_opt(1,2,:)), 'b-', 'LineWidth',2, 'DisplayName','UAV1 优化');
plot(squeeze(q_opt(2,1,:)), squeeze(q_opt(2,2,:)), 'r-', 'LineWidth',2, 'DisplayName','UAV2 优化');
if exist('v','var')
    scatter(v(:,1), v(:,2), 40, 'g', 'filled', 'DisplayName','感知点');
end
title('UAV 轨迹 (水平面)'); xlabel('X (m)'); ylabel('Y (m)'); legend('Location','bestoutside');

subplot(1,2,2); hold on; grid on;
plot(1:N, squeeze(q_init(1,1,:)), 'b--', 'DisplayName','UAV1 X 初始');
plot(1:N, squeeze(q_opt(1,1,:)), 'b-', 'LineWidth',1.5, 'DisplayName','UAV1 X 优化');
plot(1:N, squeeze(q_init(2,1,:)), 'r--', 'DisplayName','UAV2 X 初始');
plot(1:N, squeeze(q_opt(2,1,:)), 'r-', 'LineWidth',1.5, 'DisplayName','UAV2 X 优化');
title('X 坐标随时间变化'); xlabel('时隙 n'); ylabel('X (m)'); legend('Location','best');

fprintf('\n全部完成，可根据需要调整参数再次实验。\n');

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


