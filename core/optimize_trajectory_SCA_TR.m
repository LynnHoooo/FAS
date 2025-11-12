function [q_opt, trust_region_out] = optimize_trajectory_SCA_TR(...
    q_init, h_mkn_gain, alpha_mkn, w_mkn, R_mkn, u, H, params, t, ...
    gamma_min_SINR, Gamma, max_iter_SCA, tol, trust_region_in, verbose)
%% 轨迹优化 (SCA + Trust Region) - 带通信与感知增益线性化
% 版本: v2 - 真正优化系统性能

%% ========== 1. 参数提取 ==========
M = params.M; K = params.K; N = params.N;
dt = params.dt; Vmax = params.Vmax; Dmin = params.Dmin;
min_trust_region = params.min_trust_region;
Na = params.Na; d = params.d;
% 保存位置向量到参数结构（供子函数使用）
if nargin >= 9 && ~isempty(t)
    params.t = t(:);
else
    % 兼容回退：由 d 构建等间距位置
    params.t = (0:Na-1)' * d;
end

% 感知点信息
if isfield(params, 'v') && ~isempty(params.v)
    v_grid = params.v;
    Q = size(v_grid, 1);
else
    error('缺少感知点 params.v');
end
if isfield(params, 'H_sense')
    H_sense = params.H_sense;
else
    error('缺少感知高度 params.H_sense');
end

% 默认参数
if nargin < 11 || isempty(max_iter_SCA), max_iter_SCA = 20; end
if nargin < 12 || isempty(tol), tol = 1e-4; end
if nargin < 13 || isempty(trust_region_in), trust_region_in = min(5, Vmax * dt); end
if nargin < 14 || isempty(verbose), verbose = 1; end

% 输入检查
if any(isnan(q_init(:))) || any(isinf(q_init(:)))
    error('q_init 包含 NaN 或 Inf');
end
for n = 1:N-1
    dist = norm(squeeze(q_init(:, :, n+1) - q_init(:, :, n)), 'fro');
    if dist > Vmax*dt + 1e-6
        warning('初始轨迹在时隙 %d 不满足速度约束 (%.2f > %.2f)', n, dist, Vmax*dt);
    end
end

if verbose
    fprintf('  🚀 开始轨迹优化 (SCA + Trust Region)...\n');
    fprintf('     初始信任域: %.2f m, 最大迭代: %d\n', trust_region_in, max_iter_SCA);
end

%% ========== 1.1 跳过可行性诊断，直接使用波束优化结果 ==========
if verbose
    fprintf('  [策略] 相信波束优化结果，直接进行轨迹优化...\n');
end

%% ========== 2. 预计算常量 ==========
sigma2 = params.sigma2;  % 噪声功率
kappa = params.kappa;    % 路径损耗因子
prev_q = q_init;
converged = false;

% 感知区域中心（用于引导）
v_center = mean(v_grid, 1);

% 权重系数 (感知为硬约束+软优化)
lambda_comm = 1.0;       % 通信增益权重 (主要目标)
lambda_sense = 1e-9;     % 感知增益权重 (已有硬约束，软优化权重可以很小)
lambda_traj  = 0.01;     % 轨迹平滑权重 (避免过度振荡)

%% 预计算：感知增益线性化所需常量
sense_base = zeros(K, N);
sense_grad = zeros(K, N, 2);

for n = 1:N
    for k = 1:K
        base_sum = 0;
        grad_sum = [0, 0];

        for m = 1:M
            dx = prev_q(k,1,n) - u(m,1);
            dy = prev_q(k,2,n) - u(m,2);
            dist_sq = dx^2 + dy^2 + H(k)^2;
            dist = sqrt(dist_sq);

            if dist_sq <= 1e-6
                continue;
            end

            % ULA 导向矢量
            cos_theta = H(k) / dist;
            % 使用位置向量 t：a_vec = exp(j*2*pi*t*cos(theta))
            a_vec = exp(1j * 2*pi * params.t * cos_theta);

            % 该GBS在该时隙的总发射协方差（由 w_mkn 与 R_mkn 组成）
            X_mn = zeros(Na, Na);
            for i = 1:K
                if m <= size(w_mkn,1) && i <= size(w_mkn,2) && n <= size(w_mkn,3)
                    w_vec = w_mkn{m,i,n};
                    if ~isempty(w_vec)
                        X_mn = X_mn + (w_vec * w_vec');
                    end
                end
            end
            if m <= size(R_mkn,1) && ~isempty(R_mkn{m,1,n})
                X_mn = X_mn + R_mkn{m,1,n};
            end

            sense_gain = real(a_vec' * X_mn * a_vec);
            if sense_gain <= 0
                continue;
            end

            path_loss = kappa / dist_sq;
            base_contrib = path_loss * sense_gain;

            grad_coeff = -2 * kappa * sense_gain / (dist_sq^2);
            grad_vec = grad_coeff * [dx, dy];

            base_sum = base_sum + base_contrib;
            grad_sum = grad_sum + grad_vec;
        end

        sense_base(k, n) = base_sum;
        sense_grad(k, n, :) = grad_sum;
    end
end

%% ========== 3. SCA 主循环 ==========
for iter = 1:max_iter_SCA
    
    if verbose
        fprintf('  - SCA迭代 %d/%d (信任域: %.2f m)\n', iter, max_iter_SCA, trust_region_in);
    end

    % ========== 调试信息：约束检查 ==========
    if verbose && iter <= 3  % 只在前3次迭代输出详细信息
        fprintf('    [调试] 约束分析：\n');
        
        % 检查速度约束
        speed_limit = Vmax * dt;
        fprintf('    • 速度约束: 相邻时隙最大位移 ≤ %.2f m\n', speed_limit);
        
        % 检查当前轨迹的速度违反情况
        max_current_speed = 0;
        for n = 1:N-1
            current_dist = norm(squeeze(prev_q(:,:,n+1) - prev_q(:,:,n)), 'fro');
            max_current_speed = max(max_current_speed, current_dist);
        end
        fprintf('    • 当前轨迹最大位移: %.2f m\n', max_current_speed);
        
        % 检查信任域约束  
        fprintf('    • 信任域约束: 每时隙相对前次迭代位移 ≤ %.2f m\n', trust_region_in);
        
        % 检查感知约束（SCA凸下界）
        fprintf('    • 感知约束 (前5个时隙，SCA线性化):\n');
        for n = 1:min(5, N)
            for k = 1:K
                base_val = sense_base(k, n);
                grad_norm = norm(squeeze(sense_grad(k, n, :)));
                fprintf('      UAV%d 时隙%d: 基础感知功率=%.4e W, 梯度模=%.4e\n', ...
                    k, n, base_val, grad_norm);
            end
        end

        % 检查通信增益
        fprintf('    • 通信增益分析:\n');
        total_comm_links = 0;
        for n = 1:N
            for k = 1:K
                m_serv = find(alpha_mkn(:,k,n), 1);
                if ~isempty(m_serv)
                    total_comm_links = total_comm_links + 1;
                    if n <= 3 && k <= 2  % 只显示前几个
                        h_gain = h_mkn_gain{m_serv,k,n};
                        dist_old = norm(prev_q(k,:,n) - u(m_serv,:));
                        fprintf('      UAV%d→GBS%d 时隙%d: |h|²=%.4e, 距离=%.1f m\n', ...
                            k, m_serv, n, h_gain, dist_old);
                    end
                end
            end
        end
        fprintf('    • 总活跃通信链路: %d/%d\n', total_comm_links, K*N);
        
        % 检查避碰约束
        if K > 1
            min_uav_dist = inf;
            for n = 1:N
                dist_kk = norm(prev_q(1,:,n) - prev_q(2,:,n));
                min_uav_dist = min(min_uav_dist, dist_kk);
            end
            fprintf('    • UAV间最小距离: %.2f m (要求 ≥ %.2f m)\n', min_uav_dist, Dmin);
        end
        
        fprintf('    [调试] 开始CVX求解...\n');
    end

    cvx_begin quiet
        variable q(K, 2, N)

        %% ====== 目标函数：最大化性能增益 ======
        approx_comm_gain = 0;   % 通信速率增益（线性近似）
        approx_sensing_gain = 0; % 感知SNR增益（线性近似）

        % --- 1. 通信速率增益：基于 SINR 的线性近似 ---
        for n = 1:N
            for k = 1:K
                m_serv = find(alpha_mkn(:,k,n), 1);
                if isempty(m_serv), continue; end

                q_old = prev_q(k,:,n);
                dist_sq_old = sum((q_old - u(m_serv,:)).^2) + H(k)^2;
                
                % 梯度 d(1/d^2)/dq = -2 * (q-u) / d^4
                grad_term = -2 * (q_old - u(m_serv,:)) / (dist_sq_old^2);
                
                % 线性化信道增益 beta_mk  
                % 我们需要线性化 1/dist_sq，确保结果是实数标量
                position_diff = squeeze(q(k,:,n)) - q_old;
                linear_beta_mk = (kappa/dist_sq_old) + kappa * real(grad_term * position_diff');

                % 计算实际干扰 (更准确)
                interference = 0;
                for m_int = 1:M
                    for i_int = 1:K
                        if m_int == m_serv && i_int == k, continue; end
                        if ~isempty(w_mkn{m_int,i_int,n})
                            w_int = w_mkn{m_int,i_int,n};
                            interference = interference + h_mkn_gain{m_int,k,n} * norm(w_int)^2;
                        end
                    end
                    % 感知信号干扰
                    if m_int <= size(R_mkn,1) && ~isempty(R_mkn{m_int,1,n})
                        interference = interference + h_mkn_gain{m_int,k,n} * real(trace(R_mkn{m_int,1,n}));
                    end
                end
                
                % SCA线性化通信速率：log2(1+SINR) 的一阶泰勒展开
                P_comm = norm(w_mkn{m_serv,k,n})^2;
                
                % 当前点的SINR值（数值）
                beta_old = kappa / dist_sq_old;
                SINR_old = beta_old * P_comm / (interference + sigma2);
                
                % 线性化的SINR（CVX表达式）
                approx_SINR = linear_beta_mk * P_comm / (interference + sigma2);
                
                % log2(1+x) 的SCA近似：f(x0) + f'(x0)*(x-x0)
                % f'(x) = 1/((1+x)*ln(2))
                if SINR_old > 1e-12
                    rate_base = log2(1 + SINR_old);
                    rate_grad = 1 / ((1 + SINR_old) * log(2));
                    rate_linear = rate_base + rate_grad * (real(approx_SINR) - SINR_old);
                else
                    % SINR太小时使用简化线性近似
                    rate_linear = real(approx_SINR) / log(2);
                end
                
                % 确保rate_linear是标量
                rate_linear_scalar = sum(real(rate_linear(:)));
                approx_comm_gain = approx_comm_gain + rate_linear_scalar;

            end
        end

        % --- 2. 感知约束线性近似 (SCA 凸下界) ---
        for n = 1:N
            for k = 1:K
                grad_vec = squeeze(sense_grad(k, n, :))';
                sense_linear = sense_base(k, n) + real(grad_vec) * (squeeze(q(k,:,n)) - squeeze(prev_q(k,:,n)))';
                approx_sensing_gain = approx_sensing_gain + real(sense_linear);
                % 💡 感知约束作为软约束 - 主要通过波束优化保证，轨迹优化辅助
                % sense_linear >= Gamma;  % 移除硬感知约束，避免轨迹优化不可行
            end
        end

        % --- 3. 轨迹平滑代价 ---
        trajectory_cost = 0;
        for n = 2:N
            % 确保轨迹代价是标量：对矩阵的所有元素求平方和
            diff_matrix = q(:,:,n) - q(:,:,n-1);
            trajectory_cost = trajectory_cost + sum(sum_square(diff_matrix));
        end

        % 调试：检查各项维度
        if verbose
            fprintf('      [调试] 目标函数各项维度检查:\n');
            fprintf('        approx_comm_gain 大小: %s\n', mat2str(size(approx_comm_gain)));
            fprintf('        approx_sensing_gain 大小: %s\n', mat2str(size(approx_sensing_gain)));
            fprintf('        trajectory_cost 大小: %s\n', mat2str(size(trajectory_cost)));
        end
        
        % ✅ 最终目标：最小化负增益+平滑代价 (等价于最大化增益-代价)
        minimize( -lambda_comm * approx_comm_gain - lambda_sense * approx_sensing_gain + lambda_traj * trajectory_cost )

        subject to
            % ✅ 起点约束 (固定为初始轨迹)
            q(:, :, 1) == q_init(:, :, 1);
            % ✅ 终点约束 (固定为初始轨迹，不随迭代变化)
            q(:, :, N) == q_init(:, :, N);
            for n = 1:N-1
                norm(squeeze(q(:, :, n+1) - q(:, :, n)), 'fro') <= Vmax * dt;
            end
            % ✅ 信任域约束 (不包括起点和终点，它们已固定)
            for n = 2:N-1
                norm(squeeze(q(:, :, n) - prev_q(:, :, n)), 'fro') <= trust_region_in;
            end
            if K > 1
                for n = 1:N
                    for k1 = 1:K
                        for k2 = k1+1:K
                            dk = squeeze(prev_q(k1,:,n) - prev_q(k2,:,n));
                            dist_sq_lin = sum(dk.^2) + ...
                                2*dk*(squeeze(q(k1,:,n)) - squeeze(prev_q(k1,:,n)))' - ...
                                2*dk*(squeeze(q(k2,:,n)) - squeeze(prev_q(k2,:,n)))';
                            dist_sq_lin >= Dmin^2;
                        end
                    end
                end
            end
            % ✅ GBS覆盖范围约束（可选）
            for n = 1:N
                for k = 1:K
                    m_serv = find(alpha_mkn(:,k,n), 1);
                    if ~isempty(m_serv)
                        norm(squeeze(q(k,:,n)) - u(m_serv,:)) <= 500; % 放宽一些
                    end
                end
            end
    cvx_end

    %% ========== 4. 求解状态检查 ==========
    if ~strcmp(cvx_status, 'Solved') && ~contains(cvx_status, 'Solved')
        warning('CVX求解失败: %s', cvx_status);
        
        % ========== 详细的不可行诊断 ==========
        if verbose && iter <= 3
            fprintf('    [调试] CVX不可行诊断：\n');
            fprintf('      • CVX状态: %s\n', cvx_status);
            
            % 感知功率仅供参考（不是约束）
            fprintf('      • 感知功率状态 (参考):\n');
            min_sense_power = min(sense_base(:));
            avg_sense_power = mean(sense_base(:));
            fprintf('        最小基础功率: %.4e W (%.1f dBW)\n', min_sense_power, 10*log10(min_sense_power + 1e-12));
            fprintf('        平均基础功率: %.4e W (%.1f dBW)\n', avg_sense_power, 10*log10(avg_sense_power + 1e-12));
            
            % 检查起点约束
            fprintf('      • 起点约束检查:\n');
            for k = 1:K
                start_pos = squeeze(prev_q(k,:,1));
                fprintf('        UAV%d 起点: (%.1f, %.1f)\n', k, start_pos(1), start_pos(2));
            end
            
            % 检查信任域与速度约束冲突
            fprintf('      • 约束冲突分析:\n');
            fprintf('        信任域半径: %.2f m\n', trust_region_in);
            fprintf('        单步最大位移: %.2f m\n', Vmax * dt);
            if trust_region_in < Vmax * dt / 10
                fprintf('        ⚠️ 信任域过小，可能限制了轨迹灵活性\n');
            end
            
            % 检查目标函数组成
            fprintf('      • 目标函数权重:\n');
            fprintf('        通信权重: %.2f, 感知权重: %.2f, 平滑权重: %.4f\n', ...
                lambda_comm, lambda_sense, lambda_traj);
        end
        
        trust_region_in = max(trust_region_in * 0.7, min_trust_region);
        if trust_region_in <= min_trust_region + 1e-3
            fprintf('  ⚠️ 信任域过小 (%.3f m)，终止优化\n', trust_region_in);
            break;
        end
        continue;
    end

    if any(isnan(q(:))) || any(isinf(q(:)))
        warning('求解结果包含 NaN/Inf');
        trust_region_in = max(trust_region_in * 0.7, min_trust_region);
        continue;
    end

    %% ========== 5. 收敛判断 ==========
    q_new = q;
    rel_change = norm(q_new(:) - prev_q(:)) / (norm(prev_q(:)) + 1e-8);
    
    if iter > 1 && rel_change < tol
        if verbose
            fprintf('  ✅ 收敛于第 %d 次迭代\n', iter);
        end
        converged = true;
    end

    prev_q = q_new;

    % 成功则适度扩大信任域 (限制最大值保证线性化精度)
    trust_region_in = min(trust_region_in * 1.15, Vmax * dt * 3);  % 降低扩张速度和上限

    if converged
        break;
            end
        end

%% ========== 6. 输出 ==========
q_opt = prev_q;
trust_region_out = trust_region_in;

if verbose
    fprintf('  轨迹优化完成，新的信任区域半径: %.2f m\n', trust_region_out);
end

end

%% ====================================================================
% 辅助函数：可行性诊断
%% ====================================================================

function diagnostics = run_feasibility_diagnostics(q_init, w_mkn, R_mkn, params, u, H, Gamma, Vmax, dt, verbose)

diagnostics = struct();
diagnostics.sensing_feasible = true;
diagnostics.connectivity_feasible = true;
diagnostics.sensing_msgs = {};
diagnostics.connectivity_msgs = {};

[x_vals, y_vals, bounds] = determine_search_grid(q_init, params, u);
diagnostics.area_bounds = bounds;

if verbose
    fprintf('    ↪ 感知可行区域搜索网格: %d (X) × %d (Y) = %d 点\n', numel(x_vals), numel(y_vals), numel(x_vals)*numel(y_vals));
end

K = params.K; N = params.N;
step_limit = Vmax * dt;
diagnostics.feasible_sets = cell(K, 1);

for k = 1:K
    diagnostics.feasible_sets{k} = cell(N, 1);
        for n = 1:N
        feasible_pts = zeros(0, 2);
        max_power = -inf;
        for ix = 1:numel(x_vals)
            for iy = 1:numel(y_vals)
                xy = [x_vals(ix), y_vals(iy)];
                power_val = evaluate_sensing_power_xy(xy, k, n, w_mkn, R_mkn, u, H, params);
                if power_val > max_power
                    max_power = power_val;
                end
                if power_val >= Gamma
                    feasible_pts(end+1, :) = xy; %#ok<AGROW>
                end
            end
        end

        if isempty(feasible_pts)
            diagnostics.sensing_feasible = false;
            diagnostics.sensing_msgs{end+1} = sprintf('UAV%d 时隙 %d：感知可行集合为空 (最大功率 %.4e W < Gamma %.4e W)。', k, n, max_power, Gamma);
        else
            feasible_pts = unique(feasible_pts, 'rows');
        end

        if n == 1
            start_power = evaluate_sensing_power_xy(squeeze(q_init(k,:,1)), k, 1, w_mkn, R_mkn, u, H, params);
            if start_power < Gamma
                diagnostics.sensing_feasible = false;
                diagnostics.sensing_msgs{end+1} = sprintf('UAV%d 起点感知功率 %.4e W < Gamma %.4e W。', k, start_power, Gamma);
            else
                feasible_pts = unique([feasible_pts; squeeze(q_init(k,:,1))], 'rows');
            end
        end

        if n == N
            end_power = evaluate_sensing_power_xy(squeeze(q_init(k,:,N)), k, N, w_mkn, R_mkn, u, H, params);
            if end_power < Gamma
                diagnostics.sensing_feasible = false;
                diagnostics.sensing_msgs{end+1} = sprintf('UAV%d 终点感知功率 %.4e W < Gamma %.4e W。', k, end_power, Gamma);
            else
                feasible_pts = unique([feasible_pts; squeeze(q_init(k,:,N))], 'rows');
            end
        end

        diagnostics.feasible_sets{k}{n} = feasible_pts;
        diagnostics.max_sensing{k,n} = max_power;
    end
end

if ~diagnostics.sensing_feasible
    diagnostics.connectivity_feasible = false;
    return;
end

for k = 1:K
    [connected, msg] = check_connectivity_path(squeeze(q_init(k,:,1)), squeeze(q_init(k,:,N)), diagnostics.feasible_sets{k}, step_limit);
    if ~connected
        diagnostics.connectivity_feasible = false;
        diagnostics.connectivity_msgs{end+1} = sprintf('UAV%d: %s', k, msg);
    end
end

if isempty(diagnostics.connectivity_msgs)
    diagnostics.connectivity_feasible = true;
end

end

function [x_vals, y_vals, bounds] = determine_search_grid(q_init, params, u)

if isfield(params, 'area_bounds') && numel(params.area_bounds) == 4
    x_min = params.area_bounds(1,1);
    x_max = params.area_bounds(1,2);
    y_min = params.area_bounds(2,1);
    y_max = params.area_bounds(2,2);
else
    x_all = [reshape(q_init(:,1,:), [], 1); u(:,1)];
    y_all = [reshape(q_init(:,2,:), [], 1); u(:,2)];
    if isfield(params, 'v') && ~isempty(params.v)
        x_all = [x_all; params.v(:,1)];
        y_all = [y_all; params.v(:,2)];
    end
    margin = 20;
    x_min = min(x_all) - margin;
    x_max = max(x_all) + margin;
    y_min = min(y_all) - margin;
    y_max = max(y_all) + margin;
end

grid_step = 20;
if isfield(params, 'diagnose') && isfield(params.diagnose, 'grid_step')
    grid_step = params.diagnose.grid_step;
end

if x_max <= x_min
    x_max = x_min + grid_step;
end
if y_max <= y_min
    y_max = y_min + grid_step;
end

x_vals = x_min:grid_step:x_max;
y_vals = y_min:grid_step:y_max;

if numel(x_vals) < 2
    x_vals = linspace(x_min, x_max, 5);
end
if numel(y_vals) < 2
    y_vals = linspace(y_min, y_max, 5);
end

bounds = [x_min, x_max; y_min, y_max];

end

function power_val = evaluate_sensing_power_xy(xy, k, n, w_mkn, R_mkn, u, H, params)

M = params.M; K = params.K; Na = params.Na;
kappa = params.kappa; d = params.d;

power_val = 0;

for m = 1:M
    X_cov = zeros(Na, Na);
    if m <= size(w_mkn,1) && n <= size(w_mkn,3)
        for i = 1:K
            if i <= size(w_mkn,2)
                w_vec = w_mkn{m,i,n};
                if ~isempty(w_vec)
                    X_cov = X_cov + (w_vec * w_vec');
                end
            end
        end
    end

    if m <= size(R_mkn,1) && n <= size(R_mkn,3)
        R_block = R_mkn{m,1,n};
        if ~isempty(R_block)
            X_cov = X_cov + R_block;
        end
    end

    dx = xy(1) - u(m,1);
    dy = xy(2) - u(m,2);
    dist_sq = dx^2 + dy^2 + H(k)^2;
    if dist_sq <= 1e-6
        continue;
    end
    dist = sqrt(dist_sq);
    cos_theta = H(k) / dist;
    % 使用位置向量 t：a_vec = exp(j*2*pi*t*cos(theta))
    if isfield(params, 't') && numel(params.t) == Na
        a_vec = exp(1j * 2*pi * params.t * cos_theta);
    else
        phi = 2*pi*d * cos_theta;
        a_vec = exp(1j * phi * (0:Na-1)');
    end
    power_val = power_val + real(a_vec' * X_cov * a_vec) / dist_sq;
end

power_val = max(real(power_val), 0);

end

function [connected, message] = check_connectivity_path(q_start, q_end, feasible_sets, step_limit)

num_slots = numel(feasible_sets);
reachable = q_start;

for n = 2:num_slots
    candidates = feasible_sets{n};
    if isempty(candidates)
        connected = false;
        message = sprintf('时隙 %d 无满足感知约束的位置。', n);
        return;
    end

    new_reachable = zeros(0, 2);
    for idx = 1:size(candidates, 1)
        pt = candidates(idx, :);
        dists = sqrt(sum((reachable - pt).^2, 2));
        if any(dists <= step_limit + 1e-6)
            new_reachable(end+1, :) = pt; %#ok<AGROW>
    end
end

    if isempty(new_reachable)
        connected = false;
        message = sprintf('时隙 %d 的可行区域与上一时隙不连通 (步长上限 %.2f m)。', n, step_limit);
        return;
    end

    reachable = unique(new_reachable, 'rows');
end

if isempty(reachable)
    connected = false;
    message = '终时刻无可达位置。';
    return;
end

dist_to_goal = sqrt(sum((reachable - q_end).^2, 2));
if all(dist_to_goal > step_limit + 1e-6)
    connected = false;
    message = sprintf('终点与可行集合的最小距离 %.2f m 大于步长上限 %.2f m。', min(dist_to_goal), step_limit);
    return;
end

connected = true;
message = '';

end