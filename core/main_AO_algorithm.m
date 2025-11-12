function [final_sum_rate, final_min_sensing, ao_history] = main_AO_algorithm(p)
%% 网络化ISAC系统交替优化（AO）算法主函数 (函数版)
% 版本: 2.1 - 增加ao_history输出用于收敛分析
% 输入: p - 包含所有初始化参数的结构体
% 输出: final_sum_rate - 最终和速率
%       final_min_sensing - 最终最小感知功率
%       ao_history - AO迭代历史数据

%% 步骤1: 参数解包
% 从 p 结构体中解包所有必要的变量
M = p.M; K = p.K; N = p.N; Q = p.Q; Na = p.Na; B = p.B;
Pmax = p.Pmax; sigma2 = p.sigma2; Gamma = p.Gamma;
kappa = p.kappa; d = p.d; H_sense = p.H_sense;
q_traj = p.q_traj; alpha_init = p.alpha_init; 
W_init = p.W_init; R_init = p.R_init; h_mkn = p.h_mkn;
u = p.u; v = p.v; H = p.H;
dt = p.dt; Vmax = p.Vmax; Dmin = p.Dmin;

% 位置向量初始化（多GBS设计，每个GBS独立的位置向量）
t_current = cell(M, 1);  % 每个GBS一个位置向量
if isfield(p, 't_init') && ~isempty(p.t_init)
    % 如果提供了t_init，检查是cell还是向量
    if iscell(p.t_init)
        % 已经是cell格式，直接使用
        for m = 1:M
            t_current{m} = p.t_init{m}(:);
        end
    else
        % 是单个向量，所有GBS共享相同初始位置
        for m = 1:M
            t_current{m} = p.t_init(:);
        end
    end
else
    % 默认：所有GBS使用等间距初始化
    d_lambda = d;
    t_default = (0:Na-1)' * d_lambda;
    for m = 1:M
        t_current{m} = t_default;
    end
end
fprintf('  [AO] Antennas: Na=%d, M=%d GBSs with independent antenna positions\n', Na, M);
for m = 1:M
    fprintf('    GBS %d: t range=[%.2f, %.2f]\n', m, t_current{m}(1), t_current{m}(end));
end

% AO算法参数设置（从参数结构体读取，如果没有则使用默认值）
if isfield(p, 'max_iterations')
    max_iterations = p.max_iterations;
else
    max_iterations = 10;
end
if isfield(p, 'tolerance')
    tolerance = p.tolerance;
else
    tolerance = 1e-4;
end
if isfield(p, 'trust_region')
    trust_region = p.trust_region;
else
    trust_region = 10;
end
if isfield(p, 'min_trust_region')
    min_trust_region = p.min_trust_region;
else
    min_trust_region = 1;
end
% 是否每次迭代后保存数据
if isfield(p, 'save_each_iteration')
    save_each_iteration = p.save_each_iteration;
else
    save_each_iteration = false;
end
% 保存文件的路径（如果save_each_iteration为true）
if isfield(p, 'save_file_path')
    save_file_path = p.save_file_path;
else
    save_file_path = 'data/ao_convergence_results.mat';
end

%% 交替优化算法主循环
% 初始化当前解
q_current = q_traj;
alpha_current = alpha_init;
W_current = W_init;
R_current = R_init;  % 添加R_current跟踪当前感知波束

% 计算初始性能
    [initial_sum_rate, initial_min_sensing] = evaluate_performance(h_mkn, alpha_current, W_current, R_init, ...
    u, v, M, K, N, Q, Na, Pmax, sigma2, Gamma, kappa, d, H_sense);

fprintf('🚀 开始AO算法主循环...\n');
fprintf('初始和速率: %.4f bps/Hz\n', initial_sum_rate);

% 主优化循环 - 增强数据保存
final_iter = max_iterations;
% 增加1个位置用于存储"第0次迭代"（初始状态）
sum_rate_history = zeros(max_iterations + 1, 1);
min_sensing_power_history = zeros(max_iterations + 1, 1);

% 详细数据保存结构
ao_history = struct();
ao_history.trajectories = cell(max_iterations + 1, 1);      % 每次迭代的轨迹
ao_history.associations = cell(max_iterations + 1, 1);      % 每次迭代的关联
ao_history.beamforming_W = cell(max_iterations + 1, 1);     % 每次迭代的通信波束
ao_history.beamforming_R = cell(max_iterations + 1, 1);     % 每次迭代的感知波束
ao_history.antenna_positions = cell(max_iterations + 1, M); % 每次迭代，每个GBS的位置
ao_history.performance = struct();                       % 性能指标
ao_history.trust_regions = zeros(max_iterations + 1, 1);    % 信任域历史

% 记录第0次迭代（初始状态）- 新增
sum_rate_history(1) = initial_sum_rate;
min_sensing_power_history(1) = initial_min_sensing;
ao_history.trajectories{1} = q_current;
ao_history.associations{1} = alpha_current;
ao_history.beamforming_W{1} = W_current;
ao_history.beamforming_R{1} = R_init;
for m = 1:M
    ao_history.antenna_positions{1, m} = t_current{m};
end
ao_history.trust_regions(1) = trust_region;

for iter = 1:max_iterations
    fprintf('\n=== AO迭代 %d/%d ===\n', iter, max_iterations);
    
    % 子问题1: 优化关联（使用当前的R_current而不是R_init）
    fprintf('  步骤1: 关联优化...\n');
    alpha_new = optimize_association(h_mkn, W_current, R_current, Pmax, sigma2, M, K, N, Na);
    % 注意：不重新构建W_new，直接使用W_current作为波束优化的初始点
    % 这样可以保持迭代的连续性，避免第一次SCA虚高的问题
    
    % 子问题2: 优化波束成形（使用当前解作为初始点）
    fprintf('  步骤2: 波束优化...\n');
    try
        % 注意：optimize_beamforming接受单个位置向量，使用GBS1的位置作为代表
        % 这是为了与原始系统保持兼容性的临时方案
        [W_new, R_new, ~] = optimize_beamforming(...
            h_mkn, alpha_new, R_current, W_current, ...
            Pmax, Gamma, sigma2, M, K, N, Na, Q, v, u, H_sense, kappa, t_current{1});
        fprintf('    ✅ 波束优化成功\n');
    catch ME
        fprintf('    ⚠️ 波束优化失败: %s\n', ME.message);
        W_new = W_current; % 保持当前波束矩阵
        R_new = R_current;
    end
    
    % 子问题2.5: 天线位置优化（真正的SCA优化，每个GBS独立优化）
    fprintf('  步骤2.5: 天线位置优化（每个GBS独立）...\n');
    if isfield(p, 't_start'), t_start = p.t_start; else, t_start = 0; end
    if isfield(p, 't_end'), t_end = p.t_end; else, t_end = (Na-1) * d; end
    if isfield(p, 'd_min'), d_min = p.d_min; else, d_min = 0.1; end
    
    % 为每个GBS单独优化天线位置
    t_new = cell(M, 1);
    position_optimized = false;
    for m = 1:M
        fprintf('    优化GBS %d的天线位置...\n', m);
        try
            % 调用位置优化函数（固定波束W和R）
            [t_new{m}, ~] = optimize_antenna_position(...
                q_current, alpha_new, W_new, R_new, ...
                u, v, H, H_sense, M, K, N, Na, Q, t_current{m}, t_start, t_end, d_min, ...
                kappa, Pmax, Gamma, sigma2);
            position_optimized = true;
            fprintf('      GBS %d: 位置优化完成\n', m);
        catch ME
            fprintf('      ⚠️ GBS %d 位置优化失败: %s（保持原位置）\n', m, ME.message);
            t_new{m} = t_current{m};
        end
    end
    
    % 更新位置
    t_current = t_new;
    
    % 位置变化后更新信道（使用每个GBS各自的位置）
    if position_optimized
        fprintf('    更新所有信道（基于新的天线位置）...\n');
        for m=1:M
            for k=1:K
                for n=1:N
                    h_mkn{m,k,n} = get_channel(m,k,n,u,q_current,H,kappa,t_current{m},Na);
                end
            end
        end
        fprintf('    ✅ 天线位置优化和信道更新完成\n');
    end
    
    % 子问题3: 优化轨迹
    fprintf('  步骤3: 轨迹优化...\n');
    params.M = M; params.K = K; params.N = N; params.Na = Na;
    params.d = d; params.kappa = kappa; params.sigma2 = sigma2;
    params.dt = dt; params.Vmax = Vmax; params.v = v;
    params.H_sense = H_sense; params.min_trust_region = min_trust_region;
    params.Dmin = Dmin;

    h_mkn_gain = cell(M, K, N);
    for m=1:M, for k=1:K, for n=1:N, h_mkn_gain{m,k,n} = norm(h_mkn{m,k,n})^2; end, end, end
    w_mkn_vectors = cell(M, K, N);
    for m=1:M, for k=1:K, for n=1:N
        if alpha_new(m,k,n) == 1 && ~isempty(W_new{m,k,n})
            [V, D] = eig(W_new{m,k,n}); [~, idx] = max(diag(D));
            w_mkn_vectors{m,k,n} = V(:,idx) * sqrt(D(idx,idx));
        else, w_mkn_vectors{m,k,n} = zeros(Na, 1); end
    end, end, end
    gamma_min_SINR = db2pow(5);
    
    [q_new, trust_region] = optimize_trajectory_SCA_TR(...
        q_current, h_mkn_gain, alpha_new, w_mkn_vectors, R_new, u, H, params, t_current{1}, ...
        gamma_min_SINR, Gamma, 15, 1e-3, trust_region, 0); % verbose=0 for AO
    % 注意：轨迹优化中使用GBS1的位置（简化处理）
    fprintf('    ✅ 轨迹优化完成，信任域: %.2f m\n', trust_region);
    
    % 更新信道（轨迹变化后，使用每个GBS各自的位置）
    for m=1:M
        for k=1:K
            for n=1:N
                h_mkn{m,k,n} = get_channel(m,k,n,u,q_new,H,kappa,t_current{m},Na);
            end
        end
    end
    
    % 计算性能并记录（注意：由于第0次迭代已占用索引1，实际AO迭代从索引2开始）
    fprintf('  步骤4: 性能评估...\n');
    [current_sum_rate, current_min_sensing] = evaluate_performance(...
        h_mkn, alpha_new, W_new, R_new, u, v, M, K, N, Q, Na, Pmax, sigma2, Gamma, kappa, d, H_sense);
    
    % 验证性能单调性（AO算法理论上应保证目标函数单调递增）
    if iter >= 1 && current_sum_rate < sum_rate_history(iter) - 1e-6
        fprintf('    ⚠️ 警告: 和速率下降 %.4f -> %.4f (下降%.2f%%)\n', ...
            sum_rate_history(iter), current_sum_rate, ...
            (sum_rate_history(iter) - current_sum_rate) / sum_rate_history(iter) * 100);
    end
    sum_rate_history(iter + 1) = current_sum_rate;  % iter=1时存到索引2
    min_sensing_power_history(iter + 1) = current_min_sensing;

    % 保存详细迭代数据
    ao_history.trajectories{iter + 1} = q_new;
    ao_history.associations{iter + 1} = alpha_new;
    ao_history.beamforming_W{iter + 1} = W_new;
    ao_history.beamforming_R{iter + 1} = R_new;
    for m = 1:M
        ao_history.antenna_positions{iter + 1, m} = t_current{m};
    end
    ao_history.trust_regions(iter + 1) = trust_region;

    % 打印迭代结果
    fprintf('  迭代 %d: %.4f bps/Hz\n', iter, current_sum_rate);

    % 收敛性检查（需要连续两次迭代的相对改善都小于tolerance才认为收敛）
    if iter >= 2
        rel_rate_improve_prev = abs(sum_rate_history(iter) - sum_rate_history(iter - 1)) / (abs(sum_rate_history(iter - 1)) + 1e-8);
        rel_rate_improve_curr = abs(sum_rate_history(iter + 1) - sum_rate_history(iter)) / (abs(sum_rate_history(iter)) + 1e-8);
        fprintf('  📈 性能改善: 上一次 %.6f%%, 本次 %.6f%% ', rel_rate_improve_prev*100, rel_rate_improve_curr*100);
        % 连续两次迭代的相对改善都小于tolerance才认为收敛
        if rel_rate_improve_prev < tolerance && rel_rate_improve_curr < tolerance
            fprintf('(已收敛)\n');
            final_iter = iter;
            break;
        else
            fprintf('(继续优化)\n');
        end
    elseif iter >= 1
        rel_rate_improve = abs(sum_rate_history(iter + 1) - sum_rate_history(iter)) / (abs(sum_rate_history(iter)) + 1e-8);
        fprintf('  📈 性能改善: %.6f%% (继续优化，需至少2次迭代判断收敛)\n', rel_rate_improve*100);
    end
    
    % 每次迭代后保存数据（如果启用）
    if save_each_iteration
        % 临时保存当前迭代的数据
        temp_ao_history = struct();
        temp_ao_history.trajectories = ao_history.trajectories(1:iter + 1);
        temp_ao_history.associations = ao_history.associations(1:iter + 1);
        temp_ao_history.beamforming_W = ao_history.beamforming_W(1:iter + 1);
        temp_ao_history.beamforming_R = ao_history.beamforming_R(1:iter + 1);
        temp_ao_history.antenna_positions = ao_history.antenna_positions(1:iter + 1, :);
        temp_ao_history.trust_regions = ao_history.trust_regions(1:iter + 1);
        temp_ao_history.performance = struct();
        temp_ao_history.performance.sum_rates = sum_rate_history(1:iter + 1);
        temp_ao_history.performance.min_sensing_powers = min_sensing_power_history(1:iter + 1);
        temp_ao_history.performance.iterations = iter + 1;
        temp_ao_history.performance.converged = false;  % 尚未收敛
        temp_ao_history.system_params = struct();
        temp_ao_history.system_params.M = M; temp_ao_history.system_params.K = K; 
        temp_ao_history.system_params.N = N; temp_ao_history.system_params.u = u;
        temp_ao_history.system_params.v = v; temp_ao_history.system_params.H = H;
        temp_ao_history.system_params.Gamma = Gamma; temp_ao_history.system_params.kappa = kappa;
        temp_ao_history.system_params.Na = Na; temp_ao_history.system_params.d = d;
        temp_ao_history.system_params.H_sense = H_sense;
        
        % 保存当前状态（用于断点续传）
        current_state = struct();
        current_state.q_current = q_new;
        current_state.alpha_current = alpha_new;
        current_state.W_current = W_new;
        current_state.R_current = R_new;
        current_state.t_current = t_current;  % 保存位置向量
        current_state.h_mkn = h_mkn;
        current_state.trust_region = trust_region;
        current_state.current_iter = iter;
        
        try
            save(save_file_path, 'temp_ao_history', 'current_state', '-v7.3');
            fprintf('  💾 迭代数据已保存到: %s\n', save_file_path);
        catch ME
            fprintf('  ⚠️ 保存数据失败: %s\n', ME.message);
        end
    end
    
    % 更新解（确保R_current也被更新）
    q_current = q_new; 
    alpha_current = alpha_new;
    W_current = W_new; 
    R_current = R_new;  % 关键：更新R_current以供下一次迭代使用
end

%% 步骤3: 结果输出和数据保存
% 注意：由于包含第0次迭代，final_iter需要+1
final_sum_rate = sum_rate_history(final_iter + 1);
final_min_sensing = min_sensing_power_history(final_iter + 1);

% 截取实际使用的历史数据（包含第0次迭代）
ao_history.trajectories = ao_history.trajectories(1:final_iter + 1);
ao_history.associations = ao_history.associations(1:final_iter + 1);
ao_history.beamforming_W = ao_history.beamforming_W(1:final_iter + 1);
ao_history.beamforming_R = ao_history.beamforming_R(1:final_iter + 1);
ao_history.trust_regions = ao_history.trust_regions(1:final_iter + 1);

% 保存性能历史（包含第0次迭代）
ao_history.performance.sum_rates = sum_rate_history(1:final_iter + 1);
ao_history.performance.min_sensing_powers = min_sensing_power_history(1:final_iter + 1);
ao_history.performance.initial_sum_rate = initial_sum_rate;
ao_history.performance.initial_min_sensing = initial_min_sensing;
ao_history.performance.final_sum_rate = final_sum_rate;
ao_history.performance.final_min_sensing = final_min_sensing;
ao_history.performance.iterations = final_iter + 1;  % 包含第0次迭代的总数
ao_history.performance.converged = (final_iter < max_iterations);

% 保存系统参数供后续画图使用
ao_history.system_params = struct();
ao_history.system_params.M = M; ao_history.system_params.K = K; ao_history.system_params.N = N;
ao_history.system_params.u = u; ao_history.system_params.v = v; ao_history.system_params.H = H;
ao_history.system_params.Gamma = Gamma; ao_history.system_params.kappa = kappa;
ao_history.system_params.Na = Na; ao_history.system_params.d = d; ao_history.system_params.H_sense = H_sense;

% 保存到文件（最终结果）
if save_each_iteration
    % 如果启用了每次迭代保存，最终结果也保存到同一个文件
    final_ao_history = ao_history;
    final_state = struct();
    final_state.q_current = q_current;
    final_state.alpha_current = alpha_current;
    final_state.W_current = W_current;
    final_state.R_current = R_current;
    final_state.h_mkn = h_mkn;
    final_state.trust_region = trust_region;
    final_state.current_iter = final_iter;
    save(save_file_path, 'final_ao_history', 'final_state', 'final_sum_rate', 'final_min_sensing', '-v7.3');
    fprintf('✅ 最终数据已保存到: %s\n', save_file_path);
else
    % 标准保存位置 - 确保目录存在
    if ~exist('data', 'dir')
        mkdir('data');
        fprintf('📁 创建data目录\n');
    end
    save('data/ao_convergence_results.mat', 'ao_history', 'final_sum_rate', 'final_min_sensing');
end

fprintf('\n🎯 AO算法完成！\n');
fprintf('最终和速率: %.4f bps/Hz\n', final_sum_rate);
fprintf('✅ 详细数据已保存到: data/ao_convergence_results.mat\n');

% 返回AO历史数据用于收敛分析
% ao_history变量已在函数开始时初始化并在每次迭代中更新

end


%% ========== 子函数定义 ==========

function [avg_sum_rate, min_sensing_power] = evaluate_performance( ...
    h_mkn_precomputed, alpha_mkn, W_mkn, R_mkn, ...
    u, v, M, K, N, Q, Na, Pmax, sigma2, Gamma, kappa, d, H_sense)

    % 计算平均和速率
    avg_sum_rate = compute_sum_rate(h_mkn_precomputed, W_mkn, R_mkn, alpha_mkn, sigma2, M, K, N);

    % 计算最小感知功率
    min_sensing_power = compute_sensing_power(alpha_mkn, W_mkn, R_mkn, u, v, M, Q, N, Na, kappa, d, H_sense);

end

function rate_per_uav = calculate_rate_per_uav(h_mkn_precomputed, alpha_mkn, W_mkn, R_mkn, M, K, N, Na, sigma2)
    rate_per_uav = zeros(K, 1);

    for k = 1:K
        rate_sum = 0;
        for n = 1:N
            m_serv = find(alpha_mkn(:, k, n) == 1, 1);
            if isempty(m_serv)
                continue;
            end

            h_mkn = h_mkn_precomputed{m_serv, k, n};
            signal_power = real(h_mkn' * W_mkn{m_serv, k, n} * h_mkn);

            interference = 0;
            for m = 1:M
                for i = 1:K
                    if m == m_serv && i == k
                        continue;
                    end
                    interference = interference + real(h_mkn' * W_mkn{m, i, n} * h_mkn);
                end
                interference = interference + real(h_mkn' * R_mkn{m, 1, n} * h_mkn);
            end

            SINR = signal_power / (interference + sigma2);
            rate_sum = rate_sum + log2(1 + max(SINR, 0));
        end

        rate_per_uav(k) = rate_sum / N;
    end
end

function alpha_bad = construct_bad_initial_association(M, K, N)
    alpha_bad = zeros(M, K, N);
    for n = 1:N
        m_idx = mod(n-1, M) + 1;
        for k = 1:K
            alpha_bad(m_idx, k, n) = 1;
        end
    end
end

function W_reconstructed = reconstruct_beamforming_for_association(alpha_mkn, h_mkn_precomputed, Pmax, M, K, N, Na)
    W_reconstructed = cell(M, K, N);

    for m = 1:M
        for n = 1:N
            served_users = find(alpha_mkn(m, :, n));
            num_users = numel(served_users);

            if num_users == 0
                for k = 1:K
                    W_reconstructed{m, k, n} = zeros(Na, Na);
                end
                continue;
            end

            power_per_user = Pmax / num_users;
            for k_idx = 1:num_users
                k = served_users(k_idx);
                h_mkn = h_mkn_precomputed{m, k, n};
                if norm(h_mkn) < 1e-9
                    W_reconstructed{m, k, n} = zeros(Na, Na);
                else
                    w = sqrt(power_per_user) * h_mkn / norm(h_mkn);
                    W_reconstructed{m, k, n} = w * w';
                end
            end

            unserved = setdiff(1:K, served_users);
            for k = unserved
                W_reconstructed{m, k, n} = zeros(Na, Na);
            end
        end
    end
end

function min_sensing_power = compute_sensing_power(alpha_mkn, W_mkn, R_mkn, u, v, M, Q, N, Na, kappa, d, H_sense)
    K = size(alpha_mkn, 2);
    zeta_qn = zeros(Q, N);

    for n = 1:N
        for q_idx = 1:Q
            total_power = 0;
            for m = 1:M
                composite = zeros(Na, Na);
                for k = 1:K
                    composite = composite + W_mkn{m, k, n};
                end
                composite = composite + R_mkn{m, 1, n};

                dx = v(q_idx, 1) - u(m, 1);
                dy = v(q_idx, 2) - u(m, 2);
                dist = sqrt(dx^2 + dy^2 + H_sense^2);

                % 修改：使用归一化功率（忽略kappa），与initial.m保持一致
                % path_loss = kappa / (dist^2);  % 旧版本
                path_loss = 1 / (dist^2);  % 新版本：归一化功率

                cos_theta = H_sense / dist;
                steering = exp(1j * (0:Na-1)' * 2 * pi * d * cos_theta);

                total_power = total_power + path_loss * real(steering' * composite * steering);
            end

            zeta_qn(q_idx, n) = total_power;
        end
    end

    min_sensing_power = min(zeta_qn(:));
end


function [q_updated, h_updated] = position_search_2d(q_current, h_current, W_mkn, R_mn, alpha_mkn, ...
    u, H, sigma2, kappa, d, Na, search_cfg, M, K, N)

    q_updated = q_current;
    h_updated = h_current;

    if ~isfield(search_cfg, 'x_range'), search_cfg.x_range = [0, 1000]; end
    if ~isfield(search_cfg, 'y_range'), search_cfg.y_range = [0, 1000]; end
    if ~isfield(search_cfg, 'grid_step') || search_cfg.grid_step <= 0
        search_cfg.grid_step = 100;
    end

    if ~isfield(search_cfg, 'weights') || numel(search_cfg.weights) ~= K
        weights = ones(K, 1);
    else
        weights = reshape(search_cfg.weights, [], 1);
    end

    x_vals = search_cfg.x_range(1):search_cfg.grid_step:search_cfg.x_range(2);
    if abs(x_vals(end) - search_cfg.x_range(2)) > 1e-6
        x_vals = [x_vals, search_cfg.x_range(2)]; %#ok<AGROW>
    end
    y_vals = search_cfg.y_range(1):search_cfg.grid_step:search_cfg.y_range(2);
    if abs(y_vals(end) - search_cfg.y_range(2)) > 1e-6
        y_vals = [y_vals, search_cfg.y_range(2)]; %#ok<AGROW>
    end

    fprintf('\n--- 二维位置搜索：在 %.0fm × %.0fm 区域内进行粗网格优化 ---\n', ...
        search_cfg.x_range(2) - search_cfg.x_range(1), search_cfg.y_range(2) - search_cfg.y_range(1));

    for k = 1:K
        current_pos = squeeze(q_updated(k, :, 1));
        best_score = -inf;
        best_pos = current_pos;
        best_h = h_updated;

        x_candidates = unique([x_vals, current_pos(1)]);
        y_candidates = unique([y_vals, current_pos(2)]);

        for xi = 1:numel(x_candidates)
            for yi = 1:numel(y_candidates)
                candidate = [x_candidates(xi), y_candidates(yi)];

                q_trial = q_updated;
                for n = 1:N
                    q_trial(k, :, n) = candidate;
                end

                h_trial = h_updated;
                for m = 1:M
                    for n = 1:N
                        h_trial{m, k, n} = get_channel(m, k, n, u, q_trial, H, kappa, d, Na);
                    end
                end

                rate_per_uav = calculate_rate_per_uav(h_trial, alpha_mkn, W_mkn, R_mn, M, K, N, Na, sigma2);
                weighted_score = sum(weights(:) .* rate_per_uav(:));

                if weighted_score > best_score + 1e-6
                    best_score = weighted_score;
                    best_pos = candidate;
                    best_h = h_trial;
                end
            end
        end

        if best_score > -inf
            for n = 1:N
                q_updated(k, :, n) = best_pos;
            end
            h_updated = best_h;
        end

        fprintf('  UAV%d: 最优位置 (%.1f m, %.1f m)，加权速率评分 %.4f\n', k, best_pos(1), best_pos(2), best_score);
    end
end

%% 缺失的辅助函数
function rate = compute_sum_rate(h_mkn_precomputed, W_mkn, R_mkn, alpha_mkn, sigma2, M, K, N)
% 计算系统总和速率
    rate = 0;
    for n = 1:N
        for k = 1:K
            m_serv = find(alpha_mkn(:,k,n) == 1, 1);
            if isempty(m_serv)
                continue;
            end
            
            h_mk = h_mkn_precomputed{m_serv,k,n};
            signal_power = real(h_mk' * W_mkn{m_serv,k,n} * h_mk);
            
            interference = 0;
            % 来自其他GBS和用户的干扰
            for l = 1:M
                for i = 1:K
                    if ~(l==m_serv && i==k) && alpha_mkn(l,i,n) == 1
                        h_lk = h_mkn_precomputed{l,k,n};
                        interference = interference + real(h_lk' * W_mkn{l,i,n} * h_lk);
                    end
                end
                % 感知信号干扰
                if ~isempty(R_mkn) && ~isempty(R_mkn{l,1,n})
                    h_lk = h_mkn_precomputed{l,k,n};
                    interference = interference + real(h_lk' * R_mkn{l,1,n} * h_lk);
                end
            end
            
            SINR = signal_power / (interference + sigma2);
            rate = rate + log2(1 + max(SINR, 1e-12));
        end
    end
end

