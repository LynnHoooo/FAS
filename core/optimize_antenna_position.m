function [t_opt, obj_history] = optimize_antenna_position(...
    q_traj, alpha_mkn, W_mkn, R_mkn, ...
    u, v, H, H_sense, M, K, N, Na, Q, t_init, t_start, t_end, d_min, ...
    kappa, Pmax, Gamma, sigma2)
% OPTIMIZE_ANTENNA_POSITION - 天线位置优化（两步SCA方法）
%
% 说明：
%   使用两步SCA方法优化天线位置，固定波束W、R和轨迹q
%   步骤1: 线性化功率项 g(t)
%   步骤2: 利用DC形式和log函数凹性构建凹优化问题
%
% 输入参数：
%   q_traj      : K×2×N, UAV轨迹
%   alpha_mkn   : M×K×N, 关联矩阵
%   W_mkn       : cell{M,K,N}, 通信波束矩阵
%   R_mkn       : cell{M,1,N}, 感知波束矩阵
%   u           : M×2, GBS位置
%   v           : Q×2, 感知点位置
%   H           : K×1, UAV高度
%   H_sense     : 标量, 感知高度
%   M,K,N,Na,Q  : 系统参数
%   t_init      : Na×1, 初始位置向量
%   t_start     : 标量, 起始坐标
%   t_end       : 标量, 结束坐标
%   d_min       : 标量, 最小间距约束
%   kappa       : 标量, 路径损耗因子
%   Pmax        : 标量, 最大功率
%   Gamma       : 标量, 感知阈值
%   sigma2      : 标量, 噪声功率
%
% 输出：
%   t_opt       : Na×1, 优化后的位置向量
%   obj_history : 目标函数历史值

    %% 参数检查和初始化
    if numel(t_init) ~= Na
        error('optimize_antenna_position:InvalidInit', 't_init 长度应等于 Na');
    end
    t_init = t_init(:);
    
    % SCA参数
    max_sca_iter = 15;
    tol = 1e-4;
    use_trust_region = true;
    delta_t_max = 0.5;  % 信任域半径（归一化单位）
    
    % 初始化
    t_old = ensure_feasible(t_init, t_start, t_end, d_min, Na);
    obj_history = [];
    
    fprintf('  [位置优化] 开始SCA迭代 (max_iter=%d)\n', max_sca_iter);
    
    %% SCA主循环
    for sca_iter = 1:max_sca_iter
        % 步骤1: 计算当前目标值
        R_old = compute_sum_rate_with_t(t_old, q_traj, alpha_mkn, W_mkn, R_mkn, ...
            u, H, M, K, N, Na, kappa, sigma2);
        obj_history(end+1) = R_old;
        
        if sca_iter == 1
            fprintf('    迭代 %d: R_sum = %.4f bps/Hz\n', sca_iter, R_old);
        end
        
        % 步骤2: 计算所有需要的梯度
        [g_all, grad_g_all] = compute_all_gradients(t_old, q_traj, alpha_mkn, W_mkn, ...
            u, H, M, K, N, Na, kappa);
        
        % 步骤3: 构建CVX优化问题（两步SCA）
        cvx_begin quiet
            cvx_precision low
            variable t_var(Na, 1)
            
            % 目标函数初始化
            obj_expr = 0;
            
            % 对每个有效链路构建线性化目标
            for n = 1:N
                for m = 1:M
                    for k = 1:K
                        if alpha_mkn(m, k, n) == 0
                            continue;
                        end
                        
                        % 提取当前(m,k,n)的g和梯度
                        g_mkn = g_all{m,k,n};  % K×1向量
                        grad_g_mkn = grad_g_all{m,k,n};  % Na×K矩阵
                        
                        % 构建 f̃₁(t) - f̂₂(t)
                        % f̃₁(t) = log₂(Σ_j g̃_mjk(t) + σ²)
                        sum_g_tilde = sum(g_mkn);
                        for j = 1:K
                            sum_g_tilde = sum_g_tilde + grad_g_mkn(:,j)' * (t_var - t_old);
                        end
                        sum_g_tilde = sum_g_tilde + sigma2;
                        tilde_f1 = log(sum_g_tilde) / log(2);
                        
                        % f̂₂(t) = f₂(t_old) + ∇f₂(t_old)ᵀ(t - t_old)
                        I_mk = sum(g_mkn) - g_mkn(k) + sigma2;
                        f2_val = log(I_mk) / log(2);
                        grad_g_interference = sum(grad_g_mkn, 2) - grad_g_mkn(:, k);
                        grad_f2 = (1 / (log(2) * I_mk)) * grad_g_interference;
                        hat_f2 = f2_val + grad_f2' * (t_var - t_old);
                        
                        % 累加到目标
                        obj_expr = obj_expr + (tilde_f1 - hat_f2);
                    end
                end
            end
            
            maximize(obj_expr)
            
            subject to
                % 边界约束
                t_var >= t_start;
                t_var <= t_end;
                t_var(1) >= t_start;
                t_var(Na) <= t_end;
                
                % 最小间距约束（排序+间距）
                for i = 1:Na-1
                    t_var(i+1) >= t_var(i) + d_min;
                end
                
                % 信任域约束（可选）
                if use_trust_region
                    for i = 1:Na
                        t_var(i) >= t_old(i) - delta_t_max;
                        t_var(i) <= t_old(i) + delta_t_max;
                    end
                end
        cvx_end
        
        % 检查CVX状态
        if ~strcmp(cvx_status, 'Solved') && ~strcmp(cvx_status, 'Inaccurate/Solved')
            warning('CVX未收敛，状态: %s', cvx_status);
            break;
        end
        
        % 更新位置
        t_new = t_var;
        
        % 计算新目标值
        R_new = compute_sum_rate_with_t(t_new, q_traj, alpha_mkn, W_mkn, R_mkn, ...
            u, H, M, K, N, Na, kappa, sigma2);
        
        % 收敛判断
        if sca_iter > 1
            improvement = abs(R_new - R_old) / (abs(R_old) + 1e-8);
            fprintf('    迭代 %d: R_sum = %.4f bps/Hz, 改善 = %.6f%%\n', ...
                sca_iter, R_new, improvement * 100);
            if improvement < tol
                fprintf('  [位置优化] ✅ 收敛！总迭代次数: %d\n', sca_iter);
                t_opt = t_new;
                return;
            end
        end
        
        % 更新迭代点
        t_old = t_new;
    end
    
    % 达到最大迭代次数
    fprintf('  [位置优化] ⚠️ 达到最大迭代次数 %d\n', max_sca_iter);
    t_opt = t_old;
end

%% 辅助函数1: 确保位置可行
function t_feas = ensure_feasible(t, t_start, t_end, d_min, Na)
    t = t(:);
    t_feas = min(max(t, t_start), t_end);
    for i = 2:Na
        t_feas(i) = max(t_feas(i), t_feas(i-1) + d_min);
    end
    if t_feas(end) > t_end
        t_feas = linspace(t_start, t_end, Na)';
    end
end

%% 辅助函数2: 计算给定位置的和速率
function R_sum = compute_sum_rate_with_t(t, q_traj, alpha_mkn, W_mkn, R_mkn, ...
    u, H, M, K, N, Na, kappa, sigma2)
    R_sum = 0;
    for n = 1:N
        for m = 1:M
            for k = 1:K
                if alpha_mkn(m, k, n) == 1
                    % 计算信道
                    h_mk = compute_channel_t(m, k, n, t, q_traj, u, H, Na, kappa);
                    
                    % 信号功率
                    signal_power = real(h_mk' * W_mkn{m,k,n} * h_mk);
                    
                    % 干扰功率
                    interference = 0;
                    for l = 1:M
                        for i = 1:K
                            if ~(l == m && i == k)
                                h_lk = compute_channel_t(l, k, n, t, q_traj, u, H, Na, kappa);
                                interference = interference + real(h_lk' * W_mkn{l,i,n} * h_lk);
                            end
                        end
                        % 感知干扰：只计算当前服务GBS的感知信号
                        if l == m
                            h_mk_sense = compute_channel_t(m, k, n, t, q_traj, u, H, Na, kappa);
                            interference = interference + real(h_mk_sense' * R_mkn{m,1,n} * h_mk_sense);
                        end
                    end
                    
                    % SINR和速率
                    SINR = signal_power / (interference + sigma2);
                    R_sum = R_sum + log2(1 + max(SINR, 1e-12));
                end
            end
        end
    end
end

%% 辅助函数3: 计算信道（带位置参数）
function h = compute_channel_t(m, k, n, t, q_traj, u, H, Na, kappa)
    qk_2d = squeeze(q_traj(k, :, n));
    um = u(m, :);
    dx = qk_2d(1) - um(1);
    dy = qk_2d(2) - um(2);
    dist_3d = sqrt(dx^2 + dy^2 + H(k)^2);
    beta = kappa / dist_3d^2;
    cos_theta = H(k) / dist_3d;
    g = exp(1j * 2*pi * t(:) * cos_theta);
    h = sqrt(beta) * g;
end

%% 辅助函数4: 计算所有梯度
function [g_all, grad_g_all] = compute_all_gradients(t, q_traj, alpha_mkn, W_mkn, ...
    u, H, M, K, N, Na, kappa)
    g_all = cell(M, K, N);
    grad_g_all = cell(M, K, N);
    
    for n = 1:N
        for m = 1:M
            for k = 1:K
                if alpha_mkn(m, k, n) == 0
                    g_all{m,k,n} = zeros(K, 1);
                    grad_g_all{m,k,n} = zeros(Na, K);
                    continue;
                end
                
                % 计算信道和角度
                h_mk = compute_channel_t(m, k, n, t, q_traj, u, H, Na, kappa);
                qk_2d = squeeze(q_traj(k, :, n));
                um = u(m, :);
                dx = qk_2d(1) - um(1);
                dy = qk_2d(2) - um(2);
                dist_3d = sqrt(dx^2 + dy^2 + H(k)^2);
                cos_theta = H(k) / dist_3d;
                
                % 计算所有j的g_mjk和梯度
                g_vec = zeros(K, 1);
                grad_g_mat = zeros(Na, K);
                
                for j = 1:K
                    % 提取波束向量
                    [V, D] = eig(W_mkn{m,j,n});
                    [~, idx] = max(diag(D));
                    w_mj = V(:,idx) * sqrt(max(D(idx,idx), 0));
                    
                    % g_mjk = |w_mj' * h_mk|^2
                    g_vec(j) = abs(w_mj' * h_mk)^2;
                    
                    % 计算梯度 ∂g/∂t
                    wHh = w_mj' * h_mk;
                    phi = 2*pi * cos_theta;
                    for l = 1:Na
                        d_wHh_dtl = 1j * phi * conj(w_mj(l)) * h_mk(l);
                        grad_g_mat(l, j) = 2 * real(conj(wHh) * d_wHh_dtl);
                    end
                end
                
                g_all{m,k,n} = g_vec;
                grad_g_all{m,k,n} = grad_g_mat;
            end
        end
    end
end


