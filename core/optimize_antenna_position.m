function [t_opt, obj_history] = optimize_antenna_position(...
    q_traj, alpha_mkn, W_mkn, R_mkn, ...
    u, v, H, H_sense, M, K, N, Na, Q, t_init, t_all_fixed, t_start, t_end, d_min, ...
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
        % 计算当前真实和速率 - 使用唯一真理函数
        % 步骤1: 预计算当前位置的信道矩阵（统一计算，避免重复）
        h_mkn_current = cell(M, K, N);
        for m = 1:M
            for k = 1:K
                for n = 1:N
                    if m == 1
                        h_mkn_current{m,k,n} = get_channel(m, k, n, u, q_traj, H, kappa, t_old, Na);
                    else
                        h_mkn_current{m,k,n} = get_channel(m, k, n, u, q_traj, H, kappa, t_all_fixed{m}, Na);
                    end
                end
            end
        end
        
        % 步骤2: 使用统一的信道矩阵计算真实速率
        [R_old, ~] = calculate_master_rate_function(W_mkn, R_mkn, h_mkn_current, alpha_mkn, sigma2, M, K, N);
        obj_history(end+1) = R_old;
        
        if sca_iter == 1
            fprintf('    迭代 %d: R_sum = %.4f bps/Hz\n', sca_iter, R_old);
        end
        
        % 步骤3: 论文2两层SCA - 基于唯一数学真理
        % 第一层：线性化功率项 g(t) = h(t)^H W h(t)
        % 第二层：DC近似 log₂(S+I+N) - log₂(I+N)
        
        % 使用calculate_master_rate_function获取真实的功率分解（唯一真理）
        [R_old_verify, power_breakdown] = calculate_master_rate_function(W_mkn, R_mkn, h_mkn_current, alpha_mkn, sigma2, M, K, N);
        
        % 验证数学一致性
        if abs(R_old - R_old_verify) > 1e-6
            warning('Step 6数学不一致: R_old=%.4f vs R_old_verify=%.4f', R_old, R_old_verify);
        end
        
        % 计算所有功率项的梯度（基于真实功率值）
        [power_gradients] = compute_power_gradients_from_breakdown(power_breakdown, t_old, t_all_fixed, ...
            h_mkn_current, alpha_mkn, W_mkn, R_mkn, u, q_traj, H, M, K, N, Na, kappa);
        
        % 构建CVX优化问题（论文2两层SCA）
        cvx_begin quiet
            cvx_precision low
            variable t_var(Na, 1)
            
            % 目标函数：基于论文1的DC方法应用于位置变量t
            obj_expr = 0;
            
            % 对每个有效链路应用两层SCA
            for n = 1:N
                for k = 1:K
                    m_serv = find(alpha_mkn(:,k,n) == 1, 1);
                    
                    if ~isempty(m_serv) && ~isempty(W_mkn{m_serv,k,n})
                        % 从power_breakdown获取当前功率值
                        S_old = power_breakdown.signal{m_serv,k,n};
                        I_comm_old = power_breakdown.comm_interference{m_serv,k,n};
                        I_sense_old = power_breakdown.sense_interference{m_serv,k,n};
                        
                        % 总功率项
                        numerator_old = S_old + I_comm_old + I_sense_old + sigma2;  % S+I+N
                        denominator_old = I_comm_old + I_sense_old + sigma2;        % I+N
                        
                        % 从梯度结构获取对应梯度
                        if isfield(power_gradients, 'signal') && ~isempty(power_gradients.signal{m_serv,k,n})
                            grad_S = power_gradients.signal{m_serv,k,n};
                            grad_I_comm = power_gradients.comm_interference{m_serv,k,n};
                            grad_I_sense = power_gradients.sense_interference{m_serv,k,n};
                            
                            % 总梯度
                            grad_numerator = grad_S + grad_I_comm + grad_I_sense;    % ∇(S+I+N)
                            grad_denominator = grad_I_comm + grad_I_sense;          % ∇(I+N)
                            
                            % 第一层SCA：线性化功率项
                            % g̃(t) = g(t⁰) + ∇g(t⁰)ᵀ(t - t⁰)
                            numerator_linearized = numerator_old + grad_numerator' * (t_var - t_old);
                            denominator_linearized = denominator_old + grad_denominator' * (t_var - t_old);
                            
                            % 第二层SCA：DC近似 log₂(g̃₁) - log₂(g̃₂)
                            % Concave₁: log₂(g̃₁) - 保持凹性
                            concave1 = log(numerator_linearized) / log(2);
                            
                            % Concave₂: log₂(g̃₂) - 用仿射上界替换
                            % log₂(g̃₂) ≈ log₂(g₂⁰) + (∇g₂⁰/g₂⁰)/ln(2) * (g̃₂ - g₂⁰)
                            affine2_const = log(denominator_old) / log(2);
                            affine2_linear = (grad_denominator' * (t_var - t_old)) / (log(2) * denominator_old);
                            affine2 = affine2_const + affine2_linear;
                            
                            % 最终SCA目标：Concave₁ - Affine₂
                            rate_term = concave1 - affine2;
                            obj_expr = obj_expr + rate_term;
                        end
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
                
                % FAS感知约束（论文Equation 14的严格SCA凹下界实现）
                fprintf('    [SCA] 构建感知约束的凹二次下界 (论文Eq.14)\n');
                
                for n = 1:N
                    for q_idx = 1:Q
                        % 基本参数
                        dx = v(q_idx, 1) - u(1, 1);
                        dy = v(q_idx, 2) - u(1, 2);
                        dist = sqrt(dx^2 + dy^2 + H_sense^2);
                        path_loss = 1 / (dist^2);
                        cos_theta = H_sense / dist;
                        
                        % 复合波束矩阵 R_w = R_total
                        R_w = zeros(Na, Na);
                        for k = 1:K
                            if ~isempty(W_mkn{1, k, n})
                                R_w = R_w + W_mkn{1, k, n};
                            end
                        end
                        if ~isempty(R_mkn{1, 1, n})
                            R_w = R_w + R_mkn{1, 1, n};
                        end
                        
                        % === 论文Eq.(14) SCA参数计算 ===
                        % Step 1: 基本常数
                        t_tilde = t_old;  % Na×1 当前迭代点
                        v_coeff = 2 * pi * cos_theta;  % 标量
                        
                        % Step 2: 中间变量
                        R_abs = abs(R_w);  % Na×Na 绝对值矩阵
                        
                        % 相位差矩阵 F_phase (Na×Na)
                        F_phase = zeros(Na, Na);
                        for nn = 1:Na
                            for mm = 1:Na
                                F_phase(nn,mm) = v_coeff * (t_tilde(nn) - t_tilde(mm)) + angle(R_w(mm,nn));
                            end
                        end
                        
                        % Step 3: 计算SCA参数 D, d, c
                        % r向量: 每行求和 (Na×1)
                        r_vec = sum(R_abs, 2);
                        
                        % D矩阵: 负半定 (保证凹性) (Na×Na)
                        D_matrix = -v_coeff^2 * (diag(r_vec) - R_abs);
                        
                        % d向量 (Na×1)
                        d_vec = zeros(Na, 1);
                        for nn = 1:Na
                            sum_term_1 = sum(R_abs(:,nn) .* sin(F_phase(nn,:)'));
                            sum_term_2 = sum(R_abs(:,nn) .* (t_tilde(nn) - t_tilde));
                            d_vec(nn) = v_coeff * sum_term_1 - v_coeff^2 * sum_term_2;
                        end
                        
                        % c标量
                        c_scalar = 0;
                        for mm = 1:Na
                            for nn = 1:Na
                                f_nm = F_phase(nn,mm);
                                t_diff = t_tilde(nn) - t_tilde(mm);
                                term = cos(f_nm) + v_coeff*sin(f_nm)*t_diff - 0.5*v_coeff^2*t_diff^2;
                                c_scalar = c_scalar + R_abs(mm,nn) * term;
                            end
                        end
                        
                        % === CVX凸约束 ===
                        % g(t|t_tilde) = t'*D*t - 2*d'*t + c >= Gamma_prime
                        Gamma_prime = dist^2 * Gamma;  % 包含路径损耗的感知阈值
                        
                        % 检查D矩阵性质
                        D_eigs = eig(D_matrix);
                        D_min_eig = min(D_eigs);
                        D_max_eig = max(D_eigs);
                        
                        
                        % 确保D矩阵是负半定的（凹函数要求）
                        if D_max_eig > 1e-10
                            fprintf('      警告: D矩阵不是负半定的，调整为负半定\n');
                            D_matrix = D_matrix - (D_max_eig + 1e-8) * eye(Na);
                        end
                        
                        % 凹二次下界约束 (凹 >= 常数 是凸约束)
                        % 使用CVX兼容的二次形式替代quad_form
                        % 由于D_matrix是负半定的，我们可以分解它
                        [V, D_eigs] = eig(D_matrix);
                        D_eigs = diag(D_eigs);
                        D_eigs = min(D_eigs, 0);  % 确保非正
                        
                        % 使用变量替换: y = V'*t_var
                        % 则 t'*D*t = y'*diag(D_eigs)*y = sum(D_eigs .* y.^2)
                        y_var = V' * t_var;
                        quad_terms = 0;
                        for ii = 1:length(D_eigs)
                            if abs(D_eigs(ii)) > 1e-12
                                quad_terms = quad_terms + D_eigs(ii) * square(y_var(ii));
                            end
                        end
                        
                        % 完整约束: quad_terms - 2*d'*t + c >= Gamma_prime
                        quad_terms - 2*d_vec'*t_var + c_scalar >= Gamma_prime;
                    end
                end
        cvx_end
        
        % 检查CVX状态
        if ~strcmp(cvx_status, 'Solved') && ~strcmp(cvx_status, 'Inaccurate/Solved')
            warning('CVX未收敛，状态: %s', cvx_status);
            
            % 诊断约束冲突
            fprintf('  [诊断] 约束冲突分析:\n');
            
            % 使用与主算法一致的感知功率计算
            fprintf('    使用标准感知功率计算函数...\n');
            
            % 创建当前位置的cell格式
            t_current_cell = cell(M, 1);
            for m = 1:M
                if m == 1
                    t_current_cell{m} = t_old;
                else
                    t_current_cell{m} = (0:Na-1)' * 0.5;  % 其他GBS使用默认ULA
                end
            end
            
            % 使用标准函数计算感知功率
            current_sensing = compute_sensing_power(alpha_mkn, W_mkn, R_mkn, ...
                u, v, M, Q, N, Na, kappa, t_current_cell, H_sense);
            
            fprintf('    当前平均感知功率: %.4e W\n', current_sensing);
            fprintf('    感知阈值 Gamma: %.4e W\n', Gamma);
            fprintf('    需要提升倍数: %.2f\n', Gamma / current_sensing);
            fprintf('    建议: 降低感知阈值或增加感知波束功率\n');
            
            break;
        end
        
        % 更新位置
        t_new = t_var;
        
        % 计算新目标值 - 使用唯一真理函数
        % 构建新位置下的信道矩阵
        h_new = cell(M, K, N);
        for m = 1:M
            for k = 1:K
                for n = 1:N
                    if m == 1
                        h_new{m,k,n} = get_channel(m, k, n, u, q_traj, H, kappa, t_new, Na);
                    else
                        h_new{m,k,n} = get_channel(m, k, n, u, q_traj, H, kappa, t_all_fixed{m}, Na);
                    end
                end
            end
        end
        [R_new, ~] = calculate_master_rate_function(W_mkn, R_mkn, h_new, alpha_mkn, sigma2, M, K, N);
        
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

%% 注意：compute_sum_rate_with_t 函数已删除
% 所有速率计算现在统一使用 calculate_master_rate_function

%% 辅助函数3: 计算信道（带位置参数）
function h = compute_channel_t(m, k, n, t, q_traj, u, H, Na, kappa)
    qk_2d = squeeze(q_traj(k, :, n));
    um = u(m, :);
    dx = qk_2d(1) - um(1);
    dy = qk_2d(2) - um(2);
    
    % 修复数组索引：H可能是标量或向量
    if length(H) == 1
        H_k = H;  % 标量高度
    else
        H_k = H(k);  % 向量高度
    end
    
    dist_3d = sqrt(dx^2 + dy^2 + H_k^2);
    beta = kappa / dist_3d^2;
    cos_theta = H_k / dist_3d;
    g = exp(1j * 2*pi * t(:) * cos_theta);
    h = sqrt(beta) * g;
end

%% 辅助函数4: 基于功率分解的梯度计算 - 确保唯一数学真理
function [power_gradients] = compute_power_gradients_from_breakdown(power_breakdown, t_old, t_all_fixed, ...
    h_mkn_current, alpha_mkn, W_mkn, R_mkn, u, q_traj, H, M, K, N, Na, kappa)
    % 基于calculate_master_rate_function的真实功率分解计算梯度
    % 不再重新计算功率值，确保与"唯一真理"完全一致
    
    % 初始化梯度结构
    power_gradients.signal = cell(M, K, N);
    power_gradients.comm_interference = cell(M, K, N);
    power_gradients.sense_interference = cell(M, K, N);
    
    % 计算梯度：只有GBS1的位置是变量
    for n = 1:N
        for k = 1:K
            m_serv = find(alpha_mkn(:,k,n) == 1, 1);
            
            if ~isempty(m_serv) && ~isempty(W_mkn{m_serv,k,n}) && ~isempty(power_breakdown.signal{m_serv,k,n})
                % 1. 信号功率梯度 ∇S(t)
                if m_serv == 1  % 只有GBS1的位置是变量
                    h_mk = h_mkn_current{m_serv,k,n};
                    power_gradients.signal{m_serv,k,n} = compute_power_gradient_unified(h_mk, W_mkn{m_serv,k,n}, u, q_traj, H, k, n, Na, kappa);
                else
                    power_gradients.signal{m_serv,k,n} = zeros(Na, 1);
                end
                
                % 2. 通信干扰功率梯度 ∇I_comm(t)
                grad_comm = zeros(Na, 1);
                for l = 1:M
                    for i = 1:K
                        if ~(l == m_serv && i == k) && alpha_mkn(l,i,n) == 1
                            if l == 1  % 只有GBS1产生的干扰有梯度
                                h_lk = h_mkn_current{l,k,n};
                                if ~isempty(W_mkn{l,i,n})
                                    grad_comm = grad_comm + compute_power_gradient_unified(h_lk, W_mkn{l,i,n}, u, q_traj, H, k, n, Na, kappa);
                                end
                            end
                        end
                    end
                end
                power_gradients.comm_interference{m_serv,k,n} = grad_comm;
                
                % 3. 感知干扰功率梯度 ∇I_sense(t)
                grad_sense = zeros(Na, 1);
                for l = 1:M
                    if l == 1  % 只有GBS1产生的感知干扰有梯度
                        h_lk = h_mkn_current{l,k,n};
                        if ~isempty(R_mkn{l,1,n})
                            grad_sense = grad_sense + compute_power_gradient_unified(h_lk, R_mkn{l,1,n}, u, q_traj, H, k, n, Na, kappa);
                        end
                    end
                end
                power_gradients.sense_interference{m_serv,k,n} = grad_sense;
            else
                % 如果没有有效的功率分解，设置零梯度
                power_gradients.signal{m_serv,k,n} = zeros(Na, 1);
                power_gradients.comm_interference{m_serv,k,n} = zeros(Na, 1);
                power_gradients.sense_interference{m_serv,k,n} = zeros(Na, 1);
            end
        end
    end
end

%% 辅助函数：统一功率梯度计算（基于矩阵形式）
function grad = compute_power_gradient_unified(h, W, u, q_traj, H, k, n, Na, kappa)
    % 计算 ∇_t trace(W * h(t) * h(t)^H) 的梯度
    % 使用矩阵形式确保与calculate_master_rate_function完全一致
    grad = zeros(Na, 1);
    
    % 计算位置相关参数
    qk_2d = squeeze(q_traj(k, :, n));
    um = u(1, :);  % GBS1的位置
    dx = qk_2d(1) - um(1);
    dy = qk_2d(2) - um(2);
    
    if length(H) == 1
        H_k = H;
    else
        H_k = H(k);
    end
    
    dist_3d = sqrt(dx^2 + dy^2 + H_k^2);
    cos_theta = H_k / dist_3d;
    phi = 2*pi * cos_theta;
    
    % 计算梯度：∇ trace(W * h * h^H) = 2 * Re(trace(W * (∇h) * h^H))
    for l = 1:Na
        % ∇h/∇t_l 是一个向量，第l个元素是 1j*phi*h(l)，其他为0
        dh_dt = zeros(Na, 1);
        dh_dt(l) = 1j * phi * h(l);
        
        % 梯度公式：2 * Re(trace(W * (∇h * h^H)))
        grad(l) = 2 * real(trace(W * (dh_dt * h')));
    end
end


