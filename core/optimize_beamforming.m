function [W_opt, R_opt, obj_history] = optimize_beamforming(...
    h_mkn_precomputed, alpha_mkn, R_mn_init, W_mkn_init, ...
    Pmax, Gamma, sigma2, M, K, N, Na, Q, v, u, H_sense, kappa, t)
% OPTIMIZE_BEAMFORMING - 基于SCA+SDR的ISAC波束成形优化（修复版）
%
% 输入:
%   h_mkn_precomputed : cell{M,K,N}, 信道向量 h_{m,k}[n]
%   alpha_mkn         : MxKxN, 关联变量 α_{m,k}[n]
%   R_mn_init         : cell{M,1,N}, 初始 R_m[n]
%   W_mkn_init        : cell{M,K,N}, 初始 W_{m,k}[n]
%   Pmax              : 标量, GBS最大发射功率
%   Gamma             : 标量, 最小感知照射功率 (W)
%   sigma2            : 噪声功率
%   M,K,N,Na          : 系统参数
%   Q                 : 感知点数量
%   v                 : Qx2, 感知点水平坐标 [x,y]
%   u                 : Mx2, GBS水平坐标 [x,y]
%   H_sense           : 感知点高度 (m)
%
% 输出:
%   W_opt       : cell{M,K,N}, 优化后的通信波束
%   R_opt       : cell{M,1,N}, 优化后的感知波束
%   obj_history : 历史目标值

    % 初始化
    W_opt = W_mkn_init;
    R_opt = R_mn_init;
    obj_history = [];
    
    % SCA 参数
    max_sca_iter = 20;
    tol = 1e-4;
    min_rate_approx = 1e-6;

    % 感知约束松弛设置（避免不可行）
    use_sensing_slack = true;   % 使用松弛约束，平衡通信与感知
    rho_slack = 1e6; % 增大惩罚系数，更重视感知约束（原1e3改为1e6，增大1000倍）

    % 若未提供位置向量 t，则回退为等间距（兼容旧调用）
    if ~exist('t','var') || isempty(t)
        d_lambda = 0.5;
        t = (0:Na-1)' * d_lambda;
    else
        t = t(:);
        if numel(t) ~= Na
            error('optimize_beamforming:InvalidT', '位置向量 t 的长度应为 Na');
        end
    end

    % 可行性预检
    sensing_stats = compute_sensing_statistics(alpha_mkn, W_mkn_init, R_mn_init, ...
        v, u, H_sense, kappa, M, K, N, Na);
    fprintf('  [预检] 初始最小感知功率: %.4e W (Gamma = %.4e W)\n', sensing_stats.min_power, Gamma);
    power_stats = compute_power_statistics(W_mkn_init, R_mn_init, Pmax, M, K, N);
    fprintf('  [预检] 最大功率占用: %.4f / %.4f W\n', power_stats.max_usage, Pmax);

    fprintf('--- 开始波束成形优化 (SCA+SDR) ---\n');

    for o = 1:max_sca_iter
        fprintf('=== SCA 迭代 %d ===\n', o);
        
        % 计算当前真实和速率作为目标值
        obj_current = compute_sum_rate(h_mkn_precomputed, W_opt, R_opt, alpha_mkn, sigma2, M, K, N);
        obj_history(end+1) = obj_current;
        fprintf('  当前和速率: %.4f bps/Hz\n', obj_current);

        % 构建SDP问题
        cvx_begin sdp quiet
            cvx_precision low % 提高求解鲁棒性
            variable W_mat(Na, Na, M, K, N) hermitian semidefinite
            variable R_mat(Na, Na, M, N) hermitian semidefinite
            if use_sensing_slack
                variable sense_slack(N, 1) nonnegative
            end

            % 目标函数：SCA近似（凹上界）
            obj_expr = sum_rate_SCA_approximation(...
                h_mkn_precomputed, alpha_mkn, W_mat, R_mat, ...
                W_opt, R_opt, sigma2, M, K, N);
            if use_sensing_slack
                obj_expr = obj_expr - rho_slack * sum(sense_slack);
            end
            maximize(obj_expr)

            subject to
                
                % 感知约束：每个感知点、每个时隙
                for n = 1:N
                    for q_idx = 1:Q
                        sense_power = 0;
                        for m = 1:M
                            % 3D距离
                            dx = v(q_idx,1) - u(m,1);
                            dy = v(q_idx,2) - u(m,2);
                            dz = H_sense;
                            dist_3d = sqrt(dx^2 + dy^2 + dz^2);

                            % 入射角和导向矢量（使用位置向量 t）
                            theta = acos(dz / dist_3d);
                            a_vec = exp(1j * 2*pi * t * cos(theta));
                            a_vec = a_vec / norm(a_vec);

                            % 修改：使用归一化功率（忽略kappa），与initial.m保持一致
                            % beta = kappa / (dist_3d^2);  % 旧版本
                            beta = 1 / (dist_3d^2);  % 新版本：归一化功率

                            % 通信信号贡献
                            for k = 1:K
                                if alpha_mkn(m,k,n) == 1
                                    sense_power = sense_power + ...
                                        beta * real(a_vec' * W_mat(:,:,m,k,n) * a_vec);
                                end
                            end

                            % 专用感知信号贡献
                            sense_power = sense_power + ...
                                beta * real(a_vec' * R_mat(:,:,m,n) * a_vec);
                        end

                        if use_sensing_slack
                            slack_term = sense_slack(n);
                        else
                            slack_term = 0;
                        end
                        sense_power >= Gamma - slack_term;
                    end
                end
                
                % 功率约束：每个GBS、每个时隙
                for n = 1:N
                    for m = 1:M
                        total_power = 0;
                        for k = 1:K
                            total_power = total_power + trace(W_mat(:,:,m,k,n));
                        end
                        total_power = total_power + trace(R_mat(:,:,m,n));
                        total_power <= Pmax;
                    end
                end
                
        cvx_end
        
        if ~strcmp(cvx_status, 'Solved') && ~strcmp(cvx_status, 'Inaccurate/Solved')
            warning('CVX未收敛，状态: %s，停止优化。', cvx_status);
            break;
        end

        if use_sensing_slack
            slack_value = full(sense_slack);
            fprintf('  松弛统计: max %.4e, mean %.4e, sum %.4e\n', ...
                max(slack_value), mean(slack_value), sum(slack_value));
        end

        % 更新解
        for m = 1:M
            for k = 1:K
                for n = 1:N
                    W_opt{m,k,n} = W_mat(:,:,m,k,n);
                end
            end
            for n = 1:N
                 R_opt{m,1,n} = R_mat(:,:,m,n);
            end
        end

        % 收敛判断
        if o > 1 && abs(obj_history(end) - obj_history(end-1)) < tol
            fprintf('✅ SCA 收敛！迭代次数: %d\n', o);
            break;
        end
    end

    % 秩一重构（简单主特征向量法）
    [W_opt, R_opt] = rank_one_reconstruction(W_opt, R_opt, h_mkn_precomputed, alpha_mkn, M, K, N, Na);

    % 最终约束统计输出
    final_sensing_stats = compute_sensing_statistics(alpha_mkn, W_opt, R_opt, v, u, H_sense, kappa, M, K, N, Na);
    final_power_stats = compute_power_statistics(W_opt, R_opt, Pmax, M, K, N);
    fprintf('  [最终] 最小感知功率: %.4e W, 最大功率占用: %.4f / %.4f W\n', ...
        final_sensing_stats.min_power, final_power_stats.max_usage, Pmax);

end


%% ========== 内部辅助函数 ==========
function rate = sum_rate_SCA_approximation(...
    h_mkn, alpha, W_cvx, R_cvx, W_old, R_old, sigma2, M, K, N)
% SCA 近似的和速率（凹函数上界）

    rate = 0;
    for n = 1:N
        for m = 1:M
            for k = 1:K
                if alpha(m,k,n) == 1
                    h_mk = h_mkn{m,k,n};
                    
                    % 分母：干扰 + 噪声（基于旧解）
                    den = sigma2;
                    for l = 1:M
                        for i = 1:K
                            if ~(l==m && i==k) && ~isempty(W_old{l,i,n})
                                h_lk = h_mkn{l,k,n};
                                den = den + real(h_lk' * W_old{l,i,n} * h_lk);
                            end
                        end
                        if ~isempty(R_old{l,1,n})
                            h_lk = h_mkn{l,k,n};
                            den = den + real(h_lk' * R_old{l,1,n} * h_lk);
                        end
                    end
                    
                    % 信号功率（凸函数）
                    signal_power = real(h_mk' * W_cvx(:,:,m,k,n) * h_mk);
                    
                    % SCA线性近似 log(1+x) ≈ log(1+x0) + (x-x0)/(ln2*(1+x0))
                    x0 = max(real(h_mk' * W_old{m,k,n} * h_mk) / den, 1e-6);
                    rate_term = log2(1 + x0) + (signal_power/den - x0) / (log(2)*(1 + x0));
                    rate = rate + rate_term;
                end
            end
        end
    end
end


function [W_recon, R_recon] = rank_one_reconstruction(W_in, R_in, h_mkn, alpha, M, K, N, Na)
% 使用主特征向量进行秩一重构

    W_recon = W_in;
    R_recon = R_in;

    for m = 1:M
        for k = 1:K
            for n = 1:N
                if alpha(m,k,n) == 1
                    W_star = W_in{m,k,n};
                    [V, D] = eig(W_star);
                    [~, idx] = max(diag(D));
                    v1 = V(:,idx);
                    lambda1 = D(idx,idx);
                    w_rank1 = sqrt(lambda1) * v1;
                    W_recon{m,k,n} = w_rank1 * w_rank1';
                end
            end
        end
        
        for n = 1:N
            if ~isempty(R_in{m,1,n})
                R_star = R_in{m,1,n};
                [V, D] = eig(R_star);
                [~, idx] = max(diag(D));
                v1 = V(:,idx);
                lambda1 = D(idx,idx);
                r_rank1 = sqrt(lambda1) * v1;
                R_recon{m,1,n} = r_rank1 * r_rank1';
            end
        end
    end
end

function stats = compute_power_statistics(W_cells, R_cells, Pmax, M, K, N)
    max_usage = 0;
    for n = 1:N
        for m = 1:M
            total_power = 0;
            for k = 1:K
                if m <= size(W_cells,1) && k <= size(W_cells,2) && n <= size(W_cells,3)
                    Wmk = W_cells{m,k,n};
                    if ~isempty(Wmk)
                        total_power = total_power + trace(Wmk);
                    end
                end
            end
            if m <= size(R_cells,1) && ~isempty(R_cells{m,1,n})
                total_power = total_power + trace(R_cells{m,1,n});
            end
            max_usage = max(max_usage, total_power);
        end
    end
    stats.max_usage = max_usage;
end