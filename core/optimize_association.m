function alpha = optimize_association(h_mkn_precomputed, W, R, Pmax, sigma2, M, K, N, Na)
    % OPTIMIZE_ASSOCIATION - 使用预计算信道和MRT假设的关联优化
    % 输入：
    %   h_mkn_precomputed : cell{M,K,N}，预计算的信道矩阵 h_{m,k}[n]
    %   W        : cell{M,K,N}，通信波束协方差 W_{m,k}[n]（用于干扰计算）
    %   R        : cell{M,1,N}，感知信号协方差 R_m[n]（用于干扰计算）
    %   Pmax     : 最大发射功率（用于MRT假设）
    %   sigma2   : 噪声功率
    %   M, K, N  : GBS数、UAV数、时隙数
    %   Na       : 天线数
    % 输出：
    %   alpha    : MxKxN，关联变量 alpha_{m,k}[n]

    alpha = zeros(M, K, N);
    
    fprintf('🔧 优化关联：解决W-α循环依赖，动态构造候选波束矩阵...\n');
    fprintf('✅ 策略：现有W矩阵用于已关联GBS，零W矩阵时动态构造MRT波束评估\n');
    fprintf('  显示前5个时隙的详细选择过程（解决循环依赖问题）:\n');
    
    for n = 1:N
        for k = 1:K
            rates = zeros(M, 1);  % 存储每个GBS的服务速率
            
            for m = 1:M
                % 使用预计算的信道 h_{m,k}[n]
                h_mkn = h_mkn_precomputed{m, k, n};
                if any(isnan(h_mkn)) || any(isinf(h_mkn))
                    rates(m) = -inf;
                    continue;
                end
                
                % 🔧 修复：使用实际的波束矩阵W而非MRT假设
                % 确保关联优化基于真实系统性能，而不是理想假设
                
                % 🔧 修复：为候选关联动态构造MRT波束矩阵
                % 解决循环依赖：W依赖关联，但关联优化又需要评估W
                
                % 检查当前W矩阵是否有效（非零）
                W_current = W{m, k, n};
                if norm(W_current, 'fro') < 1e-10
                    % 当前W矩阵为零，动态构造MRT波束用于评估
                    power_per_user = Pmax * 0.4;  % 与initial.m保持一致
                    w_mrt = sqrt(power_per_user) * h_mkn / norm(h_mkn);
                    W_eval = w_mrt * w_mrt';
                else
                    % 使用现有的W矩阵
                    W_eval = W_current;
                end
                
                % 临时设置W矩阵和关联矩阵用于评估
                W_temp = W;
                W_temp{m, k, n} = W_eval;
                alpha_temp = zeros(M, K, 1);
                alpha_temp(m, k, 1) = 1;  % 假设UAV k由GBS m服务
                
                % 使用标准SINR计算函数
                [~, rate] = compute_sinr(h_mkn_precomputed, W_temp, R, alpha_temp, sigma2, m, k, 1, M, K);
                rates(m) = rate;
            end
            
            % 选择最大速率的GBS
            [~, m_star] = max(rates);
            alpha(m_star, k, n) = 1;
            
            % 显示前5个时隙的详细选择过程
            if n <= 5
                fprintf('    时隙%d UAV%d: 速率[%.2f, %.2f, %.2f] -> 选择GBS%d (速率=%.2f)\n', ...
                    n, k, rates(1), rates(2), rates(3), m_star, rates(m_star));
            end
        end
    end
    
    fprintf('  关联优化完成！使用预计算信道，避免了%d次重复信道计算\n', M*K*N);
    
    %% 局部交换改进：在贪心初值基础上进一步提升速率
    fprintf('\n--- 局部交换改进：对每个时隙执行 UAV-GBS 关联对调 ---\n');
    for n = 1:N
        alpha_slot = alpha(:,:,n);
        current_slot_rate = compute_slot_rate(alpha_slot, n);
        swap_counter = 0;

        improved = true;
        while improved
            improved = false;
            for k1 = 1:K-1
                m1 = find(alpha_slot(:,k1), 1);
                if isempty(m1)
                    continue;
                end
                for k2 = k1+1:K
                    m2 = find(alpha_slot(:,k2), 1);
                    if isempty(m2) || m1 == m2
                        continue;
                    end

                    swapped_slot = alpha_slot;
                    swapped_slot(m1, k1) = 0;
                    swapped_slot(m2, k2) = 0;
                    swapped_slot(m1, k2) = 1;
                    swapped_slot(m2, k1) = 1;

                    new_rate = compute_slot_rate(swapped_slot, n);

                    if new_rate > current_slot_rate + 1e-4
                        alpha_slot = swapped_slot;
                        current_slot_rate = new_rate;
                        swap_counter = swap_counter + 1;
                        improved = true;
                        break;
                    end
                end
                if improved
                    break;
                end
            end
        end

        alpha(:,:,n) = alpha_slot;
        if swap_counter > 0 && n <= 5
            fprintf('  时隙%d: 执行局部交换 %d 次，槽内和速率提升至 %.4f bps/Hz\n', n, swap_counter, current_slot_rate);
        end
    end

    % 详细验证关联选择过程
    fprintf('\n--- 关联优化结果验证 ---\n');
    fprintf('前5个时隙的详细关联信息：\n');
    for n = 1:min(5, N)
        for k = 1:K
            m_star = find(alpha(:,k,n));
            if ~isempty(m_star)
                h_mk = h_mkn_precomputed{m_star,k,n};
                % 使用实际波束计算真实速率（与主系统一致）
                W_actual = W{m_star, k, n};
                rate_actual = log2(1 + real(h_mk' * W_actual * h_mk) / sigma2);
                fprintf('  时隙%d UAV%d -> GBS%d (实际速率=%.4f bps/Hz, |h|²=%.2e)\n', ...
                    n, k, m_star, rate_actual, norm(h_mk)^2);
            end
        end
    end
    
    % 统计关联分布
    gbs_usage = zeros(M, 1);
    for m = 1:M
        gbs_usage(m) = sum(sum(alpha(m, :, :)));
    end
    fprintf('\nGBS使用统计（总连接次数）：\n');
    for m = 1:M
        fprintf('  GBS%d: %d次连接 (%.1f%%)\n', m, gbs_usage(m), gbs_usage(m)/(K*N)*100);
    end
    
    %% 嵌套函数：速率评估与波束获取
    function slot_rate = compute_slot_rate(alpha_slot, slot_idx)
        slot_rate = 0;
        for k_eval = 1:K
            m_eval = find(alpha_slot(:, k_eval), 1);
            if isempty(m_eval)
                continue;
            end

            h_serv = h_mkn_precomputed{m_eval, k_eval, slot_idx};
            if isempty(h_serv)
                continue;
            end

            W_eval = get_effective_W(m_eval, k_eval, slot_idx);
            signal_power = real(h_serv' * W_eval * h_serv);

            interference = 0;
            for m_other = 1:M
                h_to_k = h_mkn_precomputed{m_other, k_eval, slot_idx};
                if isempty(h_to_k)
                    continue;
                end

                if ~isempty(R) && ~isempty(R{m_other,1,slot_idx})
                    interference = interference + real(h_to_k' * R{m_other,1,slot_idx} * h_to_k);
                end

                for k_other = 1:K
                    if alpha_slot(m_other, k_other) == 0
                        continue;
                    end
                    if m_other == m_eval && k_other == k_eval
                        continue;
                    end

                    W_other = get_effective_W(m_other, k_other, slot_idx);
                    if isempty(W_other) || all(W_other(:) == 0)
                        continue;
                    end
                    interference = interference + real(h_to_k' * W_other * h_to_k);
                end
            end

            SINR_val = signal_power / (interference + sigma2);
            slot_rate = slot_rate + log2(1 + max(SINR_val, 0));
        end
    end

    function W_eff = get_effective_W(m_idx, k_idx, slot_idx)
        if m_idx < 1 || m_idx > M || k_idx < 1 || k_idx > K
            W_eff = zeros(Na, Na);
            return;
        end

        W_candidate = W{m_idx, k_idx, slot_idx};
        if isempty(W_candidate) || norm(W_candidate, 'fro') < 1e-10
            h_serv = h_mkn_precomputed{m_idx, k_idx, slot_idx};
            if isempty(h_serv) || norm(h_serv) < 1e-9
                W_eff = zeros(Na, Na);
            else
                power_per_user = Pmax * 0.4;
                w_tmp = sqrt(power_per_user) * h_serv / max(norm(h_serv), 1e-9);
                W_eff = w_tmp * w_tmp';
            end
        else
            W_eff = W_candidate;
        end
    end
end

%% 缺失的辅助函数
function [SINR_val, rate] = compute_sinr(h_mkn_precomputed, W, R, alpha, sigma2, m_eval, k_eval, n_eval, M, K)
% 计算指定用户的SINR和速率
    h_mk = h_mkn_precomputed{m_eval, k_eval, n_eval};
    if isempty(h_mk)
        SINR_val = 0;
        rate = 0;
        return;
    end
    
    % 信号功率
    signal_power = real(h_mk' * W{m_eval, k_eval, n_eval} * h_mk);
    
    % 干扰功率
    interference = 0;
    for l = 1:M
        for i = 1:K
            if ~(l == m_eval && i == k_eval) && alpha(l, i, n_eval) == 1
                h_lk = h_mkn_precomputed{l, k_eval, n_eval};
                if ~isempty(h_lk) && ~isempty(W{l, i, n_eval})
                    interference = interference + real(h_lk' * W{l, i, n_eval} * h_lk);
                end
            end
        end
        % 感知信号干扰
        if ~isempty(R) && ~isempty(R{l, 1, n_eval})
            h_lk = h_mkn_precomputed{l, k_eval, n_eval};
            if ~isempty(h_lk)
                interference = interference + real(h_lk' * R{l, 1, n_eval} * h_lk);
            end
        end
    end
    
    % SINR和速率
    SINR_val = signal_power / (interference + sigma2);
    rate = log2(1 + max(SINR_val, 1e-12));
end
