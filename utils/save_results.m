%% ===================================================================
%                       保存结果脚本
%  功能: 将当前工作区中的AO算法结果保存到 .mat 文件中。
%  用法: 在成功运行 main_AO_algorithm.m 后运行此脚本。
% ===================================================================

fprintf('\n正在保存最终结果到 results_for_plotting.mat ...\n');

% 检查必需的变量是否存在于工作区
required_vars = {'q_current', 'W_current', 'R_current', 'alpha_current', ...
                 'u', 'v', 'H_sense', 'params', 'M', 'K', 'N', 'Na', ...
                 'kappa', 'd', 'H', 'sum_rate_history', ...
                 'min_sensing_power_history', 'constraint_violation_history', ...
                 'trust_region_history', 'alpha_history', 'W_history', ...
                 'q_history', 'rate_per_uav_history', 'initial_sum_rate', ...
                 'initial_min_sensing', 'final_iter', 'converged', 'h_mkn', 'sigma2'};

missing_vars = {};
for i = 1:length(required_vars)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', required_vars{i}))
        missing_vars{end+1} = required_vars{i};
    end
end

if ~isempty(missing_vars)
    fprintf('❌ 结果保存失败: 工作区缺少以下必需变量:\n');
    for i = 1:length(missing_vars)
        fprintf('  - %s\n', missing_vars{i});
    end
    error('请先完整运行 main_AO_algorithm.m 以生成这些变量。');
end


try
    % 在保存前，重新计算最终的和速率以确保一致性
    % 这是必要的，因为它是基于最终状态计算的
    final_sum_rate_check = compute_sum_rate(h_mkn, W_current, R_current, alpha_current, sigma2, M, K, N);
    
    save('results_for_plotting.mat', ...
        'q_current', 'W_current', 'R_current', 'alpha_current', ...
        'u', 'v', 'H_sense', 'params', ...
        'M', 'K', 'N', 'Na', 'kappa', 'd', 'H', ...
        'sum_rate_history', ...
        'min_sensing_power_history', ...
        'constraint_violation_history', ...
        'trust_region_history', ...
        'alpha_history', ...
        'W_history', ...
        'q_history', ...
        'rate_per_uav_history', ...
        'initial_sum_rate', ...
        'initial_min_sensing', ...
        'final_sum_rate_check', ...
        'final_iter', ...
        'converged', ...
        '-v7.3');
    fprintf('✅ 结果保存成功。\n');
catch ME
    fprintf('❌ 结果保存失败: %s\n', ME.message);
    fprintf('   错误发生在文件: %s, 第 %d 行\n', ME.stack(1).file, ME.stack(1).line);
end

% 同样，需要 compute_sum_rate 函数
function rate = compute_sum_rate(h_mkn_precomputed, W, R, alpha, sigma2, M, K, N)
% 计算真实和速率
    rate = 0;
    for n = 1:N
        for k = 1:K % 遍历所有用户
            m_serv = find(alpha(:,k,n) == 1, 1);
            if isempty(m_serv)
                continue;
            end
            
            h_mk = h_mkn_precomputed{m_serv,k,n};
            sig = real(h_mk' * W{m_serv,k,n} * h_mk);
            
            int = 0;
            % 蜂内和蜂间干扰
            for l = 1:M
                for i = 1:K
                    if ~(l==m_serv && i==k) && alpha(l,i,n) == 1
                        h_lk = h_mkn_precomputed{l,k,n}; % Correct interference channel
                        int = int + real(h_lk' * W{l,i,n} * h_lk);
                    end
                end
                % 来自所有GBS的感知信号干扰
                if ~isempty(R) && ~isempty(R{l,1,n})
                    h_lk = h_mkn_precomputed{l,k,n}; % Correct interference channel
                    int = int + real(h_lk' * R{l,1,n} * h_lk);
                end
            end
            
            SINR = sig / (int + sigma2);
            rate = rate + log2(1 + max(SINR, 1e-12));
        end
    end
end
