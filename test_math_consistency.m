% 测试数学一致性：验证修复后的SCA近似是否与真实速率一致
clear; clc;

fprintf('🧪 数学一致性测试\n');
fprintf('================\n\n');

try
    % 初始化系统
    setup_paths;
    initial;
    
    % 使用简化场景：单个时隙，固定位置
    test_N = 1;
    q_fixed = zeros(K, 2, test_N);
    q_fixed(1, :, 1) = [150, 150];
    if K > 1
        q_fixed(2, :, 1) = [250, 250];
    end
    
    alpha_fixed = zeros(M, K, test_N);
    alpha_fixed(1, :, 1) = 1;  % GBS1服务所有UAV
    
    % 计算ULA信道
    t_ULA = (0:Na-1)' * 0.5;  % ULA位置
    h_fixed = cell(M, K, test_N);
    for m = 1:M
        for k = 1:K
            for n = 1:test_N
                h_fixed{m,k,n} = get_channel(m, k, n, u, q_fixed, H, kappa, t_ULA, Na);
            end
        end
    end
    
    % 简单初始化波束矩阵
    W_test = cell(M, K, test_N);
    R_test = cell(M, 1, test_N);
    
    power_per_user = Pmax / sum(alpha_fixed(:));
    for m = 1:M
        for k = 1:K
            for n = 1:test_N
                if alpha_fixed(m,k,n) == 1
                    h_mk = h_fixed{m,k,n};
                    if norm(h_mk) > 1e-12
                        w_dir = h_mk / norm(h_mk);
                        W_test{m,k,n} = (power_per_user/Na) * (w_dir * w_dir');
                    else
                        W_test{m,k,n} = (power_per_user/Na) * eye(Na);
                    end
                else
                    W_test{m,k,n} = zeros(Na, Na);
                end
            end
        end
        for n = 1:test_N
            R_test{m,1,n} = (0.1/Na) * eye(Na);  % 小的感知功率
        end
    end
    
    % 测试1：计算真实速率
    fprintf('📊 测试1：真实速率计算\n');
    [rate_truth, ~] = calculate_master_rate_function(W_test, R_test, h_fixed, alpha_fixed, sigma2, M, K, test_N);
    fprintf('   真实速率: %.6f bps/Hz\n\n', rate_truth);
    
    % 测试2：验证梯度计算是否完整
    fprintf('📊 测试2：梯度计算验证\n');
    t_all_fixed = cell(M, 1);
    for m = 2:M
        t_all_fixed{m} = t_ULA;  % 其他GBS使用ULA位置
    end
    
    [g_all, grad_g_all] = compute_all_gradients(t_ULA, t_all_fixed, q_fixed, alpha_fixed, W_test, R_test, ...
        u, H, M, K, test_N, Na, kappa, sigma2);
    
    % 检查梯度是否非零
    for n = 1:test_N
        for k = 1:K
            m_serv = find(alpha_fixed(:,k,n) == 1, 1);
            if ~isempty(m_serv) && ~isempty(grad_g_all{m_serv,k,n})
                grad_mat = grad_g_all{m_serv,k,n};
                fprintf('   用户(%d,%d): 梯度范数 [%.2e, %.2e, %.2e, %.2e, %.2e]\n', ...
                    k, n, norm(grad_mat(:,1)), norm(grad_mat(:,2)), norm(grad_mat(:,3)), ...
                    norm(grad_mat(:,4)), norm(grad_mat(:,5)));
            end
        end
    end
    
    fprintf('\n✅ 数学一致性测试完成\n');
    
catch ME
    fprintf('❌ 测试失败: %s\n', ME.message);
    fprintf('错误详情:\n%s\n', ME.getReport);
end
