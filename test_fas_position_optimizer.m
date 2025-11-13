%% FAS位置优化器单元测试
% 专门测试 optimize_antenna_position.m 模块
% 在固定场景下验证位置优化功能

clear; clc; close all;

fprintf('🔧 FAS位置优化器单元测试\n');
fprintf('================================\n\n');

try
    %% 1. 初始化系统
    fprintf('1. 初始化系统...\n');
    setup_paths;
    initial;
    fprintf('   ✅ 系统初始化完成\n');
    
    %% 2. 搭建固定快照场景
    fprintf('2. 搭建固定快照场景...\n');
    
    % 固定UAV位置（使用与系统初始化相同的时隙数以确保一致性）
    test_N = N;  % 使用与系统相同的时隙数
    q_fixed = zeros(K, 2, test_N);
    % 为所有时隙设置相同的固定位置
    for n = 1:test_N
        q_fixed(1, :, n) = [150, 150];  % UAV1固定位置
        if K > 1
            q_fixed(2, :, n) = [250, 250];  % UAV2固定位置
        end
    end
    
    % 固定关联（基站1服务所有用户）
    alpha_fixed = zeros(M, K, test_N);
    for n = 1:test_N
        alpha_fixed(1, :, n) = 1;  % GBS1服务所有UAV
    end
    
    % 验证关联矩阵
    fprintf('   关联矩阵验证:\n');
    for m = 1:M
        users_served = find(alpha_fixed(m, :, 1) == 1);
        fprintf('     GBS %d 服务 UAV: [%s]\n', m, num2str(users_served));
    end
    
    % 固定感知方向和阈值
    theta_sense_test = 60 * pi/180;  % 60度感知方向
    Gamma_test = Gamma;  % 使用原始感知阈值测试修复后的SCA
    
    fprintf('   固定场景设置:\n');
    fprintf('     UAV位置: [%.0f,%.0f]', q_fixed(1,:,1));
    if K > 1, fprintf(', [%.0f,%.0f]', q_fixed(2,:,1)); end
    fprintf('\n');
    fprintf('     关联策略: GBS1服务所有UAV\n');
    fprintf('     感知方向: %.0f°\n', theta_sense_test*180/pi);
    fprintf('     感知阈值: %.2e W\n', Gamma_test);
    
    %% 3. 计算固定场景下的信道
    fprintf('3. 计算固定场景信道...\n');
    
    % 使用ULA初始位置计算信道
    t_ULA = (0:Na-1)' * d;  % ULA位置向量
    h_fixed = cell(M, K, test_N);
    
    for m = 1:M
        for k = 1:K
            for n = 1:test_N
                h_fixed{m,k,n} = get_channel(m, k, n, u, q_fixed, H, kappa, t_ULA, Na);
            end
        end
    end
    
    fprintf('   ✅ 信道计算完成\n');
    
    %% 4. 计算初始波束（基于ULA）
    fprintf('4. 计算初始波束（基于ULA）...\n');
    
    % 创建ULA位置向量的cell格式
    t_ULA_cell = cell(M, 1);
    for m = 1:M
        t_ULA_cell{m} = t_ULA;
    end
    
    % 使用真正的波束优化（这是关键！）
    fprintf('   正在调用 optimize_beamforming...\n');
    
    % 创建初始波束矩阵（作为优化起点）
    W_init_test = cell(M, K, test_N);
    R_init_test = cell(M, 1, test_N);
    
    % 简单初始化作为起点
    total_users = sum(alpha_fixed(:));
    power_per_user = Pmax * 0.7 / total_users;
    power_sensing = Pmax * 0.3;
    
    for m = 1:M
        for k = 1:K
            for n = 1:test_N
                if alpha_fixed(m,k,n) == 1
                    h_mk = h_fixed{m,k,n};
                    if norm(h_mk) > 1e-12
                        w_mrt = sqrt(power_per_user) * h_mk / norm(h_mk);
                        W_init_test{m,k,n} = w_mrt * w_mrt';
                    else
                        W_init_test{m,k,n} = (power_per_user/Na) * eye(Na);
                    end
                else
                    W_init_test{m,k,n} = zeros(Na, Na);
                end
            end
        end
        for n = 1:test_N
            R_init_test{m,1,n} = (power_sensing/Na) * eye(Na);
        end
    end
    
    % 调用真正的波束优化
    try
        [W_test, R_test, ~] = optimize_beamforming(...
            h_fixed, alpha_fixed, R_init_test, W_init_test, ...
            Pmax, Gamma_test, sigma2, M, K, test_N, Na, Q, v, u, H_sense, kappa, t_ULA_cell);
        fprintf('   ✅ 波束优化成功完成\n');
    catch ME
        fprintf('   ❌ 波束优化失败: %s\n', ME.message);
        fprintf('   错误详情:\n%s\n', ME.getReport);
        fprintf('   这是致命错误，必须修复！\n');
        return;  % 立即退出，不能继续
    end
    
    %% 5. 计算ULA基准性能
    fprintf('5. 计算ULA基准性能...\n');
    
    % 计算ULA和速率 - 使用唯一真理函数
    [sum_rate_ULA, ~] = calculate_master_rate_function(W_test, R_test, h_fixed, alpha_fixed, sigma2, M, K, test_N);
    
    % 计算ULA感知功率
    sensing_power_ULA = compute_sensing_power(alpha_fixed, W_test, R_test, ...
        u, v, M, Q, test_N, Na, kappa, t_ULA_cell, H_sense);
    
    fprintf('   ULA基准性能:\n');
    fprintf('     和速率: %.4f bps/Hz\n', sum_rate_ULA);
    fprintf('     感知功率: %.4e W (%.2f dBW)\n', sensing_power_ULA, 10*log10(sensing_power_ULA));
    if sensing_power_ULA >= Gamma_test
        fprintf('     感知约束: 满足\n');
    else
        fprintf('     感知约束: 不满足\n');
    end
    
    %% 6. 执行FAS位置优化（核心测试）
    fprintf('6. 执行FAS位置优化（核心测试）...\n');
    
    % 只优化GBS1的位置（简化测试）
    m_test = 1;
    fprintf('   测试GBS %d的位置优化...\n', m_test);
    
    tic;
    try
        % 创建所有GBS的位置信息（假设只优化GBS 1）
        t_all_fixed = cell(M, 1);
        for m = 1:M
            t_all_fixed{m} = t_ULA;  % 所有GBS初始都使用ULA位置
        end
        
        [t_opt, obj_history] = optimize_antenna_position(...
            q_fixed, alpha_fixed, W_test, R_test, ...
            u, v, H_sense, H_sense, M, K, test_N, Na, Q, t_ULA, t_all_fixed, t_start, t_end, d_min, ...
            kappa, Pmax, Gamma_test, sigma2);
        
        optimization_time = toc;
        
        % 检查优化是否真正成功
        if ~isempty(obj_history)
            fprintf('   优化历史: %d 次迭代\n', length(obj_history));
            % 这里需要检查CVX状态，但目前函数没有返回状态
            fprintf('   ⚠️ 需要检查CVX求解状态\n');
        else
            fprintf('   ❌ 位置优化失败：无优化历史\n');
            return;
        end
        
        fprintf('   耗时: %.2f 秒\n', optimization_time);
        
    catch ME
        fprintf('   ❌ 位置优化失败: %s\n', ME.message);
        fprintf('   错误详情: %s\n', ME.getReport);
        return;
    end
    
    %% 7. 分析优化结果
    fprintf('7. 分析优化结果...\n');
    
    % 检查1: 位置是否改变？
    position_change = norm(t_opt - t_ULA);
    fprintf('   位置变化分析:\n');
    fprintf('     ULA位置: [%.3f, %.3f, %.3f, %.3f, ...]\n', t_ULA(1:min(4,end)));
    fprintf('     FAS位置: [%.3f, %.3f, %.3f, %.3f, ...]\n', t_opt(1:min(4,end)));
    fprintf('     位置变化: %.4f λ\n', position_change);
    
    if position_change < 1e-6
        fprintf('     ⚠️ 位置几乎无变化，可能优化未生效\n');
    else
        fprintf('     ✅ 位置发生显著变化\n');
    end
    
    % 检查2: 约束是否满足？
    fprintf('   约束验证:\n');
    
    % 边界约束
    boundary_ok = (t_opt(1) >= t_start - 1e-6) && (t_opt(end) <= t_end + 1e-6);
    if boundary_ok
        fprintf('     边界约束: ✅ (%.3f ≤ t ≤ %.3f)\n', t_start, t_end);
    else
        fprintf('     边界约束: ❌ (%.3f ≤ t ≤ %.3f)\n', t_start, t_end);
    end
    
    % 排序约束
    sorting_ok = all(diff(t_opt) >= -1e-6);
    if sorting_ok
        fprintf('     排序约束: ✅ (t1 ≤ t2 ≤ ... ≤ tNa)\n');
    else
        fprintf('     排序约束: ❌ (t1 ≤ t2 ≤ ... ≤ tNa)\n');
    end
    
    % 最小间距约束
    min_spacing = min(diff(t_opt));
    spacing_ok = min_spacing >= d_min - 1e-6;
    if spacing_ok
        fprintf('     间距约束: ✅ (%.3f λ ≥ %.3f λ)\n', min_spacing, d_min);
    else
        fprintf('     间距约束: ❌ (%.3f λ ≥ %.3f λ)\n', min_spacing, d_min);
    end
    
    %% 8. 计算FAS优化后性能
    fprintf('8. 计算FAS优化后性能...\n');
    
    % 使用优化后的位置重新计算信道
    t_FAS_cell = cell(M, 1);
    for m = 1:M
        if m == m_test
            t_FAS_cell{m} = t_opt;
        else
            t_FAS_cell{m} = t_ULA;  % 其他GBS保持ULA
        end
    end
    
    h_optimized = cell(M, K, test_N);
    for m = 1:M
        for k = 1:K
            for n = 1:test_N
                h_optimized{m,k,n} = get_channel(m, k, n, u, q_fixed, H, kappa, t_FAS_cell{m}, Na);
            end
        end
    end
    
    % 计算FAS和速率（使用相同的波束矩阵）
    sum_rate_FAS = 0;
    for n = 1:test_N
        for k = 1:K
            m_serv = find(alpha_fixed(:,k,n) == 1, 1);
            if ~isempty(m_serv)
                h_mk = h_optimized{m_serv,k,n};
                signal_power = real(h_mk' * W_test{m_serv,k,n} * h_mk);
                
                % 计算干扰
                interference = 0;
                for l = 1:M
                    for i = 1:K
                        if ~(l==m_serv && i==k) && alpha_fixed(l,i,n) == 1
                            h_lk = h_optimized{l,k,n};
                            interference = interference + real(h_lk' * W_test{l,i,n} * h_lk);
                        end
                    end
                    % 感知信号干扰
                    h_lk = h_optimized{l,k,n};
                    interference = interference + real(h_lk' * R_test{l,1,n} * h_lk);
                end
                
                SINR = signal_power / (interference + sigma2);
                sum_rate_FAS = sum_rate_FAS + log2(1 + max(SINR, 1e-12));
            end
        end
    end
    
    % 计算FAS感知功率
    sensing_power_FAS = compute_sensing_power(alpha_fixed, W_test, R_test, ...
        u, v, M, Q, test_N, Na, kappa, t_FAS_cell, H_sense);
    
    fprintf('   FAS优化后性能:\n');
    fprintf('     和速率: %.4f bps/Hz\n', sum_rate_FAS);
    fprintf('     感知功率: %.4e W (%.2f dBW)\n', sensing_power_FAS, 10*log10(sensing_power_FAS));
    if sensing_power_FAS >= Gamma_test
        fprintf('     感知约束: 满足\n');
    else
        fprintf('     感知约束: 不满足\n');
    end
    
    %% 9. 性能对比分析
    fprintf('9. 性能对比分析...\n');
    
    rate_improvement = sum_rate_FAS - sum_rate_ULA;
    rate_improvement_percent = (rate_improvement / sum_rate_ULA) * 100;
    sensing_improvement = sensing_power_FAS / sensing_power_ULA;
    
    fprintf('   性能提升分析:\n');
    fprintf('     和速率提升: %.4f bps/Hz (%.2f%%)\n', rate_improvement, rate_improvement_percent);
    fprintf('     感知功率比值: %.4f (%.2f dB)\n', sensing_improvement, 10*log10(sensing_improvement));
    
    %% 10. 测试结果总结
    fprintf('\n================================\n');
    fprintf('🎯 FAS位置优化器测试结果\n');
    fprintf('================================\n');
    
    % 功能性检查
    if boundary_ok && sorting_ok && spacing_ok
        fprintf('✅ 约束满足: 所有FAS约束都满足\n');
    else
        fprintf('❌ 约束违反: 存在约束违反\n');
    end
    
    if position_change > 1e-6
        fprintf('✅ 位置优化: 天线位置发生显著变化\n');
    else
        fprintf('⚠️ 位置优化: 位置变化很小\n');
    end
    
    % 性能检查
    if rate_improvement > 1e-6
        fprintf('✅ 通信性能: 和速率提升 %.2f%%\n', rate_improvement_percent);
    elseif abs(rate_improvement) < 1e-6
        fprintf('➖ 通信性能: 和速率基本不变\n');
    else
        fprintf('❌ 通信性能: 和速率下降 %.2f%%\n', abs(rate_improvement_percent));
    end
    
    if sensing_improvement > 1.01
        fprintf('✅ 感知性能: 感知功率提升 %.2f dB\n', 10*log10(sensing_improvement));
    elseif sensing_improvement > 0.99
        fprintf('➖ 感知性能: 感知功率基本不变\n');
    else
        fprintf('❌ 感知性能: 感知功率下降 %.2f dB\n', -10*log10(sensing_improvement));
    end
    
    % 总体评估
    overall_success = boundary_ok && sorting_ok && spacing_ok && ...
                     (rate_improvement >= -1e-6) && (sensing_improvement >= 0.99);
    
    if overall_success
        fprintf('\n🚀 测试结论: FAS位置优化器工作正常！\n');
        fprintf('   可以安全集成到完整AO算法中\n');
    else
        fprintf('\n⚠️ 测试结论: FAS位置优化器需要调试\n');
        fprintf('   建议检查优化算法实现\n');
    end
    
catch ME
    fprintf('\n❌ 测试过程中出现错误:\n');
    fprintf('错误信息: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('错误位置: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    end
    fprintf('\n请检查相关代码。\n');
end
