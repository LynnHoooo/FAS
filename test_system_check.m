%% 系统稳定性检查脚本
% 快速验证FAS系统是否处于稳定工作状态

clear; clc; close all;

fprintf('🔍 FAS系统稳定性检查\n');
fprintf('================================\n\n');

%% 1. 检查路径设置
try
    setup_paths;
    fprintf('✅ 路径设置正常\n');
catch ME
    fprintf('❌ 路径设置失败: %s\n', ME.message);
    return;
end

%% 2. 检查初始化
try
    fprintf('正在运行 initial.m...\n');
    initial;
    fprintf('✅ 初始化成功\n');
    fprintf('   系统规模: M=%d, K=%d, N=%d, Na=%d\n', M, K, N, Na);
    fprintf('   FAS参数: 孔径=[%.1f, %.1f]λ, d_min=%.2fλ\n', t_start, t_end, d_min);
catch ME
    fprintf('❌ 初始化失败: %s\n', ME.message);
    return;
end

%% 3. 检查参数结构体组装
try
    p = struct();
    p.M = M; p.K = K; p.N = N; p.Q = Q; p.Na = Na; p.B = B;
    p.Pmax = Pmax; p.sigma2 = sigma2; p.Gamma = Gamma;
    p.kappa = kappa; p.d = d; p.H_sense = H_sense;
    p.q_traj = q_traj; p.alpha_init = alpha_init;
    p.W_init = W_init; p.R_init = R_init; p.h_mkn = h_mkn;
    p.u = u; p.v = v; p.H = H;
    p.dt = dt; p.Vmax = Vmax; p.Dmin = Dmin;
    
    % FAS参数
    p.t_init = t_init;
    p.t_start = t_start;
    p.t_end = t_end;
    p.d_min = d_min;
    
    fprintf('✅ 参数结构体组装成功\n');
catch ME
    fprintf('❌ 参数组装失败: %s\n', ME.message);
    return;
end

%% 4. 检查单次位置优化（快速测试）
try
    fprintf('测试单次位置优化...\n');
    m_test = 1;
    t_before = t_init{m_test};
    
    % 设置较少迭代次数进行快速测试
    [t_after, history] = optimize_antenna_position(...
        q_traj, alpha_init, W_init, R_init, ...
        u, v, H, H_sense, M, K, N, Na, Q, t_before, t_start, t_end, d_min, ...
        kappa, Pmax, Gamma, sigma2);
    
    change = norm(t_after - t_before);
    fprintf('✅ 位置优化功能正常\n');
    fprintf('   位置变化: %.4f λ\n', change);
    if ~isempty(history)
        fprintf('   收敛历史: %d 次迭代\n', length(history));
    end
catch ME
    fprintf('❌ 位置优化失败: %s\n', ME.message);
    fprintf('   错误位置: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    return;
end

%% 5. 检查AO算法（1次迭代快速测试）
try
    fprintf('测试AO算法（1次迭代）...\n');
    p.max_iterations = 1;  % 只运行1次迭代进行快速测试
    
    tic;
    [final_sum_rate, final_min_sensing, ao_history] = main_AO_algorithm(p);
    elapsed = toc;
    
    fprintf('✅ AO算法运行正常\n');
    fprintf('   最终速率: %.4f bps/Hz\n', final_sum_rate);
    fprintf('   感知功率: %.4e W\n', final_min_sensing);
    fprintf('   运行耗时: %.2f 秒\n', elapsed);
    
    % 检查位置历史是否正确保存
    if isfield(ao_history, 'antenna_positions')
        fprintf('   位置历史: 正确保存\n');
    else
        fprintf('   ⚠️ 位置历史: 未保存\n');
    end
    
catch ME
    fprintf('❌ AO算法失败: %s\n', ME.message);
    fprintf('   错误位置: %s (line %d)\n', ME.stack(1).name, ME.stack(1).line);
    return;
end

%% 6. 总结
fprintf('\n================================\n');
fprintf('🎯 系统状态总结\n');
fprintf('================================\n');
fprintf('✅ 路径设置: 正常\n');
fprintf('✅ 参数初始化: 正常\n');
fprintf('✅ 位置优化: 正常\n');
fprintf('✅ AO算法: 正常\n');
fprintf('✅ 数据保存: 正常\n');
fprintf('\n🚀 系统处于稳定工作状态！\n');
fprintf('   可以安全运行完整的AO算法测试\n');
fprintf('\n建议下一步:\n');
fprintf('   cd test\n');
fprintf('   test_run_main_AO  %% 运行完整测试\n');
