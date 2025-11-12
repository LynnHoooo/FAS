%% 运行完整AO算法直到收敛的测试脚本
%% ============================================================================
%  测试目标: 运行完整的AO算法主循环直到收敛，保存每次迭代的详细数据
%  数据用途: 后续分析AO收敛过程和绘制性能演化图
%% ============================================================================

clear; clc; close all;

fprintf('🚀 完整AO算法收敛测试\n');
fprintf('================================\n\n');

%% 0. 设置路径
setup_paths;

%% 1. 初始化系统参数
fprintf('步骤1: 初始化系统参数...\n');
initial; % 运行完整的初始化

% 将所有参数打包到结构体p中
p = struct();

% 基础系统参数
p.M = M; p.K = K; p.N = N; p.Q = Q; p.Na = Na; p.B = B;
p.Pmax = Pmax; p.sigma2 = sigma2; p.Gamma = Gamma;
p.kappa = kappa; p.d = d; p.H_sense = H_sense;

% 天线位置相关参数
p.t_init = t_init; p.t_start = t_start; p.t_end = t_end; p.d_min = d_min;

% 几何参数
p.u = u; p.v = v; p.H = H;
p.dt = dt; p.Vmax = Vmax; p.Dmin = Dmin;

% 初始化结果
p.q_traj = q_traj;
p.alpha_init = alpha_init;
p.W_init = W_init;
p.R_init = R_init;
p.h_mkn = h_mkn;

fprintf('✅ 系统参数初始化完成\n');
fprintf('  系统规模: %d GBS, %d UAV, %d 时隙, %d 感知点\n', M, K, N, Q);
fprintf('  初始轨迹: UAV1 (%.0f,%.0f)→(%.0f,%.0f), UAV2 (%.0f,%.0f)→(%.0f,%.0f)\n', ...
    qI(1,1), qI(1,2), qF(1,1), qF(1,2), ...
    qI(2,1), qI(2,2), qF(2,1), qF(2,2));

%% 2. 运行完整AO算法
fprintf('\n步骤2: 运行完整AO算法...\n');
fprintf('────────────────────────────────────────\n');

% 记录开始时间
start_time = tic;

% 调用主AO算法
[final_sum_rate, final_min_sensing] = main_AO_algorithm(p);

% 记录结束时间
total_time = toc(start_time);

fprintf('────────────────────────────────────────\n');
fprintf('⏱️ AO算法总运行时间: %.2f 秒\n', total_time);

%% 3. 加载和分析结果
fprintf('\n步骤3: 分析AO收敛结果...\n');
load('data/ao_convergence_results.mat');

fprintf('📊 收敛分析:\n');
fprintf('  迭代次数: %d 次\n', ao_history.performance.iterations);
if ao_history.performance.converged
    fprintf('  收敛状态: ✅ 已收敛\n');
else
    fprintf('  收敛状态: ⚠️ 达到最大迭代次数\n');
end

% 性能改善统计
rate_improvement = final_sum_rate - ao_history.performance.initial_sum_rate;
sensing_improvement = final_min_sensing - ao_history.performance.initial_min_sensing;

fprintf('  和速率: %.4f → %.4f bps/Hz (%+.4f, %.2f%%)\n', ...
    ao_history.performance.initial_sum_rate, final_sum_rate, ...
    rate_improvement, (rate_improvement / ao_history.performance.initial_sum_rate) * 100);

fprintf('  感知功率: %.4e → %.4e W (%+.4e)\n', ...
    ao_history.performance.initial_min_sensing, final_min_sensing, sensing_improvement);

% 每次迭代的改善
if ao_history.performance.iterations > 1
    fprintf('  每次迭代平均改善: %.4f bps/Hz\n', ...
        rate_improvement / ao_history.performance.iterations);
end

%% 4. 简单可视化预览
fprintf('\n步骤4: 生成收敛曲线预览...\n');

figure('Position', [100, 100, 1200, 400]);

% 子图1: 和速率收敛
subplot(1, 3, 1);
iterations = 1:ao_history.performance.iterations;
plot(0, ao_history.performance.initial_sum_rate, 'ro-', 'MarkerSize', 8, 'LineWidth', 2);
hold on;
plot(iterations, ao_history.performance.sum_rates, 'b.-', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('AO 迭代次数');
ylabel('和速率 (bps/Hz)');
title('通信性能收敛');
grid on;
legend('初始', '优化过程', 'Location', 'best');

% 子图2: 感知功率收敛
subplot(1, 3, 2);
semilogy(0, ao_history.performance.initial_min_sensing, 'ro-', 'MarkerSize', 8, 'LineWidth', 2);
hold on;
semilogy(iterations, ao_history.performance.min_sensing_powers, 'g.-', 'MarkerSize', 10, 'LineWidth', 2);
yline(Gamma, 'r--', 'LineWidth', 2, 'DisplayName', sprintf('Gamma阈值 (%.0e W)', Gamma));
xlabel('AO 迭代次数');
ylabel('最小感知功率 (W)');
title('感知性能收敛');
grid on;
legend('初始', '优化过程', 'Gamma阈值', 'Location', 'best');

% 子图3: 信任域变化
subplot(1, 3, 3);
plot(iterations, ao_history.trust_regions, 'm.-', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('AO 迭代次数');
ylabel('信任域半径 (m)');
title('轨迹优化信任域');
grid on;

sgtitle('AO算法收敛过程预览', 'FontSize', 14, 'FontWeight', 'bold');

%% 5. 保存扩展分析数据
fprintf('\n步骤5: 保存扩展分析数据...\n');

% 计算轨迹变化统计
trajectory_changes = zeros(ao_history.performance.iterations, 1);
for iter = 1:ao_history.performance.iterations
    if iter == 1
        trajectory_changes(iter) = norm(ao_history.trajectories{iter}(:) - p.q_traj(:));
    else
        trajectory_changes(iter) = norm(ao_history.trajectories{iter}(:) - ao_history.trajectories{iter-1}(:));
    end
end

% 扩展统计数据
extended_stats = struct();
extended_stats.trajectory_changes = trajectory_changes;
extended_stats.total_runtime = total_time;
extended_stats.avg_time_per_iteration = total_time / ao_history.performance.iterations;
extended_stats.final_trajectory_change = norm(ao_history.trajectories{end}(:) - p.q_traj(:));

% 保存到同一文件
save('data/ao_convergence_results.mat', 'ao_history', 'final_sum_rate', 'final_min_sensing', 'extended_stats');

fprintf('✅ 扩展数据已更新到: data/ao_convergence_results.mat\n');

%% 6. 总结
fprintf('\n🎯 AO收敛测试完成！\n');
fprintf('═══════════════════════════════════════════════\n');
fprintf('✅ 主要成果:\n');
fprintf('   • AO算法成功运行 %d 次迭代\n', ao_history.performance.iterations);
if ao_history.performance.converged
    fprintf('   • 收敛至最优解\n');
else
    fprintf('   • 达到最大迭代次数限制\n');
end
fprintf('   • 和速率提升: %.2f%%\n', (rate_improvement / ao_history.performance.initial_sum_rate) * 100);
if final_min_sensing >= Gamma
    fprintf('   • 感知约束: ✅ 满足\n');
else
    fprintf('   • 感知约束: ❌ 不满足\n');
end
fprintf('   • 总运行时间: %.2f 秒\n', total_time);
fprintf('\n📁 数据输出:\n');
fprintf('   • 详细AO历史: data/ao_convergence_results.mat\n');
fprintf('   • 包含每次迭代的轨迹、关联、波束数据\n');
fprintf('   • 可用于后续收敛分析和性能可视化\n');
fprintf('\n💡 后续建议:\n');
fprintf('   • 使用 plot_ao_convergence_detailed.m 生成详细收敛图\n');
fprintf('   • 使用 analyze_ao_performance.m 进行深入性能分析\n');
fprintf('═══════════════════════════════════════════════\n\n');
