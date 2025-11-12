%% 处理已有的AO算法结果
% 该脚本处理已经完成的AO算法结果，无需重新运行算法
% 版本: 1.0

clear; clc; close all;

% 设置路径
setup_paths;

fprintf('📊 处理已有的AO算法结果...\n');

%% 1. 加载已保存的AO结果
data_file = 'data/ao_convergence_results.mat';
if ~exist(data_file, 'file')
    error('未找到AO算法结果文件: %s', data_file);
end

fprintf('📁 加载数据: %s\n', data_file);
load(data_file);

% 加载系统参数
initial;  % 获取B, Gamma等参数

%% 2. 提取并整理数据
fprintf('📋 整理实验数据...\n');

% 创建实验结果结构体
experiment_results = struct();

% 实验元数据
experiment_results.meta = struct();
experiment_results.meta.experiment_name = sprintf('AO_Processed_%s', datestr(now, 'yyyymmdd_HHMMSS'));
experiment_results.meta.timestamp = datestr(now);

% 系统配置
experiment_results.config = struct();
experiment_results.config.M = M;
experiment_results.config.K = K;
experiment_results.config.N = N;
experiment_results.config.Q = Q;
experiment_results.config.Na = Na;
experiment_results.config.B = B;
experiment_results.config.Pmax = Pmax;
experiment_results.config.Gamma = Gamma;

% AO收敛历史数据
experiment_results.ao_history = ao_history;

% 提取性能数据
iterations = 1:ao_history.performance.iterations;
sum_rates = ao_history.performance.sum_rates;
sum_rates_mbps = sum_rates * B / 1e6;
min_sensing_powers = ao_history.performance.min_sensing_powers;

% 最终结果
experiment_results.final_results = struct();
experiment_results.final_results.sum_rate = ao_history.performance.final_sum_rate;
experiment_results.final_results.sum_rate_mbps = ao_history.performance.final_sum_rate * B / 1e6;
experiment_results.final_results.min_sensing = ao_history.performance.final_min_sensing;
experiment_results.final_results.converged = ao_history.performance.converged;
experiment_results.final_results.total_iterations = ao_history.performance.iterations;

% 初始结果
experiment_results.initial_results = struct();
experiment_results.initial_results.sum_rate = ao_history.performance.initial_sum_rate;
experiment_results.initial_results.sum_rate_mbps = ao_history.performance.initial_sum_rate * B / 1e6;
experiment_results.initial_results.min_sensing = ao_history.performance.initial_min_sensing;

% 性能改善
performance_improvement = struct();
performance_improvement.rate_improvement_bps_hz = experiment_results.final_results.sum_rate - experiment_results.initial_results.sum_rate;
performance_improvement.rate_improvement_mbps = performance_improvement.rate_improvement_bps_hz * B / 1e6;
performance_improvement.rate_improvement_percent = (performance_improvement.rate_improvement_bps_hz / experiment_results.initial_results.sum_rate) * 100;
performance_improvement.sensing_improvement = experiment_results.final_results.min_sensing - experiment_results.initial_results.min_sensing;
experiment_results.performance_improvement = performance_improvement;

%% 3. 保存完整的实验数据
processed_filename = sprintf('data/ao_processed_results_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(processed_filename, 'experiment_results', '-v7.3');

% 保存CSV数据
csv_filename = sprintf('data/ao_convergence_data_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'));
% 确保所有数组长度一致
num_iters = length(iterations);
trust_regions_data = ao_history.trust_regions(1:num_iters);
convergence_table = table(iterations', sum_rates(iterations)', sum_rates_mbps(iterations)', ...
    min_sensing_powers(iterations)', trust_regions_data', ...
    'VariableNames', {'Iteration', 'SumRate_bps_Hz', 'SumRate_Mbps', 'MinSensingPower_W', 'TrustRegion_m'});
writetable(convergence_table, csv_filename);

fprintf('✅ 处理后的数据已保存:\n');
fprintf('  📁 MATLAB数据: %s\n', processed_filename);
fprintf('  📊 CSV数据: %s\n', csv_filename);

%% 4. 生成实验报告
fprintf('\n📋 实验结果摘要\n');
fprintf('=====================================================\n');
if experiment_results.final_results.converged
    fprintf('🎯 收敛状态: ✅ 已收敛\n');
else
    fprintf('🎯 收敛状态: ⚠️ 未收敛\n');
end
fprintf('🔄 总迭代次数: %d\n', experiment_results.final_results.total_iterations);

fprintf('\n📊 性能指标:\n');
fprintf('  初始和速率: %.4f bps/Hz (%.2f Mbps)\n', ...
    experiment_results.initial_results.sum_rate, experiment_results.initial_results.sum_rate_mbps);
fprintf('  最终和速率: %.4f bps/Hz (%.2f Mbps)\n', ...
    experiment_results.final_results.sum_rate, experiment_results.final_results.sum_rate_mbps);
fprintf('  性能提升: %.4f bps/Hz (%.2f Mbps, %.2f%%)\n', ...
    performance_improvement.rate_improvement_bps_hz, ...
    performance_improvement.rate_improvement_mbps, ...
    performance_improvement.rate_improvement_percent);

fprintf('\n🛡️ 感知性能:\n');
fprintf('  初始最小感知功率: %.4e W (%.2f dBW)\n', ...
    experiment_results.initial_results.min_sensing, 10*log10(experiment_results.initial_results.min_sensing));
fprintf('  最终最小感知功率: %.4e W (%.2f dBW)\n', ...
    experiment_results.final_results.min_sensing, 10*log10(experiment_results.final_results.min_sensing));
fprintf('  感知阈值 Gamma: %.4e W (%.1f dBW)\n', Gamma, 10*log10(Gamma));
if experiment_results.final_results.min_sensing >= Gamma
    fprintf('  约束满足: ✅ 是\n');
else
    fprintf('  约束满足: ❌ 否\n');
end

%% 5. 生成收敛图
fprintf('\n📈 生成收敛分析图...\n');
figure('Name', 'AO Convergence Analysis - Processed Results', 'Position', [100, 100, 1200, 800]);

% 获取绘图数据（确保数组长度一致）
plot_iterations = iterations;
plot_sum_rates_mbps = sum_rates_mbps(iterations);
plot_min_sensing_powers = min_sensing_powers(iterations);
plot_sum_rates = sum_rates(iterations);

% 子图1: 和速率收敛
subplot(2, 2, 1);
plot(plot_iterations, plot_sum_rates_mbps, 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
xlabel('AO 迭代次数');
ylabel('和速率 (Mbps)');
title('通信性能收敛');
xlim([1, max(plot_iterations)]);

% 子图2: 感知功率
subplot(2, 2, 2);
semilogy(plot_iterations, plot_min_sensing_powers, 'r-s', 'LineWidth', 2, 'MarkerSize', 6);
hold on;
semilogy([1, max(plot_iterations)], [Gamma, Gamma], 'k--', 'LineWidth', 2);
grid on;
xlabel('AO 迭代次数');
ylabel('最小感知功率 (W)');
title('感知性能收敛');
legend('最小感知功率', 'Gamma阈值', 'Location', 'best');
xlim([1, max(plot_iterations)]);

% 子图3: 频谱效率
subplot(2, 2, 3);
plot(plot_iterations, plot_sum_rates, 'g-^', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
xlabel('AO 迭代次数');
ylabel('频谱效率 (bps/Hz)');
title('频谱效率收敛');
xlim([1, max(plot_iterations)]);

% 子图4: 性能改善率
if length(plot_iterations) > 1
    improvement_rates = zeros(1, length(plot_iterations)-1);
    for i = 2:length(plot_iterations)
        improvement_rates(i-1) = (plot_sum_rates(i) - plot_sum_rates(i-1)) / plot_sum_rates(i-1) * 100;
    end
    subplot(2, 2, 4);
    bar(2:length(plot_iterations), improvement_rates, 'FaceColor', [0.2, 0.6, 0.8]);
    grid on;
    xlabel('AO 迭代次数');
    ylabel('相对改善率 (%)');
    title('逐次迭代改善');
    xlim([1.5, max(plot_iterations)+0.5]);
end

sgtitle(sprintf('AO收敛分析 - %s', experiment_results.meta.experiment_name), 'FontSize', 14, 'FontWeight', 'bold');

% 保存图片
fig_filename = sprintf('data/ao_processed_plot_%s.png', datestr(now, 'yyyymmdd_HHMMSS'));
saveas(gcf, fig_filename);
fprintf('📊 收敛图已保存: %s\n', fig_filename);

fprintf('\n🎉 AO结果处理完成！\n');
fprintf('现在您可以运行: plot_ao_convergence_detailed(''%s'') 生成详细分析图\n', processed_filename);
