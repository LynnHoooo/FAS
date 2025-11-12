    %% AO收敛分析详细绘图脚本
% 该脚本读取AO收敛实验数据并生成详细的分析图表
% 版本: 1.0
% 输入: 从data/目录读取最新的实验数据
% 输出: 多种收敛分析图表

function plot_ao_convergence_detailed(data_file)

% 设置路径
setup_paths;

if nargin < 1
    % 如果没有指定文件，寻找最新的实验数据
    data_dir = 'data/';
    % 尝试多种文件名模式
    patterns = {
        'ao_convergence_experiment_*.mat',
        'ao_processed_results_*.mat',
        'ao_convergence_results.mat'
    };

    files = [];
    for p = 1:length(patterns)
        temp_files = dir(fullfile(data_dir, patterns{p}));
        if ~isempty(temp_files)
            files = [files; temp_files];
        end
    end

    if isempty(files)
        error('未找到AO收敛实验数据文件！请先运行 test_ao_convergence_experiment.m');
    end
    % 按时间排序，取最新的
    [~, idx] = max([files.datenum]);
    data_file = fullfile(data_dir, files(idx).name);
    fprintf('📁 自动选择最新数据文件: %s\n', files(idx).name);
end

% 加载实验数据
fprintf('📊 加载实验数据...\n');
load(data_file, 'experiment_results');
fprintf('✅ 数据加载完成\n');

% 提取数据
ao_data = experiment_results.ao_history;
config = experiment_results.config;
total_iters = ao_data.performance.iterations;
iterations = 1:total_iters;
sum_rates = ao_data.performance.sum_rates(iterations);
% sum_rates已经是每个时隙的平均和速率（mean across N slots）
avg_sum_rates = sum_rates;  % 直接使用，不再除以N
avg_sum_rates_mbps = avg_sum_rates * config.B / 1e6;
sum_rates_mbps = sum_rates * config.B / 1e6;
min_sensing_powers = ao_data.performance.min_sensing_powers(iterations);
trust_regions = ao_data.trust_regions(iterations);

fprintf('📈 生成详细收敛分析图表...\n');

%% 主要收敛分析图
figure('Name', 'AO Algorithm Convergence Analysis', 'Position', [50, 50, 1400, 1000]);

% 配置颜色
colors = {
    [0.0, 0.4470, 0.7410],  % 蓝色
    [0.8500, 0.3250, 0.0980],  % 橙色
    [0.9290, 0.6940, 0.1250],  % 黄色
    [0.4940, 0.1840, 0.5560],  % 紫色
    [0.4660, 0.6740, 0.1880],  % 绿色
    [0.3010, 0.7450, 0.9330]   % 青色
};

% 子图1: 通信性能收敛 (bps/Hz) - 平均和速率
subplot(3, 3, 1);
plot(iterations, avg_sum_rates, 'o-', 'Color', colors{1}, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', colors{1});
grid on; grid minor;
xlabel('AO迭代次数', 'FontSize', 12);
ylabel('平均和速率 (bps/Hz)', 'FontSize', 12);
title('频谱效率收敛', 'FontSize', 14, 'FontWeight', 'bold');
xlim([1, max(iterations)]);
set(gca, 'FontSize', 11);

% 子图2: 通信性能收敛 (Mbps) - 平均和速率
subplot(3, 3, 2);
plot(iterations, avg_sum_rates_mbps, 's-', 'Color', colors{2}, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', colors{2});
grid on; grid minor;
xlabel('AO迭代次数', 'FontSize', 12);
ylabel('平均和速率 (Mbps)', 'FontSize', 12);
title('平均速率收敛', 'FontSize', 14, 'FontWeight', 'bold');
xlim([1, max(iterations)]);
set(gca, 'FontSize', 11);

% 子图3: 感知功率收敛
subplot(3, 3, 3);
semilogy(iterations, min_sensing_powers, '^-', 'Color', colors{3}, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', colors{3});
hold on;
semilogy([1, max(iterations)], [config.Gamma, config.Gamma], '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 2);
grid on; grid minor;
xlabel('AO迭代次数', 'FontSize', 12);
ylabel('最小感知功率 (W)', 'FontSize', 12);
title('感知性能收敛', 'FontSize', 14, 'FontWeight', 'bold');
legend('最小感知功率', 'Gamma阈值', 'Location', 'best', 'FontSize', 10);
xlim([1, max(iterations)]);
set(gca, 'FontSize', 11);

% 子图4: 信任域演化
subplot(3, 3, 4);
plot(iterations, trust_regions, 'd-', 'Color', colors{4}, 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', colors{4});
grid on; grid minor;
xlabel('AO迭代次数', 'FontSize', 12);
ylabel('信任域半径 (m)', 'FontSize', 12);
title('信任域调整策略', 'FontSize', 14, 'FontWeight', 'bold');
xlim([1, max(iterations)]);
set(gca, 'FontSize', 11);

% 子图5: 逐次改善率
if length(iterations) > 1
    improvement_rates = zeros(1, length(iterations)-1);
    for i = 2:length(iterations)
        improvement_rates(i-1) = (avg_sum_rates(i) - avg_sum_rates(i-1)) / avg_sum_rates(i-1) * 100;
    end
    subplot(3, 3, 5);
    bar(2:length(iterations), improvement_rates, 'FaceColor', colors{5}, 'EdgeColor', colors{5}, 'LineWidth', 1);
    grid on; grid minor;
    xlabel('AO迭代次数', 'FontSize', 12);
    ylabel('相对改善率 (%)', 'FontSize', 12);
    title('逐次性能提升', 'FontSize', 14, 'FontWeight', 'bold');
    xlim([1.5, max(iterations)+0.5]);
    set(gca, 'FontSize', 11);
end

% 子图6: 累积改善
cumulative_improvement = (avg_sum_rates - avg_sum_rates(1)) ./ avg_sum_rates(1) * 100;
subplot(3, 3, 6);
area(iterations, cumulative_improvement, 'FaceColor', colors{6}, 'FaceAlpha', 0.7, 'EdgeColor', colors{6}, 'LineWidth', 2);
grid on; grid minor;
xlabel('AO迭代次数', 'FontSize', 12);
ylabel('累积改善率 (%)', 'FontSize', 12);
title('累积性能提升', 'FontSize', 14, 'FontWeight', 'bold');
xlim([1, max(iterations)]);
set(gca, 'FontSize', 11);

% 子图7: 性能 vs 感知权衡
subplot(3, 3, 7);
scatter(min_sensing_powers, avg_sum_rates_mbps, 80, iterations, 'filled', 's');
cb = colorbar;
ylabel(cb, 'AO迭代次数', 'FontSize', 11);
grid on; grid minor;
xlabel('最小感知功率 (W)', 'FontSize', 12);
ylabel('平均和速率 (Mbps)', 'FontSize', 12);
title('通信-感知权衡轨迹', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XScale', 'log', 'FontSize', 11);

% 子图8: 收敛速度分析
if length(iterations) > 2
    subplot(3, 3, 8);
    convergence_metric = abs(diff(avg_sum_rates));
    semilogy(2:length(iterations), convergence_metric, 'o-', 'Color', [0.6, 0.2, 0.8], 'LineWidth', 2, 'MarkerSize', 6);
    grid on; grid minor;
    xlabel('AO迭代次数', 'FontSize', 12);
    ylabel('平均速率变化量 (bps/Hz)', 'FontSize', 12);
    title('收敛速度', 'FontSize', 14, 'FontWeight', 'bold');
    xlim([2, max(iterations)]);
    set(gca, 'FontSize', 11);
end

% 子图9: 性能摘要
subplot(3, 3, 9);
axis off;

% 修复三元运算符语法（MATLAB不支持 ? : 语法）
if ao_data.performance.converged
    converged_status = '已收敛';
else
    converged_status = '未收敛';
end

if experiment_results.final_results.min_sensing >= config.Gamma
    sensing_status = '满足';
else
    sensing_status = '不满足';
end

summary_text = {
    '\bf性能摘要';
    sprintf('总迭代次数: %d', total_iters);
    sprintf('收敛状态: %s', converged_status);
    '';
    sprintf('初始速率: %.2f Mbps', experiment_results.initial_results.sum_rate_mbps);
    sprintf('最终速率: %.2f Mbps', experiment_results.final_results.sum_rate_mbps);
    sprintf('性能提升: %.2f%%', experiment_results.performance_improvement.rate_improvement_percent);
    '';
    sprintf('感知约束: %s', sensing_status);
    sprintf('系统带宽: %.0f MHz', config.B/1e6);
    sprintf('最大功率: %.0f W', config.Pmax);
};
text(0.05, 0.95, summary_text, 'FontSize', 11, 'VerticalAlignment', 'top', 'Units', 'normalized');

% 设置总标题
sgtitle(sprintf('AO算法收敛分析 - %s', experiment_results.meta.experiment_name), ...
    'FontSize', 16, 'FontWeight', 'bold');

%% 保存图片
fig_filename = sprintf('data/ao_detailed_analysis_%s.png', datestr(now, 'yyyymmdd_HHMMSS'));
saveas(gcf, fig_filename, 'png');
fprintf('📊 详细分析图已保存: %s\n', fig_filename);

%% 生成收敛曲线对比图
figure('Name', 'AO Convergence Curves', 'Position', [100, 100, 1200, 600]);

% 左图: 通信性能
subplot(1, 2, 1);
yyaxis left;
plot(iterations, avg_sum_rates, 'o-', 'LineWidth', 3, 'MarkerSize', 8, 'Color', colors{1}, 'MarkerFaceColor', colors{1});
ylabel('平均频谱效率 (bps/Hz)', 'FontSize', 14, 'Color', colors{1});
yyaxis right;
plot(iterations, avg_sum_rates_mbps, 's-', 'LineWidth', 3, 'MarkerSize', 8, 'Color', colors{2}, 'MarkerFaceColor', colors{2});
ylabel('平均速率 (Mbps)', 'FontSize', 14, 'Color', colors{2});
xlabel('AO迭代次数', 'FontSize', 14);
title('通信性能收敛', 'FontSize', 16, 'FontWeight', 'bold');
grid on; grid minor;
xlim([1, max(iterations)]);
set(gca, 'FontSize', 12);

% 右图: 感知性能
subplot(1, 2, 2);
semilogy(iterations, min_sensing_powers, '^-', 'LineWidth', 3, 'MarkerSize', 8, 'Color', colors{3}, 'MarkerFaceColor', colors{3});
hold on;
semilogy([1, max(iterations)], [config.Gamma, config.Gamma], '--', 'LineWidth', 3, 'Color', [0.5, 0.5, 0.5]);
xlabel('AO迭代次数', 'FontSize', 14);
ylabel('感知功率 (W)', 'FontSize', 14);
title('感知性能收敛', 'FontSize', 16, 'FontWeight', 'bold');
legend('最小感知功率', 'Gamma阈值', 'Location', 'best', 'FontSize', 12);
grid on; grid minor;
xlim([1, max(iterations)]);
set(gca, 'FontSize', 12);

sgtitle('AO算法主要性能指标收敛曲线', 'FontSize', 18, 'FontWeight', 'bold');

% 保存收敛曲线图
curves_filename = sprintf('data/ao_convergence_curves_%s.png', datestr(now, 'yyyymmdd_HHMMSS'));
saveas(gcf, curves_filename, 'png');
fprintf('📈 收敛曲线图已保存: %s\n', curves_filename);

fprintf('🎉 AO收敛分析绘图完成！\n');
fprintf('生成的图片文件:\n');
fprintf('  - 详细分析图: %s\n', fig_filename);
fprintf('  - 收敛曲线图: %s\n', curves_filename);

end