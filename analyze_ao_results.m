%% AO算法结果分析脚本
% 加载并分析完整仿真的AO迭代数据
% 生成详细的收敛分析图和性能报告

clear; clc; close all;

fprintf('📊 AO算法结果分析\n');
fprintf('=====================================\n\n');

%% 1. 加载数据
fprintf('1. 加载仿真数据...\n');

% 检查数据文件是否存在
data_files = {
    'data/ao_convergence_results.mat',
    'data/simulation_config.mat', 
    'data/performance_analysis.mat'
};

missing_files = {};
for i = 1:length(data_files)
    if ~exist(data_files{i}, 'file')
        missing_files{end+1} = data_files{i};
    end
end

if ~isempty(missing_files)
    fprintf('❌ 缺少以下数据文件:\n');
    for i = 1:length(missing_files)
        fprintf('   - %s\n', missing_files{i});
    end
    fprintf('请先运行 run_full_simulation.m 生成数据\n');
    return;
end

% 加载数据
load('data/ao_convergence_results.mat');
load('data/simulation_config.mat');
load('data/performance_analysis.mat');

fprintf('   ✅ 数据加载完成\n');
fprintf('   - AO迭代次数: %d\n', ao_history.performance.iterations - 1);
fprintf('   - 系统配置: %dGBS × %dUAV × %d时隙\n', ...
    simulation_config.system_params.M, simulation_config.system_params.K, simulation_config.system_params.N);

%% 2. 详细收敛分析
fprintf('2. 进行详细收敛分析...\n');

sum_rates = ao_history.performance.sum_rates;
min_sensing_powers = ao_history.performance.min_sensing_powers;
iterations = length(sum_rates) - 1;

% 计算收敛指标
rate_improvements = diff(sum_rates);
sensing_improvements = diff(min_sensing_powers);

% 相对改善
relative_rate_improvements = rate_improvements ./ sum_rates(1:end-1) * 100;
relative_sensing_improvements = sensing_improvements ./ min_sensing_powers(1:end-1) * 100;

fprintf('   ✅ 收敛分析完成\n');

%% 3. 生成详细分析图
fprintf('3. 生成详细分析图...\n');

% 创建大图窗
figure('Position', [50, 50, 1600, 1200], 'Name', 'AO算法详细分析');

% 子图1：和速率收敛（线性尺度）
subplot(3, 3, 1);
plot(0:iterations, sum_rates, 'b-o', 'LineWidth', 2, 'MarkerSize', 4);
grid on;
title('和速率收敛 (线性尺度)');
xlabel('AO迭代次数');
ylabel('和速率 (bps/Hz)');
xlim([0, iterations]);

% 子图2：和速率收敛（对数尺度改善）
subplot(3, 3, 2);
if length(rate_improvements) > 0
    semilogy(1:length(rate_improvements), abs(rate_improvements), 'b-s', 'LineWidth', 2, 'MarkerSize', 4);
    grid on;
    title('和速率改善 (对数尺度)');
    xlabel('AO迭代次数');
    ylabel('|Δ和速率| (bps/Hz)');
    xlim([1, length(rate_improvements)]);
end

% 子图3：相对和速率改善
subplot(3, 3, 3);
if length(relative_rate_improvements) > 0
    plot(1:length(relative_rate_improvements), relative_rate_improvements, 'b-^', 'LineWidth', 2, 'MarkerSize', 4);
    grid on;
    title('相对和速率改善');
    xlabel('AO迭代次数');
    ylabel('相对改善 (%)');
    xlim([1, length(relative_rate_improvements)]);
end

% 子图4：感知功率收敛（对数尺度）
subplot(3, 3, 4);
semilogy(0:iterations, min_sensing_powers, 'r-o', 'LineWidth', 2, 'MarkerSize', 4);
grid on;
title('感知功率收敛 (对数尺度)');
xlabel('AO迭代次数');
ylabel('最小感知功率 (W)');
xlim([0, iterations]);

% 子图5：感知功率改善
subplot(3, 3, 5);
if length(sensing_improvements) > 0
    plot(1:length(sensing_improvements), sensing_improvements*1e3, 'r-s', 'LineWidth', 2, 'MarkerSize', 4);
    grid on;
    title('感知功率改善');
    xlabel('AO迭代次数');
    ylabel('Δ感知功率 (mW)');
    xlim([1, length(sensing_improvements)]);
end

% 子图6：相对感知功率改善
subplot(3, 3, 6);
if length(relative_sensing_improvements) > 0
    plot(1:length(relative_sensing_improvements), relative_sensing_improvements, 'r-^', 'LineWidth', 2, 'MarkerSize', 4);
    grid on;
    title('相对感知功率改善');
    xlabel('AO迭代次数');
    ylabel('相对改善 (%)');
    xlim([1, length(relative_sensing_improvements)]);
end

% 子图7：天线位置演化（如果有数据）
subplot(3, 3, 7);
if isfield(performance_analysis, 'antenna_optimization') && ...
   isfield(performance_analysis.antenna_optimization, 'position_changes_per_iteration')
    position_changes = performance_analysis.antenna_optimization.position_changes_per_iteration;
    if ~isempty(position_changes) && any(position_changes > 0)
        semilogy(1:length(position_changes), position_changes, 'g-o', 'LineWidth', 2, 'MarkerSize', 4);
        grid on;
        title('天线位置变化');
        xlabel('AO迭代次数');
        ylabel('位置变化 (λ)');
        xlim([1, length(position_changes)]);
    else
        text(0.5, 0.5, '天线位置无变化', 'HorizontalAlignment', 'center', 'Units', 'normalized');
        title('天线位置变化');
    end
else
    text(0.5, 0.5, '无天线位置数据', 'HorizontalAlignment', 'center', 'Units', 'normalized');
    title('天线位置变化');
end

% 子图8：收敛速度分析
subplot(3, 3, 8);
if length(rate_improvements) > 1
    convergence_rate = abs(rate_improvements(2:end) ./ rate_improvements(1:end-1));
    plot(2:length(rate_improvements), convergence_rate, 'k-d', 'LineWidth', 2, 'MarkerSize', 4);
    grid on;
    title('收敛速度 (连续改善比)');
    xlabel('AO迭代次数');
    ylabel('|Δr_{i+1}/Δr_i|');
    xlim([2, length(rate_improvements)]);
end

% 子图9：性能提升总结
subplot(3, 3, 9);
categories = {'初始', '最终'};
rate_data = [sum_rates(1), sum_rates(end)];
sensing_data = [min_sensing_powers(1)*1e3, min_sensing_powers(end)*1e3];

yyaxis left;
bar_h1 = bar([1, 2], rate_data, 0.4, 'FaceColor', 'b', 'FaceAlpha', 0.7);
ylabel('和速率 (bps/Hz)', 'Color', 'b');
set(gca, 'YColor', 'b');

yyaxis right;
bar_h2 = bar([1.4, 2.4], sensing_data, 0.4, 'FaceColor', 'r', 'FaceAlpha', 0.7);
ylabel('感知功率 (mW)', 'Color', 'r');
set(gca, 'YColor', 'r');

set(gca, 'XTick', [1.2, 2.2], 'XTickLabel', categories);
title('性能对比');
grid on;

% 保存详细分析图
saveas(gcf, 'data/detailed_ao_analysis.png');
saveas(gcf, 'data/detailed_ao_analysis.fig');
fprintf('   ✅ 详细分析图已保存\n');

%% 4. 生成天线位置演化图
fprintf('4. 生成天线位置演化图...\n');

if size(ao_history.antenna_positions, 2) >= 1 && ~isempty(ao_history.antenna_positions{1,1})
    figure('Position', [100, 100, 1200, 400], 'Name', '天线位置演化');
    
    % 获取GBS1的天线位置演化
    num_antennas = length(ao_history.antenna_positions{1,1});
    colors = lines(num_antennas);
    
    subplot(1, 2, 1);
    hold on;
    for ant_idx = 1:min(num_antennas, 8)  % 最多显示8个天线
        positions = zeros(iterations+1, 1);
        for iter = 1:iterations+1
            if ~isempty(ao_history.antenna_positions{iter,1})
                positions(iter) = ao_history.antenna_positions{iter,1}(ant_idx);
            end
        end
        plot(0:iterations, positions, '-o', 'Color', colors(ant_idx,:), ...
             'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', sprintf('天线%d', ant_idx));
    end
    grid on;
    xlabel('AO迭代次数');
    ylabel('天线位置 (λ)');
    title('天线位置演化');
    legend('Location', 'best');
    hold off;
    
    % 位置变化热力图
    subplot(1, 2, 2);
    if iterations > 1
        position_matrix = zeros(num_antennas, iterations+1);
        for iter = 1:iterations+1
            if ~isempty(ao_history.antenna_positions{iter,1})
                position_matrix(:, iter) = ao_history.antenna_positions{iter,1};
            end
        end
        imagesc(0:iterations, 1:num_antennas, position_matrix);
        colorbar;
        xlabel('AO迭代次数');
        ylabel('天线索引');
        title('天线位置热力图 (λ)');
        colormap('jet');
    end
    
    saveas(gcf, 'data/antenna_evolution.png');
    saveas(gcf, 'data/antenna_evolution.fig');
    fprintf('   ✅ 天线位置演化图已保存\n');
end

%% 5. 生成性能报告
fprintf('5. 生成性能报告...\n');

report_file = 'data/ao_performance_report.txt';
fid = fopen(report_file, 'w');

fprintf(fid, 'FAS-ISAC系统AO算法性能报告\n');
fprintf(fid, '=====================================\n');
fprintf(fid, '生成时间: %s\n\n', datestr(now));

fprintf(fid, '系统配置:\n');
fprintf(fid, '  GBS数量: %d\n', simulation_config.system_params.M);
fprintf(fid, '  UAV数量: %d\n', simulation_config.system_params.K);
fprintf(fid, '  时隙数量: %d\n', simulation_config.system_params.N);
fprintf(fid, '  天线元素: %d\n', simulation_config.system_params.Na);
fprintf(fid, '  最大功率: %.2f W\n', simulation_config.system_params.Pmax);
fprintf(fid, '  感知阈值: %.2e W\n', simulation_config.system_params.Gamma);

fprintf(fid, '\n收敛性能:\n');
fprintf(fid, '  总迭代次数: %d\n', iterations);
fprintf(fid, '  是否收敛: %s\n', ao_history.performance.converged ? '是' : '否');
if isfield(performance_analysis.convergence, 'convergence_iteration')
    fprintf(fid, '  收敛迭代: %d\n', performance_analysis.convergence.convergence_iteration);
end

fprintf(fid, '\n和速率性能:\n');
fprintf(fid, '  初始和速率: %.4f bps/Hz\n', sum_rates(1));
fprintf(fid, '  最终和速率: %.4f bps/Hz\n', sum_rates(end));
fprintf(fid, '  绝对提升: %.4f bps/Hz\n', sum_rates(end) - sum_rates(1));
fprintf(fid, '  相对提升: %.2f%%\n', (sum_rates(end) - sum_rates(1))/sum_rates(1)*100);

fprintf(fid, '\n感知功率性能:\n');
fprintf(fid, '  初始感知功率: %.4e W\n', min_sensing_powers(1));
fprintf(fid, '  最终感知功率: %.4e W\n', min_sensing_powers(end));
fprintf(fid, '  绝对提升: %.4e W\n', min_sensing_powers(end) - min_sensing_powers(1));
fprintf(fid, '  相对提升: %.2f%%\n', (min_sensing_powers(end) - min_sensing_powers(1))/min_sensing_powers(1)*100);

if isfield(performance_analysis, 'antenna_optimization')
    fprintf(fid, '\n天线位置优化:\n');
    fprintf(fid, '  总位置变化: %.4f λ\n', performance_analysis.antenna_optimization.position_change_norm);
end

fprintf(fid, '\n仿真时间:\n');
fprintf(fid, '  总耗时: %.2f分钟\n', performance_analysis.simulation_time/60);

fclose(fid);
fprintf('   ✅ 性能报告已保存到: %s\n', report_file);

%% 6. 输出分析总结
fprintf('\n📊 分析总结\n');
fprintf('=====================================\n');
fprintf('📈 收敛性能:\n');
fprintf('   迭代次数: %d\n', iterations);
fprintf('   收敛状态: %s\n', ao_history.performance.converged ? '已收敛' : '未收敛');

fprintf('\n📊 性能提升:\n');
fprintf('   和速率: %.4f → %.4f bps/Hz (提升%.2f%%)\n', ...
    sum_rates(1), sum_rates(end), (sum_rates(end)-sum_rates(1))/sum_rates(1)*100);
fprintf('   感知功率: %.2e → %.2e W (提升%.2f%%)\n', ...
    min_sensing_powers(1), min_sensing_powers(end), ...
    (min_sensing_powers(end)-min_sensing_powers(1))/min_sensing_powers(1)*100);

fprintf('\n💾 生成文件:\n');
fprintf('   详细分析图: data/detailed_ao_analysis.png\n');
fprintf('   天线演化图: data/antenna_evolution.png\n');
fprintf('   性能报告: data/ao_performance_report.txt\n');

fprintf('\n✅ AO结果分析完成！\n');
fprintf('=====================================\n');
