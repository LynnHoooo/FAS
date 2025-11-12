%% 分析保存的AO结果
% 用于加载和分析 test_run_main_AO.m 保存的结果

clear; clc; close all;

fprintf('📊 AO结果分析工具\n');
fprintf('================================\n\n');

%% 1. 列出所有可用的结果文件
fprintf('可用的结果文件:\n');
files = dir('data/ao_results_FAS_*.mat');
if isempty(files)
    fprintf('  ❌ 未找到结果文件\n');
    fprintf('  提示: 请先运行 test_run_main_AO.m\n');
    return;
end

for i = 1:length(files)
    fprintf('  %d. %s (%.2f MB, %s)\n', i, files(i).name, ...
        files(i).bytes/1024/1024, files(i).date);
end

%% 2. 选择文件（默认最新）
fprintf('\n选择文件: ');
if length(files) == 1
    file_idx = 1;
    fprintf('(自动选择唯一文件)\n');
else
    file_idx = length(files);  % 默认选择最新的
    fprintf('(默认选择最新文件 #%d)\n', file_idx);
end

selected_file = fullfile('data', files(file_idx).name);
fprintf('正在加载: %s\n', selected_file);

%% 3. 加载数据
load(selected_file);
fprintf('✅ 数据加载成功\n\n');

%% 4. 显示基本信息
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('📋 系统配置\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('  GBS数量: %d\n', results.system_params.M);
fprintf('  UAV数量: %d\n', results.system_params.K);
fprintf('  时隙数: %d\n', results.system_params.N);
fprintf('  天线数: %d\n', results.system_params.Na);
fprintf('  带宽: %.1f MHz\n', results.system_params.B / 1e6);
fprintf('  FAS孔径: [%.1f, %.1f]λ\n', ...
    results.fas_params.t_start, results.fas_params.t_end);

%% 5. 性能分析
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('📊 性能分析\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

if isfield(results.ao_history, 'performance')
    perf = results.ao_history.performance;
    
    % 速率分析
    if isfield(perf, 'sum_rates')
        rates = perf.sum_rates;
        fprintf('\n【和速率】\n');
        fprintf('  初始: %.4f bps/Hz (%.2f Mbps)\n', ...
            rates(1), rates(1) * results.system_params.B / 1e6);
        fprintf('  最终: %.4f bps/Hz (%.2f Mbps)\n', ...
            rates(end), rates(end) * results.system_params.B / 1e6);
        fprintf('  提升: %.4f bps/Hz (%+.2f%%)\n', ...
            rates(end) - rates(1), (rates(end)/rates(1) - 1) * 100);
        fprintf('  迭代次数: %d\n', length(rates) - 1);
    end
    
    % 感知功率分析
    if isfield(perf, 'min_sensing_powers')
        powers = perf.min_sensing_powers;
        fprintf('\n【感知功率】\n');
        fprintf('  初始: %.4e W (%.2f dBW)\n', powers(1), 10*log10(powers(1)));
        fprintf('  最终: %.4e W (%.2f dBW)\n', powers(end), 10*log10(powers(end)));
        fprintf('  阈值: %.4e W (%.2f dBW)\n', ...
            results.system_params.Gamma, 10*log10(results.system_params.Gamma));
        if powers(end) >= results.system_params.Gamma
            fprintf('  状态: ✅ 满足约束\n');
        else
            fprintf('  状态: ❌ 不满足约束\n');
        end
    end
end

%% 6. 天线位置分析
if isfield(results.ao_history, 'antenna_positions')
    fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('📡 天线位置优化分析\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');
    
    M = results.system_params.M;
    num_iters = size(results.ao_history.antenna_positions, 1);
    
    for m = 1:M
        t_init = results.ao_history.antenna_positions{1, m};
        t_final = results.ao_history.antenna_positions{end, m};
        
        fprintf('【GBS %d】\n', m);
        fprintf('  初始位置: [%.2f, %.2f, ..., %.2f]λ\n', ...
            t_init(1), t_init(2), t_init(end));
        fprintf('  最终位置: [%.2f, %.2f, ..., %.2f]λ\n', ...
            t_final(1), t_final(2), t_final(end));
        fprintf('  位置变化: %.4fλ (L2范数)\n', norm(t_final - t_init));
        fprintf('  最小间距: %.4fλ (阈值: %.2fλ)\n', ...
            min(diff(t_final)), results.fas_params.d_min);
        fprintf('\n');
    end
end

%% 7. 绘制详细分析图
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('📈 生成详细分析图...\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% 图1: 收敛曲线
figure('Name', '详细收敛分析', 'Position', [50, 50, 1400, 800]);

if isfield(results.ao_history, 'performance')
    % 子图1: 和速率
    subplot(2,2,1);
    if isfield(results.ao_history.performance, 'sum_rates')
        rates = results.ao_history.performance.sum_rates;
        plot(0:length(rates)-1, rates, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
        grid on;
        xlabel('AO迭代', 'FontSize', 11);
        ylabel('和速率 (bps/Hz)', 'FontSize', 11);
        title('和速率收敛', 'FontSize', 12, 'FontWeight', 'bold');
    end
    
    % 子图2: 感知功率
    subplot(2,2,2);
    if isfield(results.ao_history.performance, 'min_sensing_powers')
        powers = results.ao_history.performance.min_sensing_powers;
        semilogy(0:length(powers)-1, powers, 'r-s', 'LineWidth', 2, 'MarkerSize', 8);
        hold on;
        yline(results.system_params.Gamma, 'k--', 'LineWidth', 1.5);
        grid on;
        xlabel('AO迭代', 'FontSize', 11);
        ylabel('感知功率 (W, log)', 'FontSize', 11);
        title('感知功率收敛', 'FontSize', 12, 'FontWeight', 'bold');
        legend('最小感知功率', '阈值Γ', 'Location', 'best');
    end
end

% 子图3: 天线位置演化（GBS 1）
if isfield(results.ao_history, 'antenna_positions')
    subplot(2,2,3);
    num_iters = size(results.ao_history.antenna_positions, 1);
    Na = results.system_params.Na;
    
    % 绘制初始和最终位置
    t_init = results.ao_history.antenna_positions{1, 1};
    t_final = results.ao_history.antenna_positions{end, 1};
    
    stem(1:Na, t_init, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', '初始');
    hold on;
    stem(1:Na, t_final, 'r-^', 'LineWidth', 2, 'MarkerSize', 10, 'DisplayName', '最终');
    grid on;
    xlabel('天线索引', 'FontSize', 11);
    ylabel('位置 (λ)', 'FontSize', 11);
    title('GBS 1 天线位置变化', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');
end

% 子图4: 位置变化量
if isfield(results.ao_history, 'antenna_positions')
    subplot(2,2,4);
    M = results.system_params.M;
    position_changes = zeros(M, 1);
    
    for m = 1:M
        t_init = results.ao_history.antenna_positions{1, m};
        t_final = results.ao_history.antenna_positions{end, m};
        position_changes(m) = norm(t_final - t_init);
    end
    
    bar(1:M, position_changes, 'FaceColor', [0.2 0.6 0.8]);
    grid on;
    xlabel('GBS索引', 'FontSize', 11);
    ylabel('位置变化量 (λ, L2范数)', 'FontSize', 11);
    title('各GBS天线位置变化', 'FontSize', 12, 'FontWeight', 'bold');
    xticks(1:M);
end

fprintf('✅ 分析图已生成\n');

fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('✅ 分析完成！\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
