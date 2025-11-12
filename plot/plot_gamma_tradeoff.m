function plot_gamma_tradeoff()
%% 绘制 感知阈值-通信速率 权衡曲线
%  加载 gamma_tradeoff_results.mat 并可视化结果

% 1. 加载数据
if ~exist('gamma_tradeoff_results.mat', 'file')
    error('未找到 gamma_tradeoff_results.mat，请先运行 sweep_gamma_tradeoff.m');
end
load('gamma_tradeoff_results.mat');

fprintf('加载扫描结果并开始绘图...\n');

%% 2. 绘图
figure('Position', [100, 100, 700, 500], 'Color', 'white');
plot(gamma_list_dBW, avg_sum_rate_list, '-o', 'LineWidth', 2, 'Color', [0.2 0.4 0.6], 'MarkerSize', 8, 'MarkerFaceColor', [0.2 0.4 0.6]);
xlabel('感知功率阈值 \Gamma (dBW)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('最终平均通信和速率 (bps/Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('ISAC系统通信-感知性能权衡曲线', 'FontSize', 14, 'FontWeight', 'bold');
grid on; 
set(gca, 'GridAlpha', 0.5, 'FontSize', 11);

% 标注趋势
% 找到非NaN的第一个和最后一个点进行标注
valid_indices = find(~isnan(avg_sum_rate_list));
if ~isempty(valid_indices)
    first_idx = valid_indices(1);
    last_idx = valid_indices(end);
    
    text(gamma_list_dBW(first_idx), avg_sum_rate_list(first_idx), '  \leftarrow 宽松感知, 高通信性能', ...
        'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left', 'FontSize', 10, 'Color', 'k');
    
    text(gamma_list_dBW(last_idx), avg_sum_rate_list(last_idx), '  \leftarrow 严格感知, 低通信性能', ...
        'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left', 'FontSize', 10, 'Color', 'k');
end

drawnow;

%% 3. 保存图像
try
    print('gamma_tradeoff.png', '-dpng', '-r300');
    print('gamma_tradeoff.pdf', '-dpdf');
    fprintf('✅ 权衡图已成功保存为: gamma_tradeoff.png/pdf\n');
catch ME
    fprintf('❌ 保存图像失败: %s\n', ME.message);
end

end
