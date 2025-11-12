function plot_ao_convergence(data_file)
    %% 绘制AO算法收敛图（通信和速率 + 感知性能）
    % 输入: 
    %   data_file (可选): 数据文件路径
    %   如果不提供，自动尝试读取：
    %     1. 新的实验数据格式: data/ao_convergence_experiment_*.mat
    %     2. 旧的数据格式: results_for_plotting.mat
    %
    % 版本: 2.0 - 支持新旧两种数据格式
    
    % 设置路径
    setup_paths;
    
    % 确定数据文件
    if nargin < 1 || isempty(data_file)
        % 优先尝试新的实验数据格式
        data_dir = 'data/';
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
        
        if ~isempty(files)
            % 使用最新的实验数据文件
            [~, idx] = max([files.datenum]);
            data_file = fullfile(data_dir, files(idx).name);
            fprintf('📁 自动选择最新数据文件: %s\n', files(idx).name);
            use_new_format = true;
        elseif exist('results_for_plotting.mat', 'file')
            % 使用旧的数据格式
            data_file = 'results_for_plotting.mat';
            fprintf('📁 使用旧数据格式: results_for_plotting.mat\n');
            use_new_format = false;
        else
            error('未找到数据文件！请先运行 test_ao_convergence_experiment.m 或 save_results.m');
        end
    else
        % 检查文件格式
        if contains(data_file, 'experiment') || contains(data_file, 'processed_results')
            use_new_format = true;
        else
            use_new_format = false;
        end
    end
    
    % 加载数据
    fprintf('📊 加载数据: %s\n', data_file);
    if use_new_format
        % 新格式：experiment_results 结构体
        load(data_file, 'experiment_results');
        ao_data = experiment_results.ao_history;
        config = experiment_results.config;
        
        total_iters = ao_data.performance.iterations;
        iterations = 1:total_iters;
        sum_rate_history = ao_data.performance.sum_rates(iterations);
        min_sensing_power_history = ao_data.performance.min_sensing_powers(iterations);
        
        iter = total_iters;
    else
        % 旧格式：直接变量
        load(data_file);
        
        if exist('final_iter','var') && ~isempty(final_iter) && final_iter > 0
            iter = min(final_iter, length(sum_rate_history));
        else
            iter = length(sum_rate_history);
        end
        sum_rate_history = sum_rate_history(1:iter);
        min_sensing_power_history = min_sensing_power_history(1:iter);
        
        % 如果是dBW，转换为线性单位
        if exist('is_dBW', 'var') && is_dBW
            min_sensing_power_history = 10.^(min_sensing_power_history / 10); % dBW -> W
        end
    end
    
    %% ========== 图1：主收敛图（双Y轴）==========
    figure('Position', [100, 100, 800, 500], 'Color', 'white');
    yyaxis left
    h_rate = plot(1:iter, sum_rate_history, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'Color', [0.85 0.325 0.099]);
    ylabel('通信和速率 (bps/Hz)', 'FontSize', 12, 'FontWeight', 'bold');

    yyaxis right
    h_sense = semilogy(1:iter, min_sensing_power_history, '-s', 'LineWidth', 2, 'MarkerSize', 6, 'Color', [0.466 0.674 0.188]);
    ylabel('最小感知功率 (W)', 'FontSize', 12, 'FontWeight', 'bold');

    xlabel('交替优化 (AO) 迭代次数', 'FontSize', 12, 'FontWeight', 'bold');
    title('AO算法收敛：通信速率与感知功率演化', 'FontSize', 14, 'FontWeight', 'bold');

    legend([h_rate, h_sense], {'通信和速率', '最小感知功率'}, 'Location', 'best', 'FontSize', 11);
    grid on; set(gca, 'GridAlpha', 0.5);
    xlim([1, iter]);
    xticks(1:iter);

    yyaxis left
    yticks(round(min(sum_rate_history)):max(1, round((max(sum_rate_history)-min(sum_rate_history))/4)):round(max(sum_rate_history)));
    yyaxis right
    valid_sensing = min_sensing_power_history(min_sensing_power_history > 0);
    if isempty(valid_sensing)
        valid_sensing = min_sensing_power_history;
    end
    all_values = valid_sensing(valid_sensing > 0);
    if isempty(all_values)
        all_values = 1e-12;
    end
    y_min = min(all_values);
    y_max = max(all_values);
    set(gca, 'YScale', 'log');
    if y_max > y_min * 1.1
        tick_min = floor(log10(y_min));
        tick_max = ceil(log10(y_max));
        yticks(logspace(tick_min, tick_max, max(2, tick_max - tick_min + 1)));
        ylim([10^tick_min * 0.9, 10^tick_max * 1.1]);
    end

    text(1, sum_rate_history(1), sprintf('%.1f', sum_rate_history(1)), 'VerticalAlignment', 'bottom', 'FontSize', 10);
    text(iter, sum_rate_history(end), sprintf('%.1f', sum_rate_history(end)), 'VerticalAlignment', 'bottom', 'FontSize', 10);

    %% 保存图像
    % 确保data目录存在
    if ~exist('data', 'dir')
        mkdir('data');
    end
    
    fig_filename = sprintf('data/ao_convergence_%s.png', datestr(now, 'yyyymmdd_HHMMSS'));
    print(fig_filename, '-dpng', '-r300');
    fprintf('✅ 收敛图已保存为: %s\n', fig_filename);