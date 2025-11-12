%% AO迭代收敛实验脚本
% 该脚本运行完整的AO算法并保存详细的收敛数据用于后续分析和绘图
% 版本: 2.0 - 支持更严格的收敛条件和断点续传
% 目的: 研究AO算法的收敛性能和迭代过程
%
% 主要改进:
% 1. 收敛条件更严格: tolerance从1e-4改为1e-6，需要连续两次迭代都满足条件才收敛
% 2. 最大迭代次数增加到30次
% 3. 每次迭代后自动保存检查点，支持断点续传
%
% 使用方法:
% 1. 正常运行: 直接运行脚本，每次迭代后会自动保存检查点
% 2. 从检查点恢复: 
%    - 设置 experiment_config.resume_from_checkpoint = true;
%    - 设置 experiment_config.checkpoint_file = 'data/ao_checkpoint_xxx.mat';
%    - 然后运行脚本
%
% 检查点文件位置: data/ao_checkpoint_<experiment_name>.mat

clear; clc; close all;

% 设置路径
setup_paths;

fprintf('🚀 开始AO迭代收敛实验...\n');
fprintf('=====================================================\n');

%% 1. 实验配置
experiment_config = struct();
experiment_config.max_iterations = 30;      % AO最大迭代次数（增加到30）
experiment_config.tolerance = 1e-6;         % 收敛容忍度（更严格，从1e-4改为1e-6）
experiment_config.trust_region = 10;        % 初始信任域半径
experiment_config.verbose = true;           % 详细输出
experiment_config.save_interval = 1;        % 每次迭代都保存
experiment_config.save_each_iteration = true;  % 启用每次迭代后保存数据
experiment_config.resume_from_checkpoint = false;  % 是否从检查点恢复
experiment_config.checkpoint_file = '';     % 检查点文件路径（如果resume_from_checkpoint为true）

% 实验标识
experiment_config.experiment_name = sprintf('AO_Convergence_%s', datestr(now, 'yyyymmdd_HHMMSS'));
fprintf('📝 实验名称: %s\n', experiment_config.experiment_name);
fprintf('📊 收敛条件: tolerance=%.2e (更严格), 最大迭代=%d\n', experiment_config.tolerance, experiment_config.max_iterations);

%% 2. 系统初始化
fprintf('\n📋 初始化ISAC系统...\n');
tic;
initial;  % 运行系统初始化
init_time = toc;
fprintf('✅ 系统初始化完成，用时: %.2f 秒\n', init_time);

% 如果从检查点恢复，加载检查点状态
if experiment_config.resume_from_checkpoint && ~isempty(experiment_config.checkpoint_file) && exist(experiment_config.checkpoint_file, 'file')
    fprintf('\n📂 从检查点恢复: %s\n', experiment_config.checkpoint_file);
    load(experiment_config.checkpoint_file, 'current_state', 'temp_ao_history');
    fprintf('✅ 检查点加载完成，从第 %d 次迭代继续\n', current_state.current_iter);
    
    % 使用检查点的状态覆盖初始化状态
    q_traj = current_state.q_current;
    alpha_init = current_state.alpha_current;
    W_init = current_state.W_current;
    R_init = current_state.R_current;
    h_mkn = current_state.h_mkn;
    start_iter = current_state.current_iter + 1;  % 从下一轮迭代开始
    
    fprintf('⚠️ 注意: 使用检查点的状态作为初始状态\n');
    fprintf('   系统参数使用当前initial.m的设置（请确保与检查点一致）\n');
else
    start_iter = 1;  % 从第一次迭代开始
end

% 显示系统配置
fprintf('\n📊 系统配置摘要:\n');
fprintf('  - GBS数量: %d\n', M);
fprintf('  - UAV数量: %d\n', K);
fprintf('  - 时隙数量: %d\n', N);
fprintf('  - 感知点数量: %d\n', Q);
fprintf('  - 系统带宽: %.1f MHz\n', B/1e6);
fprintf('  - 最大发射功率: %.1f W\n', Pmax);
fprintf('  - 感知阈值: %.1f dBW\n', 10*log10(Gamma));

%% 3. 创建参数结构体
fprintf('\n🔧 准备AO算法参数...\n');
p = struct();
% 基本参数
p.M = M; p.K = K; p.N = N; p.Q = Q; p.Na = Na; p.B = B;
p.Pmax = Pmax; p.sigma2 = sigma2; p.Gamma = Gamma;
p.kappa = kappa; p.d = d; p.H_sense = H_sense;
p.dt = dt; p.Vmax = Vmax; p.Dmin = Dmin;

% 初始化数据
p.q_traj = q_traj; p.alpha_init = alpha_init;
p.W_init = W_init; p.R_init = R_init; p.h_mkn = h_mkn;
p.u = u; p.v = v; p.H = H;

% AO算法配置
p.max_iterations = experiment_config.max_iterations;
p.tolerance = experiment_config.tolerance;
p.trust_region = experiment_config.trust_region;
p.verbose = experiment_config.verbose;
p.save_each_iteration = experiment_config.save_each_iteration;

% 设置保存文件路径
if experiment_config.resume_from_checkpoint && ~isempty(experiment_config.checkpoint_file)
    p.save_file_path = experiment_config.checkpoint_file;
else
    p.save_file_path = sprintf('data/ao_checkpoint_%s.mat', experiment_config.experiment_name);
end

% 如果从检查点恢复，使用检查点的状态作为初始状态
% （已在前面覆盖了q_traj, alpha_init等变量，这里不需要再修改p）

%% 4. 运行AO算法并收集数据
fprintf('\n🔄 开始AO算法收敛实验...\n');
fprintf('=====================================================\n');
if experiment_config.resume_from_checkpoint && exist('start_iter', 'var')
    fprintf('📌 使用检查点状态作为初始状态（检查点显示已完成 %d 次迭代）\n', start_iter - 1);
    fprintf('   将重新开始迭代计数，但使用检查点的优化结果作为起点\n');
end

experiment_start_time = tic;
[final_sum_rate, final_min_sensing, ao_history] = main_AO_algorithm(p);
total_experiment_time = toc(experiment_start_time);

fprintf('\n✅ AO算法实验完成！\n');
fprintf('📊 实验总时间: %.2f 秒\n', total_experiment_time);

%% 5. 整理和保存实验数据
fprintf('\n💾 保存实验数据...\n');

% 创建完整的实验结果结构体
experiment_results = struct();

% 实验元数据
experiment_results.meta = struct();
experiment_results.meta.experiment_name = experiment_config.experiment_name;
experiment_results.meta.timestamp = datestr(now);
experiment_results.meta.total_time = total_experiment_time;
experiment_results.meta.matlab_version = version;

% 系统配置
experiment_results.config = struct();
experiment_results.config.M = M;
experiment_results.config.K = K;
experiment_results.config.N = N;
experiment_results.config.Q = Q;
experiment_results.config.Na = Na;
experiment_results.config.B = B;
experiment_results.config.Pmax = Pmax;
experiment_results.config.sigma2 = sigma2;
experiment_results.config.Gamma = Gamma;
experiment_results.config.max_iterations = experiment_config.max_iterations;
experiment_results.config.tolerance = experiment_config.tolerance;

% AO收敛历史数据
experiment_results.ao_history = ao_history;

% 最终结果
experiment_results.final_results = struct();
experiment_results.final_results.sum_rate = final_sum_rate;
experiment_results.final_results.sum_rate_mbps = final_sum_rate * B / 1e6;
experiment_results.final_results.min_sensing = final_min_sensing;
experiment_results.final_results.converged = ao_history.performance.converged;
experiment_results.final_results.total_iterations = ao_history.performance.iterations;

% 初始性能
experiment_results.initial_results = struct();
experiment_results.initial_results.sum_rate = ao_history.performance.sum_rates(1);
experiment_results.initial_results.sum_rate_mbps = ao_history.performance.sum_rates(1) * B / 1e6;
experiment_results.initial_results.min_sensing = ao_history.performance.min_sensing_powers(1);

% 性能改善
performance_improvement = struct();
performance_improvement.rate_improvement_bps_hz = final_sum_rate - ao_history.performance.sum_rates(1);
performance_improvement.rate_improvement_mbps = performance_improvement.rate_improvement_bps_hz * B / 1e6;
performance_improvement.rate_improvement_percent = (performance_improvement.rate_improvement_bps_hz / ao_history.performance.sum_rates(1)) * 100;
performance_improvement.sensing_improvement = final_min_sensing - ao_history.performance.min_sensing_powers(1);
experiment_results.performance_improvement = performance_improvement;

% 确保数据目录存在
if ~exist('data', 'dir')
    mkdir('data');
end

% 保存数据
data_filename = sprintf('data/ao_convergence_experiment_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(data_filename, 'experiment_results', '-v7.3');

% 保存CSV格式的收敛数据（便于其他软件分析）
csv_filename = sprintf('data/ao_convergence_data_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'));
iterations = 1:ao_history.performance.iterations;
sum_rates = ao_history.performance.sum_rates(1:ao_history.performance.iterations);
sum_rates_mbps = sum_rates * B / 1e6;
min_sensing_powers = ao_history.performance.min_sensing_powers(1:ao_history.performance.iterations);
trust_regions = ao_history.trust_regions(1:ao_history.performance.iterations);

% 创建表格
convergence_table = table(iterations', sum_rates', sum_rates_mbps', min_sensing_powers', trust_regions', ...
    'VariableNames', {'Iteration', 'SumRate_bps_Hz', 'SumRate_Mbps', 'MinSensingPower_W', 'TrustRegion_m'});
writetable(convergence_table, csv_filename);

fprintf('✅ 实验数据已保存:\n');
fprintf('  📁 MATLAB数据: %s\n', data_filename);
fprintf('  📊 CSV数据: %s\n', csv_filename);

%% 6. 生成实验摘要报告
fprintf('\n📋 实验摘要报告\n');
fprintf('=====================================================\n');
if experiment_results.final_results.converged
    fprintf('🎯 收敛状态: ✅ 已收敛\n');
else
    fprintf('🎯 收敛状态: ⚠️ 未收敛\n');
end
fprintf('🔄 总迭代次数: %d / %d\n', ao_history.performance.iterations, experiment_config.max_iterations);
fprintf('⏱️  总实验时间: %.2f 秒\n', total_experiment_time);

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

%% 7. 快速可视化
fprintf('\n📈 生成快速收敛图...\n');
figure('Name', 'AO Convergence Analysis', 'Position', [100, 100, 1200, 800]);

% 子图1: 和速率收敛
subplot(2, 2, 1);
plot(iterations, sum_rates_mbps, 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
xlabel('AO 迭代次数');
ylabel('和速率 (Mbps)');
title('通信性能收敛');
xlim([1, max(iterations)]);

% 子图2: 感知功率
subplot(2, 2, 2);
semilogy(iterations, min_sensing_powers, 'r-s', 'LineWidth', 2, 'MarkerSize', 6);
hold on;
semilogy([1, max(iterations)], [Gamma, Gamma], 'k--', 'LineWidth', 2);
grid on;
xlabel('AO 迭代次数');
ylabel('最小感知功率 (W)');
title('感知性能收敛');
legend('最小感知功率', 'Gamma阈值', 'Location', 'best');
xlim([1, max(iterations)]);

% 子图3: 信任域变化
subplot(2, 2, 3);
plot(iterations, trust_regions, 'g-^', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
xlabel('AO 迭代次数');
ylabel('信任域半径 (m)');
title('信任域调整');
xlim([1, max(iterations)]);

% 子图4: 性能改善率
if length(iterations) > 1
    improvement_rates = zeros(1, length(iterations)-1);
    for i = 2:length(iterations)
        improvement_rates(i-1) = (sum_rates(i) - sum_rates(i-1)) / sum_rates(i-1) * 100;
    end
    subplot(2, 2, 4);
    bar(2:length(iterations), improvement_rates, 'FaceColor', [0.2, 0.6, 0.8]);
    grid on;
    xlabel('AO 迭代次数');
    ylabel('相对改善率 (%)');
    title('逐次迭代改善');
    xlim([1.5, max(iterations)+0.5]);
end

sgtitle(sprintf('AO收敛分析 - %s', experiment_config.experiment_name), 'FontSize', 14, 'FontWeight', 'bold');

% 保存图片
fig_filename = sprintf('data/ao_convergence_plot_%s.png', datestr(now, 'yyyymmdd_HHMMSS'));
saveas(gcf, fig_filename);
fprintf('📊 收敛图已保存: %s\n', fig_filename);

fprintf('\n🎉 AO迭代收敛实验完成！\n');
fprintf('所有数据和图片已保存到 data/ 目录\n');

%% 8. 返回实验结果供进一步分析
fprintf('\n💡 实验数据已存储在工作空间变量 "experiment_results" 中\n');
fprintf('您可以使用该变量进行进一步的数据分析和可视化\n');
