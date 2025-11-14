%% 完整FAS-ISAC系统仿真
% 运行完整的AO算法并保存所有过程数据用于后续分析
% 
% 输出文件：
%   - data/ao_convergence_results.mat: 完整的AO迭代历史数据
%   - data/simulation_config.mat: 仿真配置参数
%   - data/performance_analysis.mat: 性能分析结果

clear; clc; close all;

fprintf('🚀 开始完整FAS-ISAC系统仿真\n');
fprintf('=====================================\n\n');

%% 1. 系统初始化
fprintf('1. 系统初始化...\n');
tic;
setup_paths;
initial;
init_time = toc;
fprintf('   ✅ 系统初始化完成 (耗时: %.2f秒)\n', init_time);

%% 2. 保存仿真配置
fprintf('2. 保存仿真配置...\n');
if ~exist('data', 'dir')
    mkdir('data');
    fprintf('   📁 创建data目录\n');
end

% 保存系统配置参数
simulation_config = struct();
simulation_config.system_params = struct();
simulation_config.system_params.M = M;
simulation_config.system_params.K = K;
simulation_config.system_params.N = N;
simulation_config.system_params.Q = Q;
simulation_config.system_params.Na = Na;
simulation_config.system_params.B = B;
simulation_config.system_params.Pmax = Pmax;
simulation_config.system_params.sigma2 = sigma2;
simulation_config.system_params.Gamma = Gamma;
simulation_config.system_params.kappa = kappa;
simulation_config.system_params.d = d;
simulation_config.system_params.H_sense = H_sense;
simulation_config.system_params.area_size = area_size;

% 保存网络拓扑
simulation_config.network_topology = struct();
simulation_config.network_topology.u = u;  % GBS位置
simulation_config.network_topology.v = v;  % 感知点位置
simulation_config.network_topology.H = H;  % UAV高度
simulation_config.network_topology.q_traj = q_traj;  % UAV轨迹
simulation_config.network_topology.alpha_init = alpha_init;  % 初始关联

% 保存初始波束和天线位置
simulation_config.initial_state = struct();
simulation_config.initial_state.W_init = W_init;
simulation_config.initial_state.R_init = R_init;
simulation_config.initial_state.t_init = t_init;

% 保存仿真时间戳
simulation_config.timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
simulation_config.matlab_version = version;

save('data/simulation_config.mat', 'simulation_config', '-v7.3');
fprintf('   ✅ 仿真配置已保存到: data/simulation_config.mat\n');

%% 3. 准备AO算法参数
fprintf('3. 准备AO算法参数...\n');

% 构建参数结构体
p = struct();

% 基本系统参数
p.M = M; p.K = K; p.N = N; p.Q = Q; p.Na = Na; p.B = B;
p.Pmax = Pmax; p.sigma2 = sigma2; p.Gamma = Gamma;
p.kappa = kappa; p.d = d; p.H_sense = H_sense;

% 网络拓扑
p.u = u; p.v = v; p.H = H;
p.q_traj = q_traj; p.alpha_init = alpha_init;

% 初始状态
p.W_init = W_init; p.R_init = R_init; p.t_init = t_init;
p.h_mkn = h_mkn;

% 轨迹优化参数
p.dt = dt; p.Vmax = Vmax; p.Dmin = Dmin;

% 天线位置优化参数
p.t_start = zeros(Na, 1);
p.t_end = 10 * ones(Na, 1);
p.d_min = 0.5;

fprintf('   ✅ AO算法参数准备完成\n');

%% 4. 运行完整AO算法
fprintf('4. 运行完整AO算法...\n');
fprintf('   这可能需要几分钟时间，请耐心等待...\n\n');

ao_start_time = tic;
[final_sum_rate, final_min_sensing, ao_history] = main_AO_algorithm(p);
ao_total_time = toc(ao_start_time);

fprintf('\n✅ AO算法完成！\n');
fprintf('   总耗时: %.2f分钟\n', ao_total_time/60);
fprintf('   最终和速率: %.4f bps/Hz\n', final_sum_rate);
fprintf('   最终感知功率: %.4e W\n', final_min_sensing);

%% 5. 性能分析
fprintf('5. 进行性能分析...\n');

performance_analysis = struct();

% 收敛性分析
sum_rates = ao_history.performance.sum_rates;
min_sensing_powers = ao_history.performance.min_sensing_powers;
iterations = length(sum_rates) - 1;  % 减去初始状态

performance_analysis.convergence = struct();
performance_analysis.convergence.iterations = iterations;
performance_analysis.convergence.converged = ao_history.performance.converged;
performance_analysis.convergence.sum_rate_improvement = final_sum_rate - sum_rates(1);
performance_analysis.convergence.sum_rate_improvement_percent = ...
    (final_sum_rate - sum_rates(1)) / sum_rates(1) * 100;
performance_analysis.convergence.sensing_improvement = final_min_sensing - min_sensing_powers(1);
performance_analysis.convergence.sensing_improvement_percent = ...
    (final_min_sensing - min_sensing_powers(1)) / min_sensing_powers(1) * 100;

% 计算收敛速度（连续两次迭代的改善小于阈值）
convergence_threshold = 0.01;  % 1%改善阈值
for i = 2:length(sum_rates)
    if i > 2
        rate_improvement = abs(sum_rates(i) - sum_rates(i-1)) / sum_rates(i-1);
        if rate_improvement < convergence_threshold
            performance_analysis.convergence.convergence_iteration = i - 1;
            break;
        end
    end
end

if ~isfield(performance_analysis.convergence, 'convergence_iteration')
    performance_analysis.convergence.convergence_iteration = iterations;
end

% 天线位置变化分析
performance_analysis.antenna_optimization = struct();
initial_positions = ao_history.antenna_positions{1, 1};  % GBS1的初始位置
final_positions = ao_history.antenna_positions{end, 1};   % GBS1的最终位置
position_change = norm(final_positions - initial_positions);
performance_analysis.antenna_optimization.position_change_norm = position_change;
performance_analysis.antenna_optimization.initial_positions = initial_positions;
performance_analysis.antenna_optimization.final_positions = final_positions;

% 计算每次迭代的位置变化
position_changes = zeros(iterations, 1);
for i = 2:length(ao_history.antenna_positions(:,1))
    if ~isempty(ao_history.antenna_positions{i,1}) && ~isempty(ao_history.antenna_positions{i-1,1})
        pos_change = norm(ao_history.antenna_positions{i,1} - ao_history.antenna_positions{i-1,1});
        position_changes(i-1) = pos_change;
    end
end
performance_analysis.antenna_optimization.position_changes_per_iteration = position_changes;

% 保存性能分析结果
performance_analysis.simulation_time = ao_total_time;
performance_analysis.timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');

save('data/performance_analysis.mat', 'performance_analysis', '-v7.3');
fprintf('   ✅ 性能分析已保存到: data/performance_analysis.mat\n');

%% 6. 生成收敛图
fprintf('6. 生成收敛图...\n');

% 创建收敛图
figure('Position', [100, 100, 1200, 800]);

% 子图1：和速率收敛
subplot(2, 2, 1);
plot(0:iterations, sum_rates, 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
title('和速率收敛曲线');
xlabel('AO迭代次数');
ylabel('和速率 (bps/Hz)');
legend('和速率', 'Location', 'best');

% 子图2：感知功率收敛
subplot(2, 2, 2);
semilogy(0:iterations, min_sensing_powers, 'r-s', 'LineWidth', 2, 'MarkerSize', 6);
grid on;
title('最小感知功率收敛曲线');
xlabel('AO迭代次数');
ylabel('最小感知功率 (W)');
legend('最小感知功率', 'Location', 'best');

% 子图3：天线位置变化
subplot(2, 2, 3);
if length(position_changes) > 0
    semilogy(1:length(position_changes), position_changes, 'g-^', 'LineWidth', 2, 'MarkerSize', 6);
    grid on;
    title('天线位置变化');
    xlabel('AO迭代次数');
    ylabel('位置变化 (λ)');
    legend('位置变化范数', 'Location', 'best');
else
    text(0.5, 0.5, '无位置变化数据', 'HorizontalAlignment', 'center');
    title('天线位置变化');
end

% 子图4：性能提升总结
subplot(2, 2, 4);
bar_data = [sum_rates(1), final_sum_rate; min_sensing_powers(1)*1e3, final_min_sensing*1e3];
bar_handle = bar(bar_data);
set(gca, 'XTickLabel', {'和速率 (bps/Hz)', '感知功率 (mW)'});
legend('初始值', '最终值', 'Location', 'best');
title('性能提升对比');
grid on;

% 保存收敛图
saveas(gcf, 'data/ao_convergence_plot.png');
saveas(gcf, 'data/ao_convergence_plot.fig');
fprintf('   ✅ 收敛图已保存到: data/ao_convergence_plot.png\n');

%% 7. 输出仿真总结
fprintf('\n🎯 完整仿真总结\n');
fprintf('=====================================\n');
fprintf('📊 性能指标:\n');
fprintf('   初始和速率: %.4f bps/Hz\n', sum_rates(1));
fprintf('   最终和速率: %.4f bps/Hz\n', final_sum_rate);
fprintf('   和速率提升: %.4f bps/Hz (%.2f%%)\n', ...
    performance_analysis.convergence.sum_rate_improvement, ...
    performance_analysis.convergence.sum_rate_improvement_percent);
fprintf('   初始感知功率: %.4e W\n', min_sensing_powers(1));
fprintf('   最终感知功率: %.4e W\n', final_min_sensing);
fprintf('   感知功率提升: %.4e W (%.2f%%)\n', ...
    performance_analysis.convergence.sensing_improvement, ...
    performance_analysis.convergence.sensing_improvement_percent);

fprintf('\n🔄 收敛性:\n');
fprintf('   总迭代次数: %d\n', iterations);
fprintf('   是否收敛: %s\n', performance_analysis.convergence.converged ? '是' : '否');
fprintf('   收敛迭代: %d\n', performance_analysis.convergence.convergence_iteration);
fprintf('   天线位置变化: %.4f λ\n', position_change);

fprintf('\n💾 数据文件:\n');
fprintf('   AO历史数据: data/ao_convergence_results.mat\n');
fprintf('   仿真配置: data/simulation_config.mat\n');
fprintf('   性能分析: data/performance_analysis.mat\n');
fprintf('   收敛图: data/ao_convergence_plot.png\n');

fprintf('\n✅ 完整仿真完成！总耗时: %.2f分钟\n', (toc + ao_total_time)/60);
fprintf('=====================================\n');
