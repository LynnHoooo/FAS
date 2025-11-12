%% ================================================================
%  测试脚本：运行 main_AO_algorithm.m
%  功能：调用 initial.m 初始化系统参数并组装结构体 p，随后
%       运行主 AO 算法，输出最终通信和感知性能。
%% ================================================================

clear; clc; close all;

fprintf('步骤1: 正在运行 initial.m 以准备系统初始状态...\n');
initial;
fprintf('✅ initial.m 执行完毕。\n');

%% 组装 main_AO_algorithm 所需的参数结构体 p
fprintf('\n步骤2: 正在将初始化结果打包至结构体 p...\n');
p = struct();

% 基础系统参数
p.M = M; p.K = K; p.N = N; p.Q = Q; p.Na = Na; p.B = B;
p.Pmax = Pmax; p.sigma2 = sigma2; p.Gamma = Gamma;
p.kappa = kappa; p.d = d; p.H_sense = H_sense;

% 初始化结果
p.q_traj = q_traj; p.alpha_init = alpha_init;
p.W_init = W_init; p.R_init = R_init; p.h_mkn = h_mkn;

% 位置和几何
p.u = u; p.v = v; p.H = H;
p.dt = dt; p.Vmax = Vmax; p.Dmin = Dmin;

% === FAS 天线位置参数（关键新增）===
p.t_init = t_init;      % cell(M,1) - 每个GBS的初始天线位置
p.t_start = t_start;    % 天线阵列起始坐标
p.t_end = t_end;        % 天线阵列结束坐标
p.d_min = d_min;        % 最小间距约束

fprintf('✅ 参数结构体组装完成（含FAS参数）。\n');
fprintf('   FAS: %d个GBS，孔径[%.1f, %.1f]λ，d_min=%.2fλ\n', M, t_start, t_end, d_min);

%% 调用主 AO 算法
fprintf('\n步骤3: 运行 main_AO_algorithm（含FAS位置优化）...\n');
fprintf('════════════════════════════════════════\n');
tic;
[final_sum_rate, final_min_sensing, ao_history] = main_AO_algorithm(p);
total_time = toc;
fprintf('════════════════════════════════════════\n');

%% 汇总输出
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('🎯 AO 算法最终结果（含FAS位置优化）\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% 性能指标
fprintf('\n📊 最终结果:\n');
fprintf('  平均和速率: %.4f bps/Hz\n', final_sum_rate);

% 收敛信息
if isfield(ao_history, 'performance')
    fprintf('\n🔄 收敛信息:\n');
    fprintf('  迭代次数: %d\n', ao_history.performance.iterations);
    if ao_history.performance.converged
        fprintf('  收敛状态: ✅ 已收敛\n');
    else
        fprintf('  收敛状态: ⚠️ 达到最大迭代次数\n');
    end
    
    % 性能改善
    if isfield(ao_history.performance, 'initial_sum_rate')
        rate_improve = final_sum_rate - ao_history.performance.initial_sum_rate;
        rate_improve_pct = (rate_improve / ao_history.performance.initial_sum_rate) * 100;
        fprintf('  速率改善: %.4f bps/Hz (%+.2f%%)\n', rate_improve, rate_improve_pct);
    end
end

% 天线位置信息
if isfield(ao_history, 'antenna_positions')
    fprintf('\n📡 天线位置优化:\n');
    num_iters = size(ao_history.antenna_positions, 1);
    if num_iters >= 2
        % 检查位置变化
        for m = 1:min(M, 3)  % 只显示前3个GBS
            t_initial = ao_history.antenna_positions{1, m};
            t_final = ao_history.antenna_positions{end, m};
            change = norm(t_final - t_initial);
            fprintf('  GBS %d: 位置变化 = %.4f λ\n', m, change);
        end
    end
end

% 计算时间
fprintf('\n⏱️  总耗时: %.2f 秒\n', total_time);

fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

%% 步骤4: 保存结果到data目录
fprintf('\n步骤4: 保存结果到 data 目录...\n');

% 创建data目录（如果不存在）
if ~exist('data', 'dir')
    mkdir('data');
end

% 保存文件名（带时间戳）
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save_filename = sprintf('data/ao_results_FAS_%s.mat', timestamp);

% 准备保存的数据
results = struct();

% 最终结果
results.final_sum_rate = final_sum_rate;
results.final_min_sensing = final_min_sensing;
results.total_time = total_time;

% AO历史
results.ao_history = ao_history;

% 系统参数（便于后续分析）
results.system_params = struct();
results.system_params.M = M;
results.system_params.K = K;
results.system_params.N = N;
results.system_params.Na = Na;
results.system_params.Q = Q;
results.system_params.B = B;
results.system_params.Pmax = Pmax;
results.system_params.Gamma = Gamma;
results.system_params.u = u;
results.system_params.v = v;
results.system_params.H = H;

% FAS参数
results.fas_params = struct();
results.fas_params.t_start = t_start;
results.fas_params.t_end = t_end;
results.fas_params.d_min = d_min;
results.fas_params.D = D;

% 保存到文件
try
    save(save_filename, 'results', '-v7.3');
    fprintf('  ✅ 结果已保存到: %s\n', save_filename);
    fprintf('  文件大小: %.2f MB\n', dir(save_filename).bytes / 1024 / 1024);
catch ME
    fprintf('  ⚠️ 保存失败: %s\n', ME.message);
end

%% 步骤5: 验证检查
fprintf('\n步骤5: AO算法验证检查...\n');
fprintf('────────────────────\n');

verification_passed = true;

% 检查1: 收敛性
if isfield(ao_history, 'performance')
    if isfield(ao_history.performance, 'sum_rates')
        sum_rates = ao_history.performance.sum_rates;
        % 检查是否单调递增（允许小幅波动）
        rate_diffs = diff(sum_rates);
        num_decreases = sum(rate_diffs < -1e-6);
        
        fprintf('\n1️⃣ 收敛性检查:\n');
        fprintf('   迭代次数: %d\n', length(sum_rates) - 1);
        fprintf('   速率序列: ');
        for i = 1:min(5, length(sum_rates))
            fprintf('%.2f ', sum_rates(i));
        end
        if length(sum_rates) > 5
            fprintf('... %.2f', sum_rates(end));
        end
        fprintf(' bps/Hz\n');
        
        if num_decreases == 0
            fprintf('   单调性: ✅ 严格单调递增\n');
        elseif num_decreases <= 2
            fprintf('   单调性: ⚠️ 基本单调（%d次小幅下降）\n', num_decreases);
        else
            fprintf('   单调性: ❌ 非单调（%d次下降）\n', num_decreases);
            verification_passed = false;
        end
    end
end

% 检查2: 感知约束
fprintf('\n2️⃣ 感知约束检查:\n');
fprintf('   最小感知功率: %.4e W\n', final_min_sensing);
fprintf('   感知阈值 Gamma: %.4e W\n', Gamma);
if final_min_sensing >= Gamma * 0.99  % 允许1%误差
    fprintf('   约束满足: ✅ 是 (%.2f%%超出阈值)\n', (final_min_sensing/Gamma - 1) * 100);
else
    fprintf('   约束满足: ❌ 否 (仅为阈值的%.2f%%)\n', final_min_sensing/Gamma * 100);
    verification_passed = false;
end

% 检查3: 天线位置约束
if isfield(ao_history, 'antenna_positions')
    fprintf('\n3️⃣ 天线位置约束检查:\n');
    num_iters = size(ao_history.antenna_positions, 1);
    if num_iters >= 1
        all_constraints_ok = true;
        for m = 1:M
            t_final = ao_history.antenna_positions{end, m};
            
            % 边界约束
            if any(t_final < t_start - 1e-6) || any(t_final > t_end + 1e-6)
                fprintf('   GBS %d: ❌ 边界约束违反\n', m);
                all_constraints_ok = false;
            end
            
            % 排序约束
            if any(diff(t_final) < -1e-6)
                fprintf('   GBS %d: ❌ 排序约束违反\n', m);
                all_constraints_ok = false;
            end
            
            % 最小间距
            min_gap = min(diff(t_final));
            if min_gap < d_min - 1e-6
                fprintf('   GBS %d: ❌ 最小间距约束违反 (min=%.4f < %.4f)\n', m, min_gap, d_min);
                all_constraints_ok = false;
            end
        end
        
        if all_constraints_ok
            fprintf('   所有GBS: ✅ 所有约束均满足\n');
        else
            verification_passed = false;
        end
    end
end

% 检查4: 性能提升
if isfield(ao_history, 'performance') && isfield(ao_history.performance, 'sum_rates')
    sum_rates = ao_history.performance.sum_rates;
    if length(sum_rates) >= 2
        improvement = sum_rates(end) - sum_rates(1);
        improvement_pct = (improvement / sum_rates(1)) * 100;
        
        fprintf('\n4️⃣ 性能提升检查:\n');
        fprintf('   初始速率: %.4f bps/Hz\n', sum_rates(1));
        fprintf('   最终速率: %.4f bps/Hz\n', sum_rates(end));
        fprintf('   绝对提升: %.4f bps/Hz\n', improvement);
        fprintf('   相对提升: %.2f%%\n', improvement_pct);
        
        if improvement >= -1e-6
            fprintf('   优化效果: ✅ 有改善\n');
        else
            fprintf('   优化效果: ❌ 性能下降\n');
            verification_passed = false;
        end
    end
end

fprintf('\n────────────────────\n');
if verification_passed
    fprintf('✅ 验证结果: 所有检查通过！\n');
else
    fprintf('⚠️ 验证结果: 存在问题，请检查以上输出\n');
end

%% 步骤6: 绘制收敛曲线
if isfield(ao_history, 'performance') && isfield(ao_history.performance, 'sum_rates')
    fprintf('\n步骤6: 绘制收敛曲线...\n');
    
    figure('Name', 'AO算法收敛曲线', 'Position', [100, 100, 1200, 400]);
    
    % 子图1: 和速率收敛
    subplot(1,2,1);
    sum_rates = ao_history.performance.sum_rates;
    plot(0:length(sum_rates)-1, sum_rates, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
    grid on;
    xlabel('AO迭代次数', 'FontSize', 12);
    ylabel('和速率 (bps/Hz)', 'FontSize', 12);
    title('AO算法速率收敛曲线', 'FontSize', 14);
    if length(sum_rates) >= 2
        improvement_pct = (sum_rates(end) - sum_rates(1)) / sum_rates(1) * 100;
        text(0.5, 0.95, sprintf('总改善: %.2f%%', improvement_pct), ...
            'Units', 'normalized', 'FontSize', 11, 'BackgroundColor', 'white');
    end
    
    % 子图2: 感知功率收敛
    if isfield(ao_history.performance, 'min_sensing_powers')
        subplot(1,2,2);
        sensing_powers = ao_history.performance.min_sensing_powers;
        semilogy(0:length(sensing_powers)-1, sensing_powers, 'r-s', 'LineWidth', 2, 'MarkerSize', 8);
        hold on;
        yline(Gamma, 'k--', 'LineWidth', 1.5, 'Label', '阈值 \Gamma');
        grid on;
        xlabel('AO迭代次数', 'FontSize', 12);
        ylabel('最小感知功率 (W, log scale)', 'FontSize', 12);
        title('感知功率收敛曲线', 'FontSize', 14);
        legend('最小感知功率', '阈值', 'Location', 'best');
    end
    
    % 保存图片
    fig_filename = sprintf('data/ao_convergence_%s.png', timestamp);
    saveas(gcf, fig_filename);
    fprintf('  ✅ 收敛图已保存: %s\n', fig_filename);
end

fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('🎉 测试完成！\n');
fprintf('\n📁 结果文件:\n');
fprintf('   - 数据: %s\n', save_filename);
if exist('fig_filename', 'var')
    fprintf('   - 图片: %s\n', fig_filename);
end
fprintf('\n💡 提示: 使用 load(''%s'') 加载结果进行后续分析\n', save_filename);
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
