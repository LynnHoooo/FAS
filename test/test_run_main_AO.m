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
fprintf('\n📊 性能指标:\n');
fprintf('  和速率: %.4f bps/Hz (%.2f Mbps)\n', final_sum_rate, final_sum_rate * B / 1e6);
fprintf('  感知功率: %.4e W (%.2f dBW)\n', final_min_sensing, 10*log10(final_min_sensing));
fprintf('  感知约束: ');
if final_min_sensing >= Gamma
    fprintf('✅ 满足 (>= %.2e W)\n', Gamma);
else
    fprintf('❌ 未满足 (< %.2e W)\n', Gamma);
end

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
fprintf('✅ 测试完成！\n');
