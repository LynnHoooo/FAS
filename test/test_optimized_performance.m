%% 测试优化后的通信性能
% 这个脚本测试功率分配优化后的系统性能

clear; clc; close all;

% 设置路径
setup_paths;

fprintf('🚀 测试优化后的通信性能...\n');

% 运行初始化
fprintf('📋 运行系统初始化...\n');
initial;

% 创建参数结构体
p = struct();
p.M = M; p.K = K; p.N = N; p.Q = Q; p.Na = Na; p.B = B;
p.Pmax = Pmax; p.sigma2 = sigma2; p.Gamma = Gamma;
p.kappa = kappa; p.d = d; p.H_sense = H_sense;
p.q_traj = q_traj; p.alpha_init = alpha_init;
p.W_init = W_init; p.R_init = R_init; p.h_mkn = h_mkn;
p.u = u; p.v = v; p.H = H;
p.dt = dt; p.Vmax = Vmax; p.Dmin = Dmin;

% 运行AO算法
fprintf('\n🔄 运行AO算法...\n');
[final_sum_rate, final_min_sensing] = main_AO_algorithm(p);

fprintf('\n✅ 测试完成！\n');
fprintf('📊 最终性能: 和速率=%.4f bps/Hz (%.2f Mbps)\n', final_sum_rate, final_sum_rate * B / 1e6);
fprintf('📡 最小感知功率: %.4e W\n', final_min_sensing);
