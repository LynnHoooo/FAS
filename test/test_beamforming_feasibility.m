%% ===================================================================
%  测试脚本：单独验证 optimize_beamforming 可行性
%  功能：运行 initial.m 后直接调用 optimize_beamforming，
%        检查 Gamma / Pmax / 初始波束 是否导致 CVX 不可行。
%% ===================================================================

clear; clc; close all;

fprintf('步骤1: 运行 initial.m 准备系统参数...\n');
initial;
fprintf('✅ initial.m 执行完毕。\n');

%% 打包参数并调用波束优化
fprintf('\n步骤2: 单独调用 optimize_beamforming 验证可行性...\n');

try
    [W_test, R_test, obj_hist] = optimize_beamforming(...
        h_mkn, alpha_init, R_init, W_init, ...
        Pmax, Gamma, sigma2, M, K, N, Na, Q, v, u, H_sense, kappa);

    fprintf('\n✅ CVX 求解成功！\n');
    if ~isempty(obj_hist)
        fprintf('最终和速率: %.4f bps/Hz\n', obj_hist(end));
    end
catch ME
    fprintf('\n❌ 波束优化失败: %s\n', ME.message);
end

%% 若需要，将结果保存以便后续调试
save('beamforming_feasibility_results.mat', 'W_init', 'R_init', 'W_test', 'R_test', 'obj_hist', ...
    'Pmax', 'Gamma', 'sigma2', 'kappa', 'alpha_init', 'h_mkn', 'u', 'H', 'v', 'H_sense');


    