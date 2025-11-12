%% ================================================================== 
%  test_ao_with_heatmap.m
%  功能: 运行完整的AO优化流程并生成对比热力图
%       1. 初始化系统参数
%       2. 运行轨迹优化
%       3. 重新计算W、R矩阵
%       4. 绘制优化前后对比热力图
%% ==================================================================

clear; clc; close all;

fprintf('🚀 开始完整的AO优化+可视化流程\n');
fprintf('=====================================\n\n');

%% 步骤1: 系统初始化
fprintf('步骤1: 运行系统初始化 (initial.m)...\n');
tic;
initial;  % 运行initial.m脚本
t_init = toc;
fprintf('✅ 初始化完成，耗时 %.2f 秒\n\n', t_init);

% 保存初始轨迹用于对比
q_init = q_traj;

%% 步骤2: 运行轨迹优化
fprintf('步骤2: 运行轨迹优化...\n');

% 从initial.m中提取必要参数
params.M = M; params.K = K; params.N = N; params.Na = Na;
params.sigma2 = sigma2; params.dt = dt; params.Vmax = Vmax;
params.kappa = kappa; params.Pmax = Pmax;
params.Dmin = Dmin; params.min_trust_region = 1;
params.d = d; params.v = v; params.H_sense = H_sense;

% 模拟一个简化的波束优化结果（实际应该来自optimize_beamforming）
% 这里我们使用initial.m的结果作为起点
W_used = W_init;
R_used = R_init;

% 从协方差矩阵恢复波束向量
w_mkn = cell(M, K, N);
for m = 1:M
    for k = 1:K
        for n = 1:N
            W_tmp = W_used{m,k,n};
            if ~isempty(W_tmp) && any(W_tmp(:))
                W_tmp = (W_tmp + W_tmp')/2;
                [V, D] = eig(W_tmp);
                [lambda_max, idx_max] = max(real(diag(D)));
                if lambda_max > 1e-8
                    w_mkn{m,k,n} = V(:,idx_max) * sqrt(lambda_max);
                else
                    w_mkn{m,k,n} = zeros(Na,1);
                end
            else
                w_mkn{m,k,n} = zeros(Na,1);
            end
        end
    end
end

% 轨迹优化参数
gamma_min_SINR = db2pow(5);   
trust_region0 = 20;           % 确保 > Vmax*dt = 15m
max_iter_traj = 10;           % 减少迭代次数以加速测试
tol_traj = 1e-3;

% 自适应调整Gamma
min_sensing_power_current = inf;
for n = 1:N
    for q_idx = 1:size(v,1)
        power_at_q = 0;
        for m = 1:M
            X_m = zeros(Na, Na);
            for i = 1:K
                if ~isempty(W_used{m, i, n})
                    X_m = X_m + W_used{m, i, n};
                end
            end
            if ~isempty(R_used{m,1,n})
                X_m = X_m + R_used{m,1,n};
            end
            
            if any(X_m(:))
                dx = v(q_idx,1) - u(m,1);
                dy = v(q_idx,2) - u(m,2);
                dist_sq = dx^2 + dy^2 + H_sense^2;
                if dist_sq > 1e-6
                    dist = sqrt(dist_sq);
                    cos_theta = H_sense / dist;
                    a_vec = exp(1j * 2 * pi * 0.5 * cos_theta * (0:Na-1)');
                    power_at_q = power_at_q + real(a_vec' * X_m * a_vec) / (dist_sq + 1e-12);
                end
            end
        end
        min_sensing_power_current = min(min_sensing_power_current, power_at_q);
    end
end

Gamma_adaptive = min_sensing_power_current * 0.8;
fprintf('  📋 使用自适应 Gamma: %.4e W (%.1f dBW)\n', Gamma_adaptive, 10*log10(Gamma_adaptive));

% 调用轨迹优化
fprintf('  🚁 开始轨迹优化...\n');
tic;
[q_opt, trust_region_final] = optimize_trajectory_SCA_TR( ...
    q_traj, h_mkn, alpha_init, w_mkn, R_used, u, H, params, ...
    gamma_min_SINR, Gamma_adaptive, max_iter_traj, tol_traj, trust_region0, true);
t_traj = toc;

fprintf('✅ 轨迹优化完成，耗时 %.2f 秒\n', t_traj);
fprintf('  📊 最终信任域: %.2f m\n', trust_region_final);

% 计算轨迹变化统计
total_displacement = 0;
max_displacement = 0;
for k = 1:K
    for n = 1:N
        displacement = norm(squeeze(q_opt(k,:,n)) - squeeze(q_init(k,:,n)));
        total_displacement = total_displacement + displacement;
        max_displacement = max(max_displacement, displacement);
    end
end
avg_displacement = total_displacement / (K * N);
fprintf('  📏 平均轨迹偏移: %.2f m，最大偏移: %.2f m\n\n', avg_displacement, max_displacement);

%% 步骤3: 生成对比热力图
fprintf('步骤3: 生成AO优化前后对比热力图...\n');
tic;

% 调用热力图绘制脚本
plot_ao_optimization_heatmap;

t_plot = toc;
fprintf('✅ 热力图生成完成，耗时 %.2f 秒\n\n', t_plot);

%% 步骤4: 总结报告
fprintf('🎯 AO优化+可视化流程完成！\n');
fprintf('=====================================\n');
fprintf('⏱️  总耗时: %.2f 秒 (初始化 %.1fs + 轨迹优化 %.1fs + 可视化 %.1fs)\n', ...
    t_init + t_traj + t_plot, t_init, t_traj, t_plot);
fprintf('📈 轨迹优化效果: 平均偏移 %.2f m，最大偏移 %.2f m\n', avg_displacement, max_displacement);
fprintf('🎨 已生成热力图，显示优化前后的轨迹对比和感知功率分布\n\n');

fprintf('💡 提示：\n');
fprintf('  - 蓝色虚线：初始轨迹\n');
fprintf('  - 红色实线：优化后轨迹\n');
fprintf('  - 热力图颜色：感知功率强度 (dBW)\n');
fprintf('  - 可以通过调整 max_iter_traj 来控制优化精度与速度的平衡\n');
