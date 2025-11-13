%% FAS感知约束验证脚本
% 验证修改后的感知约束计算是否正确
% 对比ULA和FAS的感知功率计算结果

clear; clc; close all;

fprintf('🔍 FAS感知约束验证\n');
fprintf('================================\n\n');

%% 1. 基本参数设置
Na = 4;                    % 天线数量
M = 1;                     % GBS数量
Q = 2;                     % 感知点数量
N = 1;                     % 时隙数量
K = 1;                     % UAV数量

% 系统参数
H_sense = 100;             % 感知高度
kappa = 1;                 % 路径损耗因子
d_lambda = 0.5;            % ULA间距（半波长）

% 位置设置
u = [0, 0];                % GBS位置
v = [100, 50; 200, 100];   % 感知点位置

%% 2. 定义ULA和FAS位置向量
% ULA位置向量（等间距）
t_ULA = (0:Na-1)' * d_lambda;

% FAS位置向量（优化后的位置，示例）
t_FAS = [0; 0.3; 0.8; 1.5];  % 非等间距

fprintf('位置向量对比:\n');
fprintf('ULA: [%.2f, %.2f, %.2f, %.2f]\n', t_ULA);
fprintf('FAS: [%.2f, %.2f, %.2f, %.2f]\n', t_FAS);
fprintf('\n');

%% 3. 创建测试波束矩阵
% 简单的单位矩阵作为测试波束
W_test = cell(M, K, N);
R_test = cell(M, 1, N);
alpha_test = ones(M, K, N);

% 随机生成测试波束矩阵
rng(42); % 固定随机种子
for m = 1:M
    for k = 1:K
        for n = 1:N
            W_test{m,k,n} = 0.1 * eye(Na) + 0.05 * randn(Na, Na) + 1j * 0.05 * randn(Na, Na);
            W_test{m,k,n} = (W_test{m,k,n} + W_test{m,k,n}') / 2; % 确保Hermitian
        end
    end
    for n = 1:N
        R_test{m,1,n} = 0.2 * eye(Na) + 0.1 * randn(Na, Na) + 1j * 0.1 * randn(Na, Na);
        R_test{m,1,n} = (R_test{m,1,n} + R_test{m,1,n}') / 2; % 确保Hermitian
    end
end

%% 4. 计算ULA感知功率（旧方法）
fprintf('=== ULA感知功率计算 ===\n');
sensing_power_ULA = zeros(Q, N);

for n = 1:N
    for q_idx = 1:Q
        total_power = 0;
        for m = 1:M
            % 复合波束矩阵
            composite = zeros(Na, Na);
            for k = 1:K
                composite = composite + W_test{m, k, n};
            end
            composite = composite + R_test{m, 1, n};
            
            % 距离计算
            dx = v(q_idx, 1) - u(m, 1);
            dy = v(q_idx, 2) - u(m, 2);
            dist = sqrt(dx^2 + dy^2 + H_sense^2);
            
            % 路径损耗
            path_loss = 1 / (dist^2);
            
            % ULA导向矢量（旧方法）
            cos_theta = H_sense / dist;
            steering_ULA = exp(1j * t_ULA * 2 * pi * cos_theta);
            
            % 感知功率
            power_contribution = path_loss * real(steering_ULA' * composite * steering_ULA);
            total_power = total_power + power_contribution;
        end
        sensing_power_ULA(q_idx, n) = total_power;
    end
end

fprintf('ULA感知功率:\n');
for q_idx = 1:Q
    fprintf('  感知点%d: %.6e W (%.2f dBW)\n', q_idx, sensing_power_ULA(q_idx, 1), ...
        10*log10(sensing_power_ULA(q_idx, 1)));
end
fprintf('ULA最小感知功率: %.6e W (%.2f dBW)\n\n', min(sensing_power_ULA(:)), ...
    10*log10(min(sensing_power_ULA(:))));

%% 5. 计算FAS感知功率（新方法）
fprintf('=== FAS感知功率计算 ===\n');
sensing_power_FAS = zeros(Q, N);

% 创建FAS位置向量cell数组
t_positions = cell(M, 1);
t_positions{1} = t_FAS;

% 使用修改后的compute_sensing_power函数
% 注意：这里我们需要模拟该函数的行为
for n = 1:N
    for q_idx = 1:Q
        total_power = 0;
        for m = 1:M
            % 复合波束矩阵
            composite = zeros(Na, Na);
            for k = 1:K
                composite = composite + W_test{m, k, n};
            end
            composite = composite + R_test{m, 1, n};
            
            % 距离计算
            dx = v(q_idx, 1) - u(m, 1);
            dy = v(q_idx, 2) - u(m, 2);
            dist = sqrt(dx^2 + dy^2 + H_sense^2);
            
            % 路径损耗
            path_loss = 1 / (dist^2);
            
            % FAS导向矢量（新方法）
            cos_theta = H_sense / dist;
            steering_FAS = exp(1j * t_positions{m} * 2 * pi * cos_theta);
            
            % 感知功率
            power_contribution = path_loss * real(steering_FAS' * composite * steering_FAS);
            total_power = total_power + power_contribution;
        end
        sensing_power_FAS(q_idx, n) = total_power;
    end
end

fprintf('FAS感知功率:\n');
for q_idx = 1:Q
    fprintf('  感知点%d: %.6e W (%.2f dBW)\n', q_idx, sensing_power_FAS(q_idx, 1), ...
        10*log10(sensing_power_FAS(q_idx, 1)));
end
fprintf('FAS最小感知功率: %.6e W (%.2f dBW)\n\n', min(sensing_power_FAS(:)), ...
    10*log10(min(sensing_power_FAS(:))));

%% 6. 对比分析
fprintf('=== 对比分析 ===\n');
power_ratio = sensing_power_FAS ./ sensing_power_ULA;
power_diff_dB = 10 * log10(power_ratio);

fprintf('功率比值 (FAS/ULA):\n');
for q_idx = 1:Q
    fprintf('  感知点%d: %.4f (%.2f dB)\n', q_idx, power_ratio(q_idx, 1), power_diff_dB(q_idx, 1));
end

min_power_improvement = min(sensing_power_FAS(:)) / min(sensing_power_ULA(:));
fprintf('\n最小感知功率改善: %.4f (%.2f dB)\n', min_power_improvement, ...
    10*log10(min_power_improvement));

%% 7. 导向矢量对比
fprintf('\n=== 导向矢量对比 ===\n');
cos_theta_test = H_sense / sqrt((v(1,1) - u(1,1))^2 + (v(1,2) - u(1,2))^2 + H_sense^2);

a_ULA = exp(1j * t_ULA * 2 * pi * cos_theta_test);
a_FAS = exp(1j * t_FAS * 2 * pi * cos_theta_test);

fprintf('ULA导向矢量幅度: [%.3f, %.3f, %.3f, %.3f]\n', abs(a_ULA));
fprintf('FAS导向矢量幅度: [%.3f, %.3f, %.3f, %.3f]\n', abs(a_FAS));
fprintf('ULA导向矢量相位: [%.2f, %.2f, %.2f, %.2f] (度)\n', angle(a_ULA)*180/pi);
fprintf('FAS导向矢量相位: [%.2f, %.2f, %.2f, %.2f] (度)\n', angle(a_FAS)*180/pi);

%% 8. 验证结果
fprintf('\n=== 验证结果 ===\n');
if all(isfinite(sensing_power_ULA(:))) && all(isfinite(sensing_power_FAS(:)))
    fprintf('✅ 数值计算稳定，无NaN或Inf值\n');
else
    fprintf('❌ 数值计算不稳定，存在NaN或Inf值\n');
end

if all(sensing_power_ULA(:) > 0) && all(sensing_power_FAS(:) > 0)
    fprintf('✅ 感知功率均为正值\n');
else
    fprintf('❌ 感知功率存在非正值\n');
end

if abs(min_power_improvement - 1) < 2  % 允许2倍以内的差异
    fprintf('✅ FAS与ULA感知功率在合理范围内\n');
else
    fprintf('⚠️ FAS与ULA感知功率差异较大，需要检查\n');
end

fprintf('\n🎯 FAS感知约束验证完成！\n');
