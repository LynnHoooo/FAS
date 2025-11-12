
%% 网络化ISAC系统模型初始化
% 论文: "Networked ISAC for Low-Altitude Economy"
% 作者: Gaoyuan Cheng et al., arXiv:2405.07568
% 时间: 2025年10月30日
% 功能: 初始化系统参数、位置、信道、感知点等

% clear; clc; close all;

%% 1. 系统参数设置
M = 3;                    % 地面基站数量 (GBSs)
K = 2;                    % 授权无人机数量 (UAVs)
Q = 20;                   % 感知采样点数量
Na = 8;                   % 每个GBS的天线元素数量
d = 0.5;                  % 参考天线间距（归一化，以波长为单位），用于初始化，相当于半波长
fc = 3e9;                 % 工作频率 3 GHz（C-band，兼顾覆盖与增益）
lambda = 3e8 / fc;        % 波长 0.1 m
% 注意：d = 0.5 表示归一化间距为 0.5*lambda，物理间距 = 0.5 * 0.1 = 0.05 m

% === 流体天线系统（FAS）参数 ===
% 设计原理：每个GBS独立优化其天线位置，实现自适应波束成形
D = 8;                     % 总孔径长度（归一化，以波长为单位），例如 8λ
t_start = 0;               % 天线阵列起始坐标（归一化）
t_end = D;                 % 天线阵列结束坐标（归一化）
d_min = 0.1;               % 最小间距约杞（归一化，避免天线过近造成互耦合）

% 初始化位置向量（多GBS设计，每个GBS独立的位置向量）
t_init = cell(M, 1);       % cell数组，每个GBS一个 Na×1 向量
t_default = linspace(t_start, t_end, Na)';  % 默认等间距分布
for m = 1:M
    % 可以为每个GBS设置不同的初始位置，这里先均用相同的
    t_init{m} = t_default;
end
fprintf('✅ FAS初始化：%d个GBS，每个%d个天线元素，孔径 [%.1f, %.1f] λ\n', M, Na, t_start, t_end);
B = 50e6;                 % 系统带宽 50 MHz（大幅提升通信容量）
N = 40;                   % 时隙总数
T_duration = 40;          % 总飞行时间 (秒)
dt = T_duration / N;      % 每个时隙时长 (秒)

Pmax = 3;                % 基站最大发射功率 (W)
% 噪声功率计算：改善信道条件，降低噪声功率谱密度
sigma2_dBm_per_Hz = -184;     % 噪声功率谱密度 (dBm/Hz) - 降低10dB以改善SNR
sigma2_dBm = sigma2_dBm_per_Hz + 10*log10(B); % 总噪声功率 (dBm)
sigma2_dBW = sigma2_dBm - 30; % 转换为 dBW
sigma2 = 10^(sigma2_dBW / 10); % 转为线性值 (W)

kappa_dB = -45;           % LoS路径损耗 at 1m（更现实）
kappa = 10^(kappa_dB / 10); % 线性值

Vmax = 10;                % 最大速度 10 m/s (≈36 km/h)
Dmin = 10;                % 最小避碰距离 (m) —— 论文未给，假设为10m

% 感知功率阈值 Gamma (可变参数，用于仿真)
Gamma_dBW = -30;          % 收紧Gamma以匹配增强的初始感知功率, -30 dBW (~1 mW)
Gamma = 10^(Gamma_dBW / 10); 

%% 2. 坐标系统设置 (3D Cartesian)
% 假设仿真区域为 400m x 400m
area_size = 400;

% GBS位置 (固定，零海拔)
u = [
    100, 200;   % GBS 1
    300, 100;   % GBS 2
    300, 300    % GBS 3
]; % M x 2, [x, y]

% UAV初始和最终位置 (水平面)
qI = [
    50, 250;   % UAV 1 初始
    50, 150    % UAV 2 初始
];

qF = [
    350, 250;  % UAV 1 最终
    350, 150   % UAV 2 最终
];

% UAV飞行高度 (固定)
H = [80; 100]; % K x 1, 单位：m

%% 3. 感知区域：100m × 50m 的矩形（长×宽）
% 感知区域中心 (cx, cy) = (200, 200)
cx = 200;
cy = 200;
width_x = 100;  % x方向长度（长边）
width_y = 50;   % y方向长度（短边）

% 定义矩形边界
x_min = cx - width_x/2; % 150
x_max = cx + width_x/2; % 250
y_min = cy - width_y/2; % 175
y_max = cx + width_y/2; % 225

% 在矩形区域内生成 Q=20 个均匀分布的感知采样点
num_x = 5;  % x方向分布点数（长边）
num_y = 4;  % y方向分布点数（短边）
x_sense = linspace(x_min, x_max, num_x);
y_sense = linspace(y_min, y_max, num_y);
[X_sense, Y_sense] = meshgrid(x_sense, y_sense);
v_x = X_sense(:);
v_y = Y_sense(:);
v = [v_x(1:Q), v_y(1:Q)]; % Q x 2, 感知点水平坐标

% 感知高度（假设统一为90m）
H_sense = 90;

% 计算每个感知点与每个GBS的距离、角度和导向矢量
d_lq = zeros(M, Q);           % d_{l,q}
theta_lq = zeros(M, Q);       % theta_{l,q}
a_theta = cell(M, Q);         % a(theta_{l,q})

for l = 1:M
    for q_idx = 1:Q
        % 3D距离
        dx = v(q_idx,1) - u(l,1);
        dy = v(q_idx,2) - u(l,2);
        d_lq(l, q_idx) = sqrt(dx^2 + dy^2 + H_sense^2);
        
        % 入射角
        theta_lq(l, q_idx) = acos(H_sense / d_lq(l, q_idx));
        
        % 流体天线导向矢量（使用该GBS的位置向量）
        % a(theta) = exp(j*2*pi*t*cos(theta))
        a_vec = exp(1j * 2*pi * t_init{l} * cos(theta_lq(l, q_idx)));
        a_theta{l, q_idx} = a_vec;
    end
end

%% 4. 初始化UAV轨迹（直线飞行）
q_traj = zeros(K, 2, N);

for k = 1:K
    x_traj = linspace(qI(k,1), qF(k,1), N);
    y_traj = linspace(qI(k,2), qF(k,2), N);
    % 正确地填充三维矩阵
    for n = 1:N
        q_traj(k, :, n) = [x_traj(n), y_traj(n)];
    end
end

%% 4.5. 预计算所有信道矩阵（避免重复计算）
fprintf('预计算通信信道矩阵...\n');
h_mkn = cell(M, K, N);  % 存储所有信道: h_{m,k}[n]

for m = 1:M
    for k = 1:K
        for n = 1:N
            % 计算GBS m到UAV k在时隙n的信道
            % 使用该GBS的位置向量 t_init{m}
            h_mkn{m, k, n} = get_channel(m, k, n, u, q_traj, H, kappa, t_init{m}, Na);
        end
    end
end
fprintf('信道预计算完成：%d个GBS × %d个UAV × %d个时隙 = %d个信道矩阵\n', M, K, N, M*K*N);

%% 5. 可视化：GBS、UAV轨迹、感知区域
figure;
hold on; grid on;

% 绘制GBS
plot(u(:,1), u(:,2), 'k^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'GBS');

% 绘制UAV轨迹
plot(squeeze(q_traj(1,1,:)), squeeze(q_traj(1,2,:)), 'b-o', 'DisplayName', 'UAV 1');
plot(squeeze(q_traj(2,1,:)), squeeze(q_traj(2,2,:)), 'r-s', 'DisplayName', 'UAV 2');

% 绘制感知区域矩形框
rectangle('Position', [x_min, y_min, width_x, width_y], ...
    'EdgeColor', 'g', 'LineWidth', 2);
% 添加矩形标签到图例 (使用虚拟线条)
plot(NaN, NaN, 'g-', 'LineWidth', 2, 'DisplayName', 'Sensing Area');

% 绘制感知采样点
plot(v(:,1), v(:,2), 'gx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Sensing Points');

xlabel('X (m)'); ylabel('Y (m)');
title('Networked ISAC System - Custom GBS & Sensing Area');
legend('show', 'Location', 'northwest'); % 放置在左上角，避免遮挡基站
axis equal;
axis([0 400 0 400]); % 设置视图范围

%% 6. 初始化波束成形 W_{m,k}[n] 和关联 alpha_{m,k}[n]

% 初始化 cell 数组
W_init = cell(M, K, N);
R_init = cell(M, 1, N);
alpha_init = zeros(M, K, N);

% === 第一步：先初始化关联 alpha ===
% 策略：基于最小距离（无干扰时的合理初始猜测）
for n = 1:N
    for k = 1:K
        dist_to_gbs = zeros(M,1);
        for m = 1:M
            dist_to_gbs(m) = norm(q_traj(k,:,n) - u(m,:));
        end
        [~, m_star] = min(dist_to_gbs);
        alpha_init(m_star, k, n) = 1;  % 只有最近的GBS服务该UAV
    end
end

% === 第二步：根据 alpha 初始化波束 W_{m,k}[n] (包含动态功率控制) ===
% 初始化感知功率占比eta (m,n)，后续可作为优化变量
% 修改策略：提高感知功率占比，确保初始状态满足Gamma约束
eta = zeros(M, N);
for m = 1:M
    for n = 1:N
        users_served = find(alpha_init(m,:,n));
        if ~isempty(users_served)
            % 修改：大幅降低感知功率占比，优先提升通信速率
            % 将eta从0.70-0.80降低到0.10-0.30
            avg_dist = mean(arrayfun(@(k) norm(squeeze(q_traj(k,:,n)) - u(m,:)), users_served));
            if avg_dist < 150
                eta(m,n) = 0.10; % 感知占10%，通信占90%
            elseif avg_dist < 250
                eta(m,n) = 0.20; % 感知占20%，通信占80%
            else
                eta(m,n) = 0.30; % 感知占30%，通信占70%
            end
        else
            eta(m,n) = 0.50; % 如果不服务任何UAV，感知占50%
        end
    end
end


for m = 1:M
    for n = 1:N
        power_comm_total = Pmax * (1 - eta(m,n));
        
        users_served = find(alpha_init(m,:,n));
        if ~isempty(users_served)
            power_per_user = power_comm_total / length(users_served);
            for k_idx = 1:length(users_served)
                k = users_served(k_idx);
                h_mk = h_mkn{m,k,n};
                w_mrt = sqrt(power_per_user) * h_mk / norm(h_mk);
                W_init{m,k,n} = w_mrt * w_mrt';
            end
        end
    end
end

% 为未服务的UAV链路设置零矩阵
for m = 1:M
    for k = 1:K
        for n = 1:N
            if alpha_init(m,k,n) == 0
                W_init{m,k,n} = zeros(Na, Na);
            end
        end
    end
end


% === 第三步：初始化感知信号 R_m[n] (优化波束成形 + 动态功率) ===
for m = 1:M
    for n = 1:N
        % 策略：对所有感知点导向矢量做加权平均，形成“广义主瓣”
        a_combined = zeros(Na, 1);
        weights = zeros(Q, 1);
        for q_idx = 1:Q
            % 权重为距离倒数平方，更强调近处目标
            weights(q_idx) = 1 / (d_lq(m, q_idx)^2 + 1e-6);
            a_combined = a_combined + weights(q_idx) * a_theta{m, q_idx};
        end
        
        if norm(a_combined) > 1e-6
            a_combined = a_combined / norm(a_combined); % 归一化
        else
            % 如果所有感知点都很远，使用默认的全向
            a_combined = ones(Na, 1) / sqrt(Na);
        end
        
        % 使用动态分配的感知功率
        power_sensing = Pmax * eta(m,n);
        R_init{m,1,n} = power_sensing * (a_combined * a_combined');
    end
end


%% 7. 计算初始通信速率（SINR 和 sum rate）

% 初始化变量
SINR_kn = zeros(K, N);   % 每个UAV在每个时隙的SINR
rate_kn = zeros(K, N);   % 通信速率 (bps/Hz)
sum_rate_n = zeros(1, N); % 每时隙总速率

fprintf('\n=== 初始通信性能 ===\n');
fprintf('时隙\tUAV1速率\tUAV2速率\t总速率\n');
fprintf('----\t--------\t--------\t------\n');

for n = 1:N
    total_rate = 0;
    for k = 1:K
        % 找出服务GBS m*
        m_serv = find(alpha_init(:,k,n) == 1, 1);
        if isempty(m_serv)
            SINR_kn(k,n) = 0;
            rate_kn(k,n) = 0;
            continue;
        end
        
        % 获取信道 h_{m*,k}[n]（使用预计算信道）
        h_mkn_temp = h_mkn{m_serv, k, n};
        
        % 通信信号功率（分子）
        W_mkn = W_init{m_serv, k, n};
        signal_power = real(h_mkn_temp' * W_mkn * h_mkn_temp); % 🔧 修复：使用real()而非abs()
        
        % 干扰 + 感知信号 + 噪声（分母）
        interference = 0;
        for m = 1:M
            for i = 1:K
                if ~(m == m_serv && i == k) % 排除自身信号
                    W_l_in = W_init{m, i, n};
                    interference = interference + real(h_mkn_temp' * W_l_in * h_mkn_temp); % 🔧 修复：使用real()
                end
            end
            % 感知信号干扰
            R_m = R_init{m,1,n};
            interference = interference + real(h_mkn_temp' * R_m * h_mkn_temp); % 🔧 修复：确保一致性
        end
        
        noise_power = sigma2;
        SINR = signal_power / (interference + noise_power);
        SINR_kn(k,n) = SINR;
        rate_kn(k,n) = log2(1 + SINR);
        total_rate = total_rate + rate_kn(k,n);
    end
    
    sum_rate_n(n) = total_rate;
    
    % 打印每5个时隙的结果（避免输出太多）
    if mod(n,5) == 1 || n == N
        fprintf('%2d\t%.4f\t%.4f\t%.4f\n', n, rate_kn(1,n), rate_kn(2,n), sum_rate_n(n));
    end
end

% 总体统计
avg_sum_rate = mean(sum_rate_n);
max_rate = max(sum_rate_n);
min_rate = min(sum_rate_n);

fprintf('\n--- 通信性能统计 ---\n');
fprintf('平均和速率: %.4f bps/Hz (%.2f Mbps)\n', avg_sum_rate, avg_sum_rate * B / 1e6);
fprintf('最大和速率: %.4f bps/Hz (%.2f Mbps)\n', max_rate, max_rate * B / 1e6);
fprintf('最小和速率: %.4f bps/Hz (%.2f Mbps)\n', min_rate, min_rate * B / 1e6);

%% 8. 计算初始感知功率（每个感知点的照射功率）

zeta_qn = zeros(Q, N); % 每个感知点在每个时隙的照射功率

fprintf('\n=== 初始感知性能 ===\n');
fprintf('时隙\t最小感知功率\t是否满足 Gamma=%.0eW?\n', Gamma);
fprintf('----\t------------\t-------------------\n');

for n = 1:N
    for q_idx = 1:Q
        power_at_q = 0;
        for m = 1:M
            % 获取该GBS在该时隙的总发射协方差矩阵
            X_m = zeros(Na, Na);
            for i = 1:K
                X_m = X_m + W_init{m, i, n};
            end
            X_m = X_m + R_init{m,1,n}; % 加上感知信号
            
            % 获取导向矢量 a(theta_{m,q})
            a_vec = a_theta{m, q_idx};
            d_lq_val = d_lq(m, q_idx);
            
            % 计算该GBS对感知点q的贡献
            power_at_q = power_at_q + (a_vec' * X_m * a_vec) / (d_lq_val^2);
        end
        zeta_qn(q_idx, n) = real(power_at_q); % 取实部
    end
    
    % 当前时隙的最小感知功率
    min_zeta_n = min(zeta_qn(:, n));
    is_satisfied = (min_zeta_n >= Gamma);
    
    if mod(n,5) == 1 || n == N
        status_text = {'否', '是'};
        fprintf('%2d\t%.4e W\t%s\n', n, min_zeta_n, status_text{is_satisfied + 1});
    end
end

% 感知性能统计
min_zeta_overall = min(zeta_qn(:));
mean_zeta_overall = mean(zeta_qn(:));

fprintf('\n--- 感知性能统计 ---\n');
fprintf('全局最小照射功率: %.4e W (%.2f dBW)\n', min_zeta_overall, 10*log10(min_zeta_overall));
fprintf('平均照射功率: %.4e W (%.2f dBW)\n', mean_zeta_overall, 10*log10(mean_zeta_overall));
fprintf('感知阈值 Gamma: %.4e W (%.1f dBW)\n', Gamma, Gamma_dBW);

% --- 正确计算系统最大可能感知功率（包含阵列增益）---
zeta_max = 0;
for q_idx = 1:Q
    zeta_at_q = 0;
    for m = 1:M
        a_vec = a_theta{m, q_idx};
        d_val = d_lq(m, q_idx);

        % 设计最优波束：GBS m 将全部功率 Pmax 用于对准感知点 q_idx
        w_opt = sqrt(Pmax) * a_vec / norm(a_vec);
        R_opt = w_opt * w_opt'; % Rank-1 beamforming matrix with Tr(R_opt) = Pmax

        % 计算该波束在该方向产生的照射功率（已包含阵列增益）
        % 修改：使用归一化路径损耗，与实际计算保持一致
        path_loss = 1 / (d_val^2);  % 归一化路径损耗
        power_contribution = path_loss * real(a_vec' * R_opt * a_vec);
        
        zeta_at_q = zeta_at_q + power_contribution;
    end
    zeta_max = max(zeta_max, zeta_at_q);
end

fprintf('系统最大感知能力 (理想情况): %.4e W (%.2f dBW)\n', zeta_max, 10*log10(zeta_max));

if min_zeta_overall >= Gamma
    fprintf('✅ 初始感知约束已满足！\n');
else
    fprintf('❌ 初始感知约束未满足，需优化波束或轨迹。\n');
end

%% 9. 可视化初始感知功率分布
figure;
scatter(v(:,1), v(:,2), 80, zeta_qn(:,1), 'filled', 'MarkerEdgeColor', 'k');
colorbar;
title('感知区域内各点初始照射功率分布 (时隙 n=1)', 'FontSize', 14);
xlabel('X 坐标 (m)', 'FontSize', 12);
ylabel('Y 坐标 (m)', 'FontSize', 12);
c = colorbar;
ylabel(c, '接收功率 (W)', 'FontSize', 12);
grid on;
set(gca, 'FontSize', 11);
% 统一颜色尺度，以便观察功率变化
if max(zeta_qn(:)) > min(zeta_qn(:))
    caxis([min(zeta_qn(:)), max(zeta_qn(:))]);
end
fprintf('✅ 已生成初始感知功率分布图。\n');

%% 9. 可视化初始感知功率热力图
fprintf('\n正在生成初始感知功率热力图...\n');
figure;
grid_resolution = 5; % m
x_range = 0:grid_resolution:area_size;
y_range = 0:grid_resolution:area_size;
[X_grid, Y_grid] = meshgrid(x_range, y_range);
power_map = zeros(size(X_grid));

for i = 1:size(X_grid, 1)
    for j = 1:size(X_grid, 2)
        grid_point = [X_grid(i,j), Y_grid(i,j)];
        power_at_point = 0;
        for m = 1:M
            % 获取该GBS在时隙1的总发射协方差
            X_m_n1 = W_init{m, 1, 1} + W_init{m, 2, 1} + R_init{m, 1, 1};
            
            % 计算导向矢量
            dx = grid_point(1) - u(m,1);
            dy = grid_point(2) - u(m,2);
            dist_3d = sqrt(dx^2 + dy^2 + H_sense^2);
            theta = acos(H_sense / dist_3d);
            % 流体天线导向矢量（使用该GBS的位置向量）
            a_vec = exp(1j * 2*pi * t_init{m} * cos(theta));
            
            % 与 zeta_qn 一致的功率计算（忽略常数 kappa，确保单位匹配）
            power_at_point = power_at_point + real(a_vec' * X_m_n1 * a_vec) / (dist_3d^2 + 1e-12);
        end
        power_map(i, j) = power_at_point;
    end
end

power_map_db = 10*log10(max(power_map, 1e-12));

pcolor(X_grid, Y_grid, power_map_db);
shading interp;
colorbar;
hold on;
plot(u(:,1), u(:,2), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'GBS');
plot(v(:,1), v(:,2), 'wx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Sensing Points');
rectangle('Position', [x_min, y_min, width_x, width_y], 'EdgeColor', 'w', 'LineStyle', '--', 'LineWidth', 2);
plot(NaN,NaN,'w--', 'DisplayName', 'Sensing Area');
title('初始感知功率分布热力图 (时隙 n=1, dBW)');
xlabel('X (m)'); ylabel('Y (m)');
legend('show');
axis equal; axis([0 400 0 400]);
valid_vals = power_map_db(isfinite(power_map_db) & power_map > 1e-8);
if ~isempty(valid_vals)
    % 让色标集中在主要功率区间（更亮）
    low_db = prctile(valid_vals, 20);
    high_db = prctile(valid_vals, 95);
    if high_db <= low_db
        low_db = min(valid_vals);
        high_db = max(valid_vals);
    end
    if high_db > low_db
        caxis([low_db, high_db]);
        tick_vals = linspace(low_db, high_db, 5);
    else
        tick_vals = [];
    end
else
    tick_vals = [];
end
c = colorbar;
ylabel(c, '接收功率 (dBW)');
if ~isempty(tick_vals)
    set(c, 'Ticks', round(tick_vals, 1));
end

fprintf('✅ 可视化完成。\n');