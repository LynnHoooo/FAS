%% ================================================================== 
%  plot_ao_optimization_heatmap.m 
%  功能: AO优化完成后，根据优化轨迹重新计算W、R，绘制感知功率热力图
%       参考 initial.m 的布局，对比优化前后的轨迹和感知性能
%% ==================================================================

%% 1. 检查必要变量并加载数据
fprintf('\n=== AO优化后热力图可视化 ===\n');

% 检查优化结果是否存在
required_vars = {'q_opt', 'q_init', 'u', 'H', 'v', 'H_sense', 'kappa', 'sigma2', 'Pmax'};
missing_vars = {};
for i = 1:length(required_vars)
    if ~exist(required_vars{i}, 'var')
        missing_vars{end+1} = required_vars{i};
    end
end

if ~isempty(missing_vars)
    error('❌ 缺少必要变量: %s\n请先运行 initial.m 和轨迹优化脚本', strjoin(missing_vars, ', '));
end

% 提取基本参数
[K, ~, N] = size(q_opt);
M = size(u, 1);
Q = size(v, 1);
Na = 8;  % 天线阵元数
d = 0.5; % 天线间距

fprintf('✅ 变量检查完成。系统配置: M=%d GBS, K=%d UAV, N=%d时隙, Q=%d感知点\n', M, K, N, Q);

%% 2. 根据优化轨迹重新计算信道矩阵
fprintf('🔄 重新计算优化轨迹对应的信道矩阵...\n');
h_mkn_opt = cell(M, K, N);

for m = 1:M
    for k = 1:K
        for n = 1:N
            % 使用优化后的轨迹重新计算信道
            h_mkn_opt{m, k, n} = get_channel(m, k, n, u, q_opt, H, kappa, d, Na);
        end
    end
end

fprintf('✅ 信道重计算完成：%d个GBS × %d个UAV × %d个时隙 = %d个信道矩阵\n', M, K, N, M*K*N);

%% 3. 根据优化轨迹重新计算关联 alpha_opt
fprintf('🔄 重新计算优化轨迹的关联策略...\n');
alpha_opt = zeros(M, K, N);

% 策略：基于最小距离（与initial.m一致）
for n = 1:N
    for k = 1:K
        dist_to_gbs = zeros(M,1);
        for m = 1:M
            dist_to_gbs(m) = norm(q_opt(k,:,n) - u(m,:));
        end
        [~, m_star] = min(dist_to_gbs);
        alpha_opt(m_star, k, n) = 1;  % 最近的GBS服务该UAV
    end
end

%% 4. 根据优化轨迹重新计算W、R矩阵
fprintf('🔄 重新计算优化轨迹的波束成形矩阵...\n');

% 初始化矩阵
W_opt = cell(M, K, N);
R_opt = cell(M, 1, N);

% 动态功率控制参数 eta（与initial.m一致）
eta_opt = zeros(M, N); 
for m = 1:M
    for n = 1:N
        users_served = find(alpha_opt(m,:,n));
        if ~isempty(users_served)
            avg_dist = mean(arrayfun(@(k) norm(squeeze(q_opt(k,:,n)) - u(m,:)), users_served));
            % 距离自适应功率分配
            if avg_dist < 150
                eta_opt(m,n) = 0.8; % 感知占80%
            elseif avg_dist < 250
                eta_opt(m,n) = 0.6;
            else
                eta_opt(m,n) = 0.4;
            end
        else
            eta_opt(m,n) = 0.9; % 不服务UAV时，绝大部分功率给感知
        end
    end
end

% 计算通信波束 W_opt
for m = 1:M
    for n = 1:N
        power_comm_total = Pmax * (1 - eta_opt(m,n));
        
        users_served = find(alpha_opt(m,:,n));
        if ~isempty(users_served)
            power_per_user = power_comm_total / length(users_served);
            for k_idx = 1:length(users_served)
                k = users_served(k_idx);
                h_mk = h_mkn_opt{m,k,n};
                w_mrt = sqrt(power_per_user) * h_mk / norm(h_mk);
                W_opt{m,k,n} = w_mrt * w_mrt';
            end
        end
    end
end

% 为未服务的UAV链路设置零矩阵
for m = 1:M
    for k = 1:K
        for n = 1:N
            if alpha_opt(m,k,n) == 0
                W_opt{m,k,n} = zeros(Na, Na);
            end
        end
    end
end

% 重新计算感知点参数（与initial.m一致）
d_lq = zeros(M, Q);           
theta_lq = zeros(M, Q);       
a_theta = cell(M, Q);         

for l = 1:M
    for q_idx = 1:Q
        % 3D距离
        dx = v(q_idx,1) - u(l,1);
        dy = v(q_idx,2) - u(l,2);
        d_lq(l, q_idx) = sqrt(dx^2 + dy^2 + H_sense^2);
        
        % 入射角
        theta_lq(l, q_idx) = acos(H_sense / d_lq(l, q_idx));
        
        % ULA导向矢量
        phi = 2*pi*d * cos(theta_lq(l, q_idx));
        a_vec = exp(1j * phi * (0:Na-1)');
        a_theta{l, q_idx} = a_vec;
    end
end

% 计算感知波束 R_opt（加权平均策略，与initial.m一致）
for m = 1:M
    for n = 1:N
        % 对所有感知点导向矢量做加权平均
        a_combined = zeros(Na, 1);
        weights = zeros(Q, 1);
        for q_idx = 1:Q
            % 权重为距离倒数平方
            weights(q_idx) = 1 / (d_lq(m, q_idx)^2 + 1e-6);
            a_combined = a_combined + weights(q_idx) * a_theta{m, q_idx};
        end
        
        if norm(a_combined) > 1e-6
            a_combined = a_combined / norm(a_combined);
        else
            a_combined = ones(Na, 1) / sqrt(Na);
        end
        
        % 使用动态分配的感知功率
        power_sensing = Pmax * eta_opt(m,n);
        R_opt{m,1,n} = power_sensing * (a_combined * a_combined');
    end
end

%% 5. 绘制优化后的感知功率热力图（参考initial.m布局）
fprintf('🎨 绘制AO优化后的感知功率热力图...\n');

% 热力图参数
area_size = 400;
grid_resolution = 5; % m
x_range = 0:grid_resolution:area_size;
y_range = 0:grid_resolution:area_size;
[X_grid, Y_grid] = meshgrid(x_range, y_range);
power_map_opt = zeros(size(X_grid));

% 选择可视化的时隙（可以是中间时隙，展示稳态性能）
n_vis = round(N/2);  % 选择中间时隙

% 计算每个网格点的感知功率
for i = 1:size(X_grid, 1)
    for j = 1:size(X_grid, 2)
        grid_point = [X_grid(i,j), Y_grid(i,j)];
        power_at_point = 0;
        for m = 1:M
            % 获取该GBS在时隙n_vis的总发射协方差
            X_m_nvis = zeros(Na, Na);
            for k = 1:K
                X_m_nvis = X_m_nvis + W_opt{m, k, n_vis};
            end
            X_m_nvis = X_m_nvis + R_opt{m, 1, n_vis};
            
            % 计算导向矢量
            dx = grid_point(1) - u(m,1);
            dy = grid_point(2) - u(m,2);
            dist_3d = sqrt(dx^2 + dy^2 + H_sense^2);
            theta = acos(H_sense / dist_3d);
            phi = 2*pi*d*cos(theta);
            a_vec = exp(1j * phi * (0:Na-1)');
            
            % 功率计算（与initial.m一致）
            power_at_point = power_at_point + real(a_vec' * X_m_nvis * a_vec) / (dist_3d^2 + 1e-12);
        end
        power_map_opt(i, j) = power_at_point;
    end
end

% 转换为dB
power_map_db_opt = 10*log10(max(power_map_opt, 1e-12));

% 创建图形
figure('Position', [100, 100, 1200, 800]);
pcolor(X_grid, Y_grid, power_map_db_opt);
shading interp;
hold on;

% 绘制GBS位置
plot(u(:,1), u(:,2), 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 2, 'DisplayName', 'GBS');

% 绘制感知点
plot(v(:,1), v(:,2), 'wx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Sensing Points');

% 绘制感知区域边界
cx = 200; cy = 200; width_x = 100; width_y = 50;
x_min = cx - width_x/2; y_min = cy - width_y/2;
rectangle('Position', [x_min, y_min, width_x, width_y], 'EdgeColor', 'w', 'LineStyle', '--', 'LineWidth', 2);
plot(NaN,NaN,'w--', 'LineWidth', 2, 'DisplayName', 'Sensing Area');

% 🚀 关键：叠加轨迹对比（优化前 vs 优化后）
% 初始轨迹（虚线）
for k = 1:K
    if k == 1
        plot(squeeze(q_init(k,1,:)), squeeze(q_init(k,2,:)), '--', 'Color', [0.3 0.3 1], 'LineWidth', 2, 'DisplayName', 'UAV初始轨迹');
    else
        plot(squeeze(q_init(k,1,:)), squeeze(q_init(k,2,:)), '--', 'Color', [0.3 0.3 1], 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end

% 优化后轨迹（实线，更粗）
for k = 1:K
    if k == 1
        plot(squeeze(q_opt(k,1,:)), squeeze(q_opt(k,2,:)), '-', 'Color', [1 0.3 0.3], 'LineWidth', 3, 'DisplayName', 'UAV优化轨迹');
    else
        plot(squeeze(q_opt(k,1,:)), squeeze(q_opt(k,2,:)), '-', 'Color', [1 0.3 0.3], 'LineWidth', 3, 'HandleVisibility', 'off');
    end
end

% 起点和终点标记
plot(squeeze(q_init(:,1,1)), squeeze(q_init(:,2,1)), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(squeeze(q_init(:,1,N)), squeeze(q_init(:,2,N)), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 图形设置
title(sprintf('AO优化后感知功率分布热力图 (时隙 n=%d, dBW)', n_vis), 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X坐标 (m)', 'FontSize', 12);
ylabel('Y坐标 (m)', 'FontSize', 12);
legend('show', 'Location', 'eastoutside');
axis equal; axis([0 400 0 400]);
grid on;

% 颜色条设置（参考initial.m）
valid_vals = power_map_db_opt(isfinite(power_map_db_opt) & power_map_opt > 1e-8);
if ~isempty(valid_vals)
    low_db = prctile(valid_vals, 20);
    high_db = prctile(valid_vals, 95);
    if high_db <= low_db
        low_db = min(valid_vals);
        high_db = max(valid_vals);
    end
    if high_db > low_db
        caxis([low_db, high_db]);
    end
end
c = colorbar;
ylabel(c, '接收功率 (dBW)', 'FontSize', 12);

%% 6. 计算并显示性能对比统计
fprintf('\n📊 AO优化前后性能对比：\n');

% 计算优化后的感知功率
zeta_qn_opt = zeros(Q, N);
for n = 1:N
    for q_idx = 1:Q
        power_at_q = 0;
        for m = 1:M
            % 获取该GBS在该时隙的总发射协方差矩阵
            X_m = zeros(Na, Na);
            for i = 1:K
                X_m = X_m + W_opt{m, i, n};
            end
            X_m = X_m + R_opt{m,1,n};
            
            % 获取导向矢量
            a_vec = a_theta{m, q_idx};
            d_lq_val = d_lq(m, q_idx);
            
            % 计算功率贡献
            power_at_q = power_at_q + (a_vec' * X_m * a_vec) / (d_lq_val^2);
        end
        zeta_qn_opt(q_idx, n) = real(power_at_q);
    end
end

min_sensing_opt = min(zeta_qn_opt(:));
mean_sensing_opt = mean(zeta_qn_opt(:));

% 如果存在初始数据，进行对比
if exist('zeta_qn', 'var')
    min_sensing_init = min(zeta_qn(:));
    mean_sensing_init = mean(zeta_qn(:));
    
    fprintf('  最小感知功率: %.4e W → %.4e W (提升 %.1f%%)\n', ...
        min_sensing_init, min_sensing_opt, (min_sensing_opt/min_sensing_init-1)*100);
    fprintf('  平均感知功率: %.4e W → %.4e W (提升 %.1f%%)\n', ...
        mean_sensing_init, mean_sensing_opt, (mean_sensing_opt/mean_sensing_init-1)*100);
else
    fprintf('  优化后最小感知功率: %.4e W (%.2f dBW)\n', min_sensing_opt, 10*log10(min_sensing_opt));
    fprintf('  优化后平均感知功率: %.4e W (%.2f dBW)\n', mean_sensing_opt, 10*log10(mean_sensing_opt));
end

% 计算轨迹变化统计
total_displacement = 0;
for k = 1:K
    for n = 1:N
        displacement = norm(squeeze(q_opt(k,:,n)) - squeeze(q_init(k,:,n)));
        total_displacement = total_displacement + displacement;
    end
end
avg_displacement = total_displacement / (K * N);
fprintf('  平均轨迹偏移: %.2f m\n', avg_displacement);

fprintf('\n✅ AO优化后热力图生成完成！\n');
fprintf('📍 图中展示了优化前轨迹(蓝虚线)与优化后轨迹(红实线)的对比\n');
fprintf('🎯 热力图显示了基于优化轨迹重新计算的感知功率分布\n');
