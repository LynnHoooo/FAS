%% ==================================================================
%  plot_beamforming_heatmap.m
%  功能: 基于优化后的通信波束 W_opt 与感知波束 R_opt, 在二维平面
%        上生成波束增益/照射功率热力图 (默认读取 beamforming_feasibility_results.mat)
%% ==================================================================

clear; clc; close all;

%% 参数设定
data_file = 'beamforming_feasibility_results.mat';
load_required = true;

n_idx = 40;      % 选择需要展示的时隙

x_range = [0, 400];
y_range = [0, 400];
grid_step = 2;

%% 读取数据
if load_required
    if ~isfile(data_file)
        error('找不到数据文件 %s, 请先运行优化脚本生成。', data_file);
    end
    data = load(data_file);
    if isfield(data, 'W_test') && ~isempty(data.W_test)
        W_source = data.W_test;
    elseif isfield(data, 'W_opt')
        W_source = data.W_opt;
    else
        error('数据文件中未找到 W_test/W_opt');
    end
    if isfield(data, 'R_test') && ~isempty(data.R_test)
        R_source = data.R_test;
    elseif isfield(data, 'R_opt')
        R_source = data.R_opt;
    else
        error('数据文件中未找到 R_test/R_opt');
    end
    if isfield(data, 'u'), u = data.u; elseif ~exist('u','var'), error('缺少基站位置 u'); end
    if isfield(data, 'H'), H = data.H; elseif ~exist('H','var'), error('缺少高度 H'); end
    if isfield(data, 'kappa'), kappa = data.kappa; elseif ~exist('kappa','var'), error('缺少路径损耗常数 kappa'); end
    if isfield(data, 'H_sense'), H_sense = data.H_sense; elseif ~exist('H_sense','var'), error('缺少感知高度 H_sense'); end
    if isfield(data, 'v'), v = data.v; end
else
    if ~exist('W_opt','var') || ~exist('R_opt','var')
        error('工作区中找不到 W_opt / R_opt，请确认已执行优化。');
    end
    W_source = W_opt;
    R_source = R_opt;
    if ~exist('u','var') || ~exist('H','var') || ~exist('kappa','var')
        error('使用工作区数据时需要存在 u/H/kappa。');
    end
end

Na = [];
for m = 1:size(W_source,1)
    for k = 1:size(W_source,2)
        if ~isempty(W_source{m,k,n_idx})
            Na = size(W_source{m,k,n_idx}, 1);
            break;
        end
    end
    if ~isempty(Na)
        break;
    end
end
if isempty(Na)
    % 尝试使用感知协方差矩阵判定天线数
    for m = 1:size(R_source,1)
        if ~isempty(R_source{m,1,n_idx})
            Na = size(R_source{m,1,n_idx}, 1);
            break;
        end
    end
end
if isempty(Na)
    % 如果所有优化结果为空，尝试回退到初始化结果
    if exist('data','var') && isfield(data,'W_init')
        W_source = data.W_init;
        if isfield(data,'R_init')
            R_source = data.R_init;
        end
        for m = 1:size(W_source,1)
            for k = 1:size(W_source,2)
                if ~isempty(W_source{m,k,n_idx})
                    Na = size(W_source{m,k,n_idx},1);
                    break;
                end
            end
            if ~isempty(Na)
                break;
            end
        end
    end
end
if isempty(Na)
    error('无法在时隙 %d 的 W/R 中找到有效的阵列维度。', n_idx);
end
x_vals = x_range(1):grid_step:x_range(2);
y_vals = y_range(1):grid_step:y_range(2);
Z = zeros(numel(y_vals), numel(x_vals));

%% 计算网格上的照射功率
for ix = 1:numel(x_vals)
    for iy = 1:numel(y_vals)
        px = x_vals(ix);
        py = y_vals(iy);
        total_power = 0;

        for m = 1:size(W_source,1)
            X_cov = zeros(Na, Na);
            for k = 1:size(W_source,2)
                if ~isempty(W_source{m,k,n_idx})
                    X_cov = X_cov + W_source{m,k,n_idx};
                end
            end
            if m <= size(R_source,1) && ~isempty(R_source{m,1,n_idx})
                X_cov = X_cov + R_source{m,1,n_idx};
            end

            dx = px - u(m,1);
            dy = py - u(m,2);
            dist_sq = dx^2 + dy^2 + H_sense^2;
            if dist_sq <= 1e-6
                continue;
            end

            dist = sqrt(dist_sq);
            cos_theta = H_sense / dist;
            a_vec = exp(1j * 2 * pi * 0.5 * cos_theta * (0:Na-1)');
            % 与 initial.m 中计算方式保持一致，便于对比
            total_power = total_power + real(a_vec' * X_cov * a_vec) / (dist_sq + 1e-12);
        end

        Z(iy, ix) = max(total_power, 0);
    end
end

power_map_db = 10 * log10(Z + 1e-12);
valid_vals = power_map_db(isfinite(power_map_db));
if isempty(valid_vals)
    error('无法计算有效的波束功率。');
end
lower = prctile(valid_vals, 10);
upper = prctile(valid_vals, 95);

%% 绘图
figure('Position',[100 100 900 720]);
ax = axes;
% 使用 pcolor 生成平滑热图
h_heat = pcolor(ax, x_vals, y_vals, power_map_db);
shading interp;
axis equal tight;
colormap(ax, parula);
colorbar;
clim([lower upper]);
title(sprintf('优化后波束照射功率热力图 (时隙 %d)', n_idx));
xlabel('X (m)');
ylabel('Y (m)');
hold on;
set(h_heat, 'DisplayName', '优化后功率');
% 绘制基站
h_gbs = plot(u(:,1), u(:,2), 'ks', 'MarkerFaceColor','w', 'MarkerSize',8);
set(h_gbs, 'DisplayName', 'GBS 位置');
text(u(:,1)+3, u(:,2)+3, arrayfun(@(idx) sprintf('GBS%d', idx), 1:size(u,1), 'UniformOutput', false));
%
if exist('v','var') && ~isempty(v)
    vx = [min(v(:,1)), max(v(:,1)), max(v(:,1)), min(v(:,1)), min(v(:,1))];
    vy = [min(v(:,2)), min(v(:,2)), max(v(:,2)), max(v(:,2)), min(v(:,2))];
    h_area = plot(ax, vx, vy, 'w--', 'LineWidth', 1.5);
    set(h_area, 'DisplayName', '感知区域');
    h_sense_pts = scatter(v(:,1), v(:,2), 50, 'wo', 'filled', 'MarkerEdgeColor', 'k');
    set(h_sense_pts, 'DisplayName', '感知采样点');
end
hold off;

legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal');

fprintf('完成热力图绘制: 时隙 %d\n', n_idx);


