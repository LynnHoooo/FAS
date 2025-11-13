function generate_sensing_heatmap_initial_style(t_positions, alpha_mkn, W_mkn, R_mkn, ...
    u, v, M, Q, N, Na, kappa, H_sense, title_suffix)
% GENERATE_SENSING_HEATMAP_INITIAL_STYLE - 生成初始化风格的感知功率热力图
%
% 参考 initial.m 中的热力图绘制方式，使用相同的网格计算和显示方法
%
% 输入参数：
%   t_positions : cell数组，每个GBS的天线位置
%   alpha_mkn   : M×K×N，关联矩阵
%   W_mkn       : M×K×N cell，通信波束矩阵
%   R_mkn       : M×1×N cell，感知波束矩阵
%   u           : M×2，GBS位置
%   v           : Q×2，感知点位置
%   M, Q, N, Na : 系统参数
%   kappa       : 信道增益常数
%   H_sense     : 感知高度
%   title_suffix: 标题后缀（如'ULA基准'或'FAS优化'）

    try
        % 创建新图窗
        figure;
        
        % 网格设置（与initial.m保持一致）
        area_size = 400;  % 区域大小 (m)
        grid_resolution = 5; % 网格分辨率 (m)
        x_range = 0:grid_resolution:area_size;
        y_range = 0:grid_resolution:area_size;
        [X_grid, Y_grid] = meshgrid(x_range, y_range);
        power_map = zeros(size(X_grid));
        
        % 使用第一个时隙进行热力图计算
        n = 1;
        
        % 遍历网格点计算感知功率
        for i = 1:size(X_grid, 1)
            for j = 1:size(X_grid, 2)
                grid_point = [X_grid(i,j), Y_grid(i,j)];
                power_at_point = 0;
                
                % 遍历所有GBS
                for m = 1:M
                    % 计算该GBS在时隙n的总发射协方差矩阵
                    X_m_total = zeros(Na, Na);
                    
                    % 通信波束贡献
                    for k = 1:size(W_mkn, 2)  % K个用户
                        if ~isempty(W_mkn{m,k,n})
                            X_m_total = X_m_total + W_mkn{m,k,n};
                        end
                    end
                    
                    % 感知波束贡献
                    if ~isempty(R_mkn{m,1,n})
                        X_m_total = X_m_total + R_mkn{m,1,n};
                    end
                    
                    % 计算导向矢量（与initial.m完全一致）
                    dx = grid_point(1) - u(m,1);
                    dy = grid_point(2) - u(m,2);
                    dist_3d = sqrt(dx^2 + dy^2 + H_sense^2);
                    theta = acos(H_sense / dist_3d);
                    
                    % 流体天线导向矢量
                    if iscell(t_positions)
                        t_m = t_positions{m};
                    else
                        t_m = t_positions;  % 如果是向量形式
                    end
                    a_vec = exp(1j * 2*pi * t_m * cos(theta));
                    
                    % 功率计算（与initial.m保持一致）
                    power_contribution = real(a_vec' * X_m_total * a_vec) / (dist_3d^2 + 1e-12);
                    power_at_point = power_at_point + power_contribution;
                end
                
                power_map(i, j) = power_at_point;
            end
        end
        
        % 转换为dB（与initial.m保持一致）
        power_map_db = 10*log10(max(power_map, 1e-12));
        
        % 绘制热力图（与initial.m保持一致）
        pcolor(X_grid, Y_grid, power_map_db);
        shading interp;
        colorbar;
        hold on;
        
        % 添加GBS位置标记
        plot(u(:,1), u(:,2), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'GBS');
        
        % 添加感知点标记
        if ~isempty(v)
            plot(v(:,1), v(:,2), 'wx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Sensing Points');
        end
        
        % 添加感知区域边界（如果定义了）
        if exist('x_min', 'var') && exist('y_min', 'var') && exist('width_x', 'var') && exist('width_y', 'var')
            rectangle('Position', [x_min, y_min, width_x, width_y], 'EdgeColor', 'w', 'LineStyle', '--', 'LineWidth', 2);
            plot(NaN,NaN,'w--', 'DisplayName', 'Sensing Area');
        end
        
        % 设置标题和标签
        title(sprintf('感知功率分布热力图 - %s (时隙 n=1, dBW)', title_suffix));
        xlabel('X (m)'); 
        ylabel('Y (m)');
        legend('show');
        axis equal; 
        axis([0 area_size 0 area_size]);
        
        % 设置颜色范围（与initial.m保持一致）
        valid_vals = power_map_db(isfinite(power_map_db) & power_map > 1e-8);
        if ~isempty(valid_vals)
            caxis([min(valid_vals)-5, max(valid_vals)+5]);
        end
        
        % 保存图像
        filename = sprintf('sensing_heatmap_%s.png', strrep(title_suffix, ' ', '_'));
        saveas(gcf, filename);
        
        fprintf('   ✅ 热力图已保存: %s\n', filename);
        
    catch ME
        fprintf('   ❌ 热力图生成失败: %s\n', ME.message);
        rethrow(ME);
    end
end
