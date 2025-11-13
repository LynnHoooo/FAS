function generate_sensing_heatmap_optimized(t_positions, alpha_mkn, W_mkn, R_mkn, ...
    u, v, M, Q, N, Na, kappa, H_sense, title_suffix)
% 生成优化后的感知功率热力图
% 
% 输入参数:
%   t_positions - cell数组，每个GBS的天线位置
%   alpha_mkn - 关联矩阵
%   W_mkn, R_mkn - 波束矩阵
%   u, v - 感知区域坐标
%   M, Q, N, Na - 系统参数
%   kappa - 信道参数
%   H_sense - 感知高度
%   title_suffix - 标题后缀

try
    fprintf('   正在生成%s感知功率热力图...\n', title_suffix);
    
    % 创建感知区域网格
    [U_grid, V_grid] = meshgrid(u, v);
    sensing_power_grid = zeros(size(U_grid));
    
    % 计算每个感知点的功率
    for i = 1:length(u)
        for j = 1:length(v)
            u_point = u(i);
            v_point = v(j);
            
            total_power = 0;
            
            % 遍历所有时隙
            for n = 1:N
                % 遍历所有GBS
                for m = 1:M
                    if ~isempty(R_mkn{m,1,n})
                        % 计算到感知点的距离和角度
                        dist_3d = sqrt(u_point^2 + v_point^2 + H_sense^2);
                        beta = kappa / dist_3d^2;
                        cos_theta = H_sense / dist_3d;
                        
                        % 计算感知信道向量
                        if m <= length(t_positions) && ~isempty(t_positions{m})
                            t_pos = t_positions{m};
                        else
                            t_pos = (0:Na-1)' * 0.5;  % 默认ULA位置
                        end
                        
                        g = exp(1j * 2*pi * t_pos * cos_theta);
                        h_sense = sqrt(beta) * g;
                        
                        % 计算感知功率
                        power = real(h_sense' * R_mkn{m,1,n} * h_sense);
                        total_power = total_power + power;
                    end
                end
            end
            
            sensing_power_grid(j, i) = total_power / N;  % 平均功率
        end
    end
    
    % 生成热力图
    figure('Name', sprintf('感知功率分布 - %s', title_suffix), 'Position', [100, 100, 800, 600]);
    
    % 转换为dBW
    sensing_power_dBW = 10*log10(max(sensing_power_grid, 1e-12));
    
    % 确保u和v是向量
    u_vec = u(:);
    v_vec = v(:);
    imagesc(u_vec, v_vec, sensing_power_dBW);
    colorbar;
    colormap('hot');
    
    xlabel('u (m)');
    ylabel('v (m)');
    title(sprintf('感知功率分布热力图 - %s', title_suffix));
    
    % 添加颜色条标签
    c = colorbar;
    c.Label.String = '感知功率 (dBW)';
    
    % 设置坐标轴
    axis xy;
    grid on;
    
    % 保存图像
    filename = sprintf('sensing_heatmap_%s.png', strrep(title_suffix, ' ', '_'));
    saveas(gcf, fullfile('plot', filename));
    
    fprintf('   ✅ 热力图已保存: %s\n', filename);
    
catch ME
    fprintf('   ❌ 热力图生成失败: %s\n', ME.message);
end

end
