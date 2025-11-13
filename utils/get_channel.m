function h = get_channel(m, k, n, u, q_traj, H, kappa, t, Na)
    % 计算 h_{m,k}[n]
    qk_2d = squeeze(q_traj(k, :, n)); % 提取UAV k在时隙n的2D位置
    um = u(m, :);
    
    dx = qk_2d(1) - um(1);
    dy = qk_2d(2) - um(2);
    
    % 修复数组索引：H可能是标量或向量
    if length(H) == 1
        H_k = H;  % 标量高度
    else
        H_k = H(k);  % 向量高度
    end
    
    dist_3d = sqrt(dx^2 + dy^2 + H_k^2);
    
    % 路径损耗
    beta = kappa / dist_3d^2;
    
    % AoD (到达角)
    cos_theta = H_k / dist_3d;
    
    % 流体天线导向矢量（使用位置向量 t，归一化以波长为单位）
    % g = exp(j*2*pi*t*cos(theta))
    if isscalar(t)
        % 兼容旧接口：当传入标量间距 d 时，生成等间距位置向量
        d_lambda = t;
        t_vec = (0:Na-1)' * d_lambda;
    else
        t_vec = t(:);
        if numel(t_vec) ~= Na
            error('get_channel:InvalidT', '位置向量 t 的长度应为 Na');
        end
    end
    g = exp(1j * 2*pi * t_vec * cos_theta);
    
    h = sqrt(beta) * g;
end