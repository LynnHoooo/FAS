function min_sensing_power = compute_sensing_power(alpha_mkn, W_mkn, R_mkn, u, v, M, Q, N, Na, kappa, t_positions, H_sense)
% COMPUTE_SENSING_POWER - 计算最小感知功率（FAS版本）
%
% 输入:
%   alpha_mkn    : M×K×N, 关联矩阵
%   W_mkn        : cell{M,K,N}, 通信波束矩阵
%   R_mkn        : cell{M,1,N}, 感知波束矩阵
%   u            : M×2, GBS位置
%   v            : Q×2, 感知点位置
%   M,Q,N,Na     : 系统参数
%   kappa        : 路径损耗因子
%   t_positions  : cell{M,1}, FAS位置向量
%   H_sense      : 感知高度
%
% 输出:
%   min_sensing_power : 最小感知功率

    K = size(alpha_mkn, 2);
    zeta_qn = zeros(Q, N);

    for n = 1:N
        for q_idx = 1:Q
            total_power = 0;
            for m = 1:M
                composite = zeros(Na, Na);
                for k = 1:K
                    if ~isempty(W_mkn{m, k, n})
                        composite = composite + W_mkn{m, k, n};
                    end
                end
                if ~isempty(R_mkn{m, 1, n})
                    composite = composite + R_mkn{m, 1, n};
                end

                dx = v(q_idx, 1) - u(m, 1);
                dy = v(q_idx, 2) - u(m, 2);
                dist = sqrt(dx^2 + dy^2 + H_sense^2);

                % 归一化功率
                path_loss = 1 / (dist^2);

                cos_theta = H_sense / dist;
                % FAS导向矢量：使用当前GBS m的天线位置向量
                steering = exp(1j * t_positions{m} * 2 * pi * cos_theta);

                total_power = total_power + path_loss * real(steering' * composite * steering);
            end

            zeta_qn(q_idx, n) = total_power;
        end
    end

    min_sensing_power = min(zeta_qn(:));
end
