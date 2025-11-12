function [SINR, rate] = compute_sinr(h_mkn, W_mkn, R_mkn, alpha_mkn, sigma2, m_serv, k, n, M, K)
%% 标准SINR和速率计算函数
% 功能: 统一的SINR计算，确保所有模块使用相同的物理模型
% 输入:
%   h_mkn: 信道矩阵 cell(M,K,N)
%   W_mkn: 通信波束矩阵 cell(M,K,N) 
%   R_mkn: 感知波束矩阵 cell(M,1,N)
%   alpha_mkn: 关联矩阵 (M,K,N)
%   sigma2: 噪声功率
%   m_serv: 服务GBS索引
%   k: UAV索引
%   n: 时隙索引
%   M, K: GBS和UAV数量
% 输出:
%   SINR: 信干噪比
%   rate: 通信速率 log2(1+SINR) bps/Hz

% 获取服务信道
h_serv = h_mkn{m_serv, k, n};

% 计算信号功率 (分子)
signal_power = real(h_serv' * W_mkn{m_serv, k, n} * h_serv);

% 计算干扰功率 (分母)
interference = 0;

% 1. 来自其他UAV通信信号的干扰
for m = 1:M
    for i = 1:K
        if m == m_serv && i == k
            continue;  % 跳过自己的信号
        end
        % 使用UAV k到GBS m的信道计算干扰
        h_m_k_n = h_mkn{m, k, n};
        interference = interference + real(h_m_k_n' * W_mkn{m, i, n} * h_m_k_n);
    end
end

% 2. 来自所有GBS感知信号的干扰 (物理正确的模型)
for m = 1:M
    h_m_k_n = h_mkn{m, k, n};
    interference = interference + real(h_m_k_n' * R_mkn{m, 1, n} * h_m_k_n);
end

% 计算SINR和速率
total_noise_interference = interference + sigma2;
if total_noise_interference <= 1e-12 || signal_power <= 1e-12
    SINR = 0;
    rate = 0;
else
    SINR = signal_power / total_noise_interference;
    rate = log2(1 + max(SINR, 1e-12));
end

end
