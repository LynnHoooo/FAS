function [R_sum_total, power_breakdown] = calculate_master_rate_function(W_mkn, R_mkn, h_mkn_precomputed, alpha_mkn, sigma2, M, K, N)
% CALCULATE_MASTER_RATE_FUNCTION - 唯一真理函数：计算系统总和速率
%
% 这是代码库中唯一允许计算"真实速率"的函数
% 所有其他速率计算必须与此函数的干扰模型完全一致
%
% 输入参数：
%   W_mkn              : M×K×N cell，通信波束矩阵
%   R_mkn              : M×1×N cell，感知波束矩阵  
%   h_mkn_precomputed  : M×K×N cell，预计算信道矩阵
%   alpha_mkn          : M×K×N，关联矩阵
%   sigma2             : 标量，噪声功率
%   M, K, N            : 系统参数
%
% 输出参数：
%   R_sum_total        : 标量，系统总和速率 (bps/Hz)
%   SINR_matrix        : K×N，每个用户每个时隙的SINR

    % 初始化
    R_sum_total = 0;
    
    % 初始化功率分解结构（支持两层SCA）
    power_breakdown.signal = cell(M, K, N);
    power_breakdown.comm_interference = cell(M, K, N);
    power_breakdown.sense_interference = cell(M, K, N);
    
    % 遍历所有时隙和用户
    for n = 1:N
        for k = 1:K
            % 找到服务该用户的GBS
            m_serv = find(alpha_mkn(:,k,n) == 1, 1);
            
            if ~isempty(m_serv) && ~isempty(W_mkn{m_serv,k,n})
                % 获取信道
                h_mk = h_mkn_precomputed{m_serv,k,n};
                
                % 信号功率
                signal_power = real(h_mk' * W_mkn{m_serv,k,n} * h_mk);
                power_breakdown.signal{m_serv,k,n} = signal_power;
                
                % === 干扰功率计算（唯一真理模型） ===
                
                % 1. 通信干扰：来自其他GBS和用户的通信信号
                comm_interference = 0;
                for l = 1:M
                    for i = 1:K
                        if ~(l == m_serv && i == k) && alpha_mkn(l,i,n) == 1
                            h_lk = h_mkn_precomputed{l,k,n};
                            comm_interference = comm_interference + real(h_lk' * W_mkn{l,i,n} * h_lk);
                        end
                    end
                end
                power_breakdown.comm_interference{m_serv,k,n} = comm_interference;
                
                % 2. 感知干扰：来自所有GBS的感知信号
                sense_interference = 0;
                for l = 1:M
                    h_lk = h_mkn_precomputed{l,k,n};
                    sense_interference = sense_interference + real(h_lk' * R_mkn{l,1,n} * h_lk);
                end
                power_breakdown.sense_interference{m_serv,k,n} = sense_interference;
                
                % 总干扰
                total_interference = comm_interference + sense_interference;
                
                % SINR和速率（基于DC形式）
                SINR = signal_power / (total_interference + sigma2);
                R_sum_total = R_sum_total + log2(1 + max(SINR, 1e-12));
            end
        end
    end
end
