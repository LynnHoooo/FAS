% 此文件已废弃 - 所有速率计算现在统一使用 calculate_master_rate_function
% 
% 原因：为了解决系统中多个不一致的速率计算函数导致的数学模型冲突
% 
% 请使用：
% [rate, ~] = calculate_master_rate_function(W_mkn, R_mkn, h_mkn_precomputed, alpha_mkn, sigma2, M, K, N);

function rate = compute_sum_rate(varargin)
    error('compute_sum_rate 已废弃。请使用 calculate_master_rate_function 替代。');
end
