%% ===================================================================
%            感知-通信性能权衡曲线生成脚本
%  功能: 扫描一系列感知阈值Gamma，为每个值运行完整的AO算法，
%        并记录最终的和速率，以生成性能权衡曲线。
%  用法: 直接运行此脚本。这将需要较长时间。
% ===================================================================

clear; clc; close all;

%% 1. 定义参数扫描范围
gamma_list_dBW = -80 : 5 : -60; % 感知阈值范围 (dBW), 使用较大步长以加速
gamma_list = 10.^(gamma_list_dBW / 10); % 转换为线性单位 (W)
num_gammas = length(gamma_list);

fprintf('===== 开始ISAC性能权衡扫描 =====\n');
fprintf('将测试 %d 个Gamma值 (从 %.1f dBW 到 %.1f dBW)\n', ...
    num_gammas, gamma_list_dBW(1), gamma_list_dBW(end));

%% 2. 运行一次系统初始化
fprintf('\n步骤1: 运行系统初始化...\n');
try
    % 使用evalc来抑制initial.m的输出
    evalc('initial'); 
    fprintf('✅ 系统初始化成功。\n');
catch ME
    error('系统初始化失败: %s', ME.message);
end

% 将工作区中的所有变量打包到一个结构体中，以便传递给并行工作者
fprintf('步骤2: 打包参数以用于并行计算...\n');
vars = whos;
p = struct();
for i = 1:length(vars)
    % 排除不需要的变量
    if ~ismember(vars(i).name, {'p', 'vars', 'i', 'gamma_list_dBW', 'gamma_list', 'num_gammas'})
        p.(vars(i).name) = eval(vars(i).name);
    end
end
fprintf('✅ 参数打包完成。\n');

%% 3. 设置并行池并执行扫描
fprintf('步骤3: 启动并行池并开始扫描 (这可能需要一些时间)...\n');

% 检查并行计算工具箱是否存在
if isempty(ver('parallel'))
    use_parfor = false;
    warning('未找到并行计算工具箱，将使用常规 for 循环（速度较慢）。');
else
    use_parfor = true;
    if isempty(gcp('nocreate'))
        parpool(); % 启动并行池
    end
end

% 初始化结果存储数组
avg_sum_rate_list = zeros(num_gammas, 1);
final_sensing_list = zeros(num_gammas, 1);
tic; % 开始计时

if use_parfor
    % --- 并行版本 ---
    parfor i = 1:num_gammas
        fprintf('并行任务: 正在运行 Gamma = %.1f dBW...\n', gamma_list_dBW(i));
        
        % 为当前循环创建一个参数副本并修改Gamma
        p_local = p;
        p_local.Gamma = gamma_list(i);
        
        % 运行AO算法
        [rate, sensing] = main_AO_algorithm(p_local);
        
        avg_sum_rate_list(i) = rate;
        final_sensing_list(i) = sensing;
    end
else
    % --- 串行版本 (备用) ---
    for i = 1:num_gammas
        fprintf('串行任务: 正在运行 Gamma = %.1f dBW (%d/%d)...\n', gamma_list_dBW(i), i, num_gammas);
        
        p_local = p;
        p_local.Gamma = gamma_list(i);
        
        [rate, sensing] = main_AO_algorithm(p_local);
        
        avg_sum_rate_list(i) = rate;
        final_sensing_list(i) = sensing;
    end
end

elapsed_time = toc;
fprintf('\n✅ 扫描完成！总耗时: %.2f 秒 (平均每次 %.2f 秒)。\n', elapsed_time, elapsed_time / num_gammas);

%% 4. 保存结果
fprintf('步骤4: 保存扫描结果...\n');
save('gamma_tradeoff_results.mat', 'gamma_list_dBW', 'avg_sum_rate_list', 'final_sensing_list');
fprintf('✅ 结果已保存到 gamma_tradeoff_results.mat\n');
