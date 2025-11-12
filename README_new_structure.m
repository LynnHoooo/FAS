%% ============================================================================
%  README: 网络化ISAC系统 - 完整工程结构说明
%  项目: Networked ISAC for Low-Altitude Economy
%  论文: Gaoyuan Cheng et al., arXiv:2405.07568
%  更新时间: 2025-11-05
%  目标: 代码组织清晰，功能分离明确
%% ============================================================================

%% 📁 完整文件夹结构
% fuxian/
% ├── setup_paths.m                          # 路径设置脚本（必须首先运行）
% ├── README_new_structure.m                 # 本文件：工程结构说明
% ├── test_ao_convergence.m                  # 完整AO算法收敛测试脚本
% │
% ├── core/                                  # 🔥 核心算法模块
% │   ├── initial.m                          # 系统初始化：设置参数、位置、信道、感知点等
% │   ├── main_AO_algorithm.m                # 主AO算法：交替优化主循环
% │   ├── optimize_association.m             # 关联优化：优化UAV-GBS关联策略
% │   ├── optimize_beamforming.m             # 波束优化：优化通信和感知波束成形
% │   └── optimize_trajectory_SCA_TR.m       # 轨迹优化：使用SCA+信任域方法优化UAV轨迹
% │
% ├── test/                                  # 🧪 测试脚本模块
% │   ├── test_ao_convergence_experiment.m   # AO收敛实验：完整AO算法收敛分析，支持断点续传
% │   ├── test_beamforming_feasibility.m     # 波束优化可行性测试：单独验证波束优化功能
% │   ├── test_single_trajectory_optimization.m  # 单次轨迹优化测试：仅测试轨迹优化子问题
% │   ├── test_trajectory_from_beamforming.m # 轨迹优化调试：利用波束优化结果调试轨迹优化
% │   ├── test_run_main_AO.m                 # 运行主AO算法：调用main_AO_algorithm的简单测试
% │   ├── test_optimized_performance.m       # 优化性能测试：测试优化后的通信性能
% │   ├── test_ao_with_heatmap.m             # AO优化+热力图：运行完整AO并生成对比热力图
% │   ├── sweep_gamma_tradeoff.m             # Gamma参数扫描：扫描感知阈值生成性能权衡曲线
% │   └── process_existing_ao_results.m      # 处理已有结果：将ao_convergence_results.mat转换为绘图格式
% │
% ├── plot/                                  # 📊 绘图脚本模块
% │   ├── plot_ao_convergence.m              # 简单收敛图：快速绘制通信和感知性能收敛曲线
% │   ├── plot_ao_convergence_detailed.m     # 详细收敛分析：9个子图的完整收敛分析
% │   ├── plot_ao_optimization_heatmap.m     # 优化热力图：AO优化后的感知功率分布热力图
% │   ├── plot_beamforming_heatmap.m         # 波束热力图：基于优化波束的波束增益热力图
% │   └── plot_gamma_tradeoff.m              # Gamma权衡图：感知阈值-通信速率权衡曲线
% │
% ├── utils/                                 # 🔧 工具函数模块
% │   ├── get_channel.m                      # 信道计算：计算GBS到UAV的信道矩阵
% │   ├── steering_vector.m                  # 导向矢量：生成均匀线阵（ULA）的阵列响应
% │   └── save_results.m                     # 保存结果：将工作区结果保存为.mat文件（旧格式）
% │
% └── data/                                  # 💾 数据文件目录
%     ├── ao_convergence_results.mat         # AO收敛结果：main_AO_algorithm保存的收敛数据
%     ├── ao_convergence_experiment_*.mat    # 实验数据：test_ao_convergence_experiment保存的完整实验数据
%     ├── ao_processed_results_*.mat         # 处理后数据：process_existing_ao_results处理后的数据
%     ├── beamforming_feasibility_results.mat # 波束优化结果：test_beamforming_feasibility保存的结果
%     ├── single_trajectory_results.mat      # 轨迹优化结果：test_single_trajectory_optimization保存的结果
%     ├── gamma_tradeoff_results.mat         # Gamma权衡结果：sweep_gamma_tradeoff保存的扫描结果
%     ├── ao_convergence_*.png               # 收敛图：plot_ao_convergence生成的图片
%     ├── ao_detailed_analysis_*.png         # 详细分析图：plot_ao_convergence_detailed生成的图片
%     └── ao_convergence_curves_*.png        # 收敛曲线图：plot_ao_convergence_detailed生成的曲线图

%% ============================================================================
%% 📄 核心算法模块 (core/) 详细说明
%% ============================================================================

%% 1. initial.m
% 【功能】系统初始化脚本
% 【作用】设置网络化ISAC系统的所有初始参数和配置
% 【包含内容】
%   - 系统参数：GBS数量M、UAV数量K、时隙数N、天线数Na、带宽B等
%   - 物理参数：工作频率fc、波长lambda、最大功率Pmax、噪声功率sigma2等
%   - 位置配置：GBS位置u、UAV初始/最终位置qI/qF、飞行高度H、感知点v等
%   - 信道计算：预计算所有GBS-UAV信道矩阵h_mkn
%   - 初始化策略：关联alpha_init、波束W_init和R_init的初始分配
%   - 可视化：绘制系统布局图（GBS、UAV轨迹、感知区域）
% 【输出变量】M, K, N, Q, Na, B, Pmax, sigma2, Gamma, kappa, d, u, v, H, 
%           q_traj, alpha_init, W_init, R_init, h_mkn等
% 【使用场景】所有测试脚本的第一步，必须先运行此脚本

%% 2. main_AO_algorithm.m
% 【功能】交替优化（AO）算法主函数
% 【作用】实现完整的AO优化循环，交替优化关联、波束和轨迹
% 【算法流程】
%   1. 关联优化：优化UAV-GBS关联策略
%   2. 波束优化：优化通信波束W和感知波束R
%   3. 轨迹优化：优化UAV飞行轨迹
%   4. 性能评估：计算和速率和感知功率
%   5. 收敛判断：检查是否满足收敛条件
% 【输入参数】p - 包含所有系统参数的结构体
% 【输出参数】
%   - final_sum_rate: 最终通信和速率 (bps/Hz)
%   - final_min_sensing: 最终最小感知功率 (W)
%   - ao_history: AO迭代历史数据（轨迹、关联、波束、性能等）
% 【特性】
%   - 支持自定义最大迭代次数和收敛容忍度
%   - 支持每次迭代后保存检查点（断点续传）
%   - 需要连续两次迭代满足收敛条件才认为收敛
% 【使用场景】完整的AO优化流程，被test_ao_convergence_experiment.m等调用

%% 3. optimize_association.m
% 【功能】关联优化函数
% 【作用】优化UAV与GBS的关联策略alpha_{m,k}[n]
% 【优化策略】
%   - 为每个UAV选择能提供最大速率的GBS
%   - 考虑干扰和噪声的影响
%   - 使用预计算的信道和当前波束矩阵评估速率
% 【输入参数】h_mkn_precomputed, W, R, Pmax, sigma2, M, K, N, Na
% 【输出参数】alpha - M×K×N关联矩阵
% 【使用场景】AO算法的子问题1，在main_AO_algorithm.m中调用

%% 4. optimize_beamforming.m
% 【功能】波束成形优化函数
% 【作用】优化通信波束W_{m,k}[n]和感知波束R_m[n]
% 【优化方法】
%   - 使用SCA（连续凸近似）+ SDR（半定松弛）方法
%   - 目标：最大化通信和速率
%   - 约束：感知功率阈值Gamma、功率限制Pmax
%   - 支持感知约束松弛（避免不可行）
%   - 秩一重构：使用主特征向量方法
% 【输入参数】h_mkn_precomputed, alpha_mkn, R_mn_init, W_mkn_init, 
%           Pmax, Gamma, sigma2, M, K, N, Na, Q, v, u, H_sense, kappa
% 【输出参数】
%   - W_opt: 优化后的通信波束 (cell{M,K,N})
%   - R_opt: 优化后的感知波束 (cell{M,1,N})
%   - obj_history: 目标函数历史值
% 【使用场景】AO算法的子问题2，在main_AO_algorithm.m中调用

%% 5. optimize_trajectory_SCA_TR.m
% 【功能】轨迹优化函数（SCA + 信任域方法）
% 【作用】优化UAV飞行轨迹q_k[n]
% 【优化方法】
%   - SCA（连续凸近似）：线性化非凸目标函数
%   - 信任域方法：限制每次迭代的轨迹变化范围
%   - 目标：最大化通信增益和感知增益
%   - 约束：速度限制Vmax、避碰距离Dmin、起点终点固定
% 【输入参数】q_init, h_mkn_gain, alpha_mkn, w_mkn, R_mkn, u, H, params,
%           gamma_min_SINR, Gamma, max_iter_SCA, tol, trust_region_in, verbose
% 【输出参数】
%   - q_opt: 优化后的轨迹 (K×2×N)
%   - trust_region_out: 输出信任域半径
% 【使用场景】AO算法的子问题3，在main_AO_algorithm.m中调用

%% ============================================================================
%% 🧪 测试脚本模块 (test/) 详细说明
%% ============================================================================

%% 1. test_ao_convergence_experiment.m
% 【功能】AO迭代收敛实验脚本（推荐使用）
% 【作用】运行完整的AO算法并保存详细的收敛数据用于后续分析
% 【特性】
%   - 收敛条件更严格：tolerance=1e-6，需要连续两次迭代满足条件
%   - 最大迭代次数：30次
%   - 每次迭代后自动保存检查点，支持断点续传
%   - 保存完整实验数据：experiment_results结构体
%   - 自动生成收敛图和CSV数据
% 【输出文件】
%   - data/ao_convergence_experiment_*.mat: 完整实验数据
%   - data/ao_convergence_data_*.csv: CSV格式收敛数据
%   - data/ao_convergence_plot_*.png: 收敛分析图
%   - data/ao_checkpoint_*.mat: 检查点文件（每次迭代保存）
% 【使用方法】
%   1. 正常运行：直接运行脚本
%   2. 从检查点恢复：设置resume_from_checkpoint=true，指定checkpoint_file
% 【使用场景】完整的AO算法收敛分析，生成论文用的收敛曲线

%% 2. test_beamforming_feasibility.m
% 【功能】波束优化可行性测试脚本
% 【作用】单独验证optimize_beamforming函数的可行性
% 【测试内容】
%   - 检查Gamma、Pmax、初始波束是否导致CVX不可行
%   - 验证波束优化能否成功收敛
%   - 测试感知约束是否满足
% 【输出文件】beamforming_feasibility_results.mat（包含W_test, R_test）
% 【使用场景】调试波束优化问题，验证参数设置是否合理

%% 3. test_single_trajectory_optimization.m
% 【功能】单次轨迹优化测试脚本
% 【作用】不运行完整AO循环，仅测试轨迹优化子问题
% 【测试步骤】
%   1. 运行initial.m获得基础参数和初始轨迹
%   2. 加载波束优化结果(W_test, R_test)
%   3. 调用optimize_trajectory_SCA_TR优化轨迹
%   4. 对比优化前后性能，绘制轨迹变化
% 【输出文件】single_trajectory_results.mat
% 【使用场景】单独调试轨迹优化，不运行完整的AO循环

%% 4. test_trajectory_from_beamforming.m
% 【功能】轨迹优化调试脚本
% 【作用】利用波束优化结果调试轨迹优化
% 【测试步骤】
%   1. 运行initial.m生成基础场景与直线路径
%   2. 加载beamforming_feasibility_results.mat中的W_test/R_test
%   3. 调用optimize_trajectory_SCA_TR优化无人机轨迹
%   4. 输出优化前后的通信和感知性能，并绘制轨迹对比
% 【使用场景】调试轨迹优化问题，使用已知的波束优化结果

%% 5. test_run_main_AO.m
% 【功能】运行主AO算法测试脚本
% 【作用】调用main_AO_algorithm的简单测试
% 【测试步骤】
%   1. 运行initial.m初始化系统参数
%   2. 组装参数结构体p
%   3. 调用main_AO_algorithm运行AO算法
%   4. 输出最终通信和感知性能
% 【使用场景】快速测试AO算法是否正常工作

%% 6. test_optimized_performance.m
% 【功能】优化性能测试脚本
% 【作用】测试优化后的通信性能
% 【测试内容】
%   - 运行完整的AO算法
%   - 计算最终的和速率和感知功率
%   - 输出性能指标
% 【使用场景】验证优化效果，检查性能是否提升

%% 7. test_ao_with_heatmap.m
% 【功能】AO优化+热力图生成脚本
% 【作用】运行完整的AO优化流程并生成对比热力图
% 【测试步骤】
%   1. 初始化系统参数
%   2. 运行轨迹优化
%   3. 重新计算W、R矩阵
%   4. 绘制优化前后对比热力图
% 【使用场景】可视化优化效果，生成优化前后对比图

%% 8. sweep_gamma_tradeoff.m
% 【功能】Gamma参数扫描脚本
% 【作用】扫描一系列感知阈值Gamma，生成性能权衡曲线
% 【扫描范围】默认-80dBW到-60dBW，步长5dB
% 【输出文件】gamma_tradeoff_results.mat（包含gamma_list_dBW, avg_sum_rate_list等）
% 【特性】
%   - 支持并行计算（parfor）
%   - 为每个Gamma值运行完整的AO算法
%   - 记录最终的和速率
% 【使用场景】研究感知-通信性能权衡，生成权衡曲线用于论文

%% 9. process_existing_ao_results.m
% 【功能】处理已有AO算法结果脚本
% 【作用】将ao_convergence_results.mat转换为plot_ao_convergence_detailed期望的格式
% 【处理步骤】
%   1. 加载ao_convergence_results.mat
%   2. 运行initial.m获取系统参数
%   3. 构建experiment_results结构体
%   4. 保存为ao_processed_results_*.mat
% 【输出文件】data/ao_processed_results_*.mat
% 【使用场景】处理旧格式的AO结果，转换为新格式用于绘图

%% ============================================================================
%% 📊 绘图脚本模块 (plot/) 详细说明
%% ============================================================================

%% 1. plot_ao_convergence.m
% 【功能】简单收敛图绘制函数
% 【作用】快速绘制通信和感知性能收敛曲线
% 【绘图内容】
%   - 双Y轴图：左轴显示通信和速率(bps/Hz)，右轴显示最小感知功率(W)
%   - 自动查找最新数据文件
%   - 支持新旧两种数据格式
% 【输入参数】data_file（可选），如果不提供则自动查找最新文件
% 【输出文件】data/ao_convergence_*.png
% 【使用场景】快速查看收敛情况，简单直观

%% 2. plot_ao_convergence_detailed.m
% 【功能】详细收敛分析绘图函数
% 【作用】生成9个子图的完整收敛分析
% 【绘图内容】
%   - 子图1：频谱效率收敛 (bps/Hz)
%   - 子图2：平均速率收敛 (Mbps)
%   - 子图3：感知性能收敛（含Gamma阈值线）
%   - 子图4：信任域调整策略
%   - 子图5：逐次性能提升
%   - 子图6：累积性能提升
%   - 子图7：通信-感知权衡轨迹
%   - 子图8：收敛速度分析
%   - 子图9：性能摘要文本
%   - 额外：主要性能指标收敛曲线图（2子图）
% 【输入参数】data_file（可选），自动查找最新experiment_results数据
% 【输出文件】
%   - data/ao_detailed_analysis_*.png: 详细分析图
%   - data/ao_convergence_curves_*.png: 收敛曲线图
% 【使用场景】完整的收敛分析，生成论文用的详细图表

%% 3. plot_ao_optimization_heatmap.m
% 【功能】AO优化后热力图绘制脚本
% 【作用】根据优化轨迹重新计算W、R，绘制感知功率热力图
% 【绘图内容】
%   - 感知功率分布热力图（优化后）
%   - 优化前后轨迹对比（蓝虚线vs红实线）
%   - GBS位置、感知点、感知区域标注
%   - 性能对比统计（最小/平均感知功率、轨迹偏移等）
% 【输入要求】需要工作区变量：q_opt, q_init, u, H, v, H_sense, kappa, sigma2, Pmax
% 【使用场景】可视化优化后的感知性能分布，对比优化前后效果

%% 4. plot_beamforming_heatmap.m
% 【功能】波束热力图绘制脚本
% 【作用】基于优化后的通信波束W_opt与感知波束R_opt生成波束增益热力图
% 【绘图内容】
%   - 波束增益/照射功率热力图
%   - 可选择特定时隙进行可视化
%   - GBS位置、UAV轨迹标注
% 【输入文件】beamforming_feasibility_results.mat（包含W_test, R_test）
% 【参数设置】n_idx（时隙索引）、x_range、y_range、grid_step
% 【使用场景】可视化波束成形效果，查看波束指向和覆盖范围

%% 5. plot_gamma_tradeoff.m
% 【功能】Gamma权衡曲线绘制函数
% 【作用】绘制感知阈值-通信速率权衡曲线
% 【绘图内容】
%   - X轴：感知功率阈值Gamma (dBW)
%   - Y轴：最终平均通信和速率 (bps/Hz)
%   - 标注趋势：宽松感知（高通信性能）vs严格感知（低通信性能）
% 【输入文件】gamma_tradeoff_results.mat（由sweep_gamma_tradeoff.m生成）
% 【输出文件】gamma_tradeoff.png, gamma_tradeoff.pdf
% 【使用场景】研究感知-通信性能权衡，生成论文用的权衡曲线

%% ============================================================================
%% 🔧 工具函数模块 (utils/) 详细说明
%% ============================================================================

%% 1. get_channel.m
% 【功能】信道计算函数
% 【作用】计算GBS m到UAV k在时隙n的信道矩阵h_{m,k}[n]
% 【计算方法】
%   - 路径损耗：beta = kappa / dist_3d^2
%   - AoD（到达角）：cos_theta = H(k) / dist_3d
%   - ULA导向矢量：g = exp(1j * 2*pi*d * cos_theta * (0:Na-1)')
%   - 信道：h = sqrt(beta) * g
% 【输入参数】m, k, n, u, q_traj, H, kappa, d, Na
% 【输出参数】h - Na×1信道向量
% 【使用场景】所有需要计算信道的地方，在initial.m和轨迹优化中调用

%% 2. steering_vector.m
% 【功能】导向矢量生成函数
% 【作用】生成均匀线阵（ULA）的阵列响应
% 【计算方法】
%   - 天线间距：d_lambda = 0.5（半波长）
%   - 导向矢量：a = exp(1j * 2*pi * d_lambda * n' * sin(theta))
% 【输入参数】Na（天线数）, theta（入射角，弧度）
% 【输出参数】a - Na×1导向矢量
% 【使用场景】计算阵列响应，在感知功率计算中使用

%% 3. save_results.m
% 【功能】保存结果脚本（旧格式）
% 【作用】将当前工作区中的AO算法结果保存到results_for_plotting.mat
% 【保存内容】q_current, W_current, R_current, alpha_current, 
%            sum_rate_history, min_sensing_power_history等
% 【使用场景】兼容旧格式，新代码建议使用test_ao_convergence_experiment.m

%% ============================================================================
%% 📋 其他文件说明
%% ============================================================================

%% 1. setup_paths.m
% 【功能】路径设置脚本
% 【作用】添加所有子文件夹到MATLAB路径，使跨文件夹调用成为可能
% 【添加路径】core/, test/, plot/, utils/, data/
% 【使用方法】在任何脚本开头运行：setup_paths;
% 【重要】必须先运行此脚本，否则无法调用其他文件夹的函数

%% 2. test_ao_convergence.m
% 【功能】完整AO算法收敛测试脚本（旧版本）
% 【作用】运行完整AO算法直到收敛，保存每次迭代的详细数据
% 【输出文件】data/ao_convergence_results.mat
% 【说明】推荐使用test_ao_convergence_experiment.m（功能更强大）

%% ============================================================================
%% 🚀 推荐工作流程
%% ============================================================================

%% 方法1: 完整AO收敛实验（推荐）
% 1. 设置路径
setup_paths;

% 2. 运行完整AO收敛实验
test_ao_convergence_experiment;

% 3. 生成详细收敛分析图
plot_ao_convergence_detailed;

%% 方法2: 分步调试
% 1. 设置路径
setup_paths;

% 2. 单独测试初始化
initial;

% 3. 单独测试波束优化
test_beamforming_feasibility;

% 4. 单独测试轨迹优化
test_single_trajectory_optimization;

% 5. 生成热力图
plot_ao_optimization_heatmap;

%% 方法3: Gamma参数扫描
% 1. 设置路径
setup_paths;

% 2. 运行Gamma扫描
sweep_gamma_tradeoff;

% 3. 绘制权衡曲线
plot_gamma_tradeoff;

%% ============================================================================
%% ⚠️ 重要提醒
%% ============================================================================

% 1. **必须先运行 setup_paths**
%    新结构下，所有脚本开头都要运行 setup_paths 才能找到其他文件夹的函数

% 2. **数据文件路径**
%    所有.mat文件保存在 data/ 文件夹中
%    所有图片文件保存在 data/ 文件夹中

% 3. **工作目录**
%    始终在 fuxian/ 主目录下工作
%    所有相对路径基于主目录

% 4. **跨文件夹调用**
%    setup_paths 确保所有文件夹都在MATLAB路径中
%    可以直接调用任意文件夹的函数

% 5. **数据格式**
%    新格式：experiment_results结构体（test_ao_convergence_experiment.m生成）
%    旧格式：直接变量（save_results.m生成，已不推荐）

% 6. **收敛条件**
%    - tolerance = 1e-6（更严格）
%    - 需要连续两次迭代满足条件才收敛
%    - 最大迭代次数 = 30

% 7. **断点续传**
%    test_ao_convergence_experiment.m支持从检查点恢复
%    每次迭代后自动保存检查点到 data/ao_checkpoint_*.mat

%% ============================================================================
%% 📊 当前工程状态
%% ============================================================================

fprintf('\n🎯 网络化ISAC系统工程结构完全就绪！\n');
fprintf('📁 代码组织清晰：core, test, plot, utils, data\n');
fprintf('🔥 核心算法：AO交替优化（关联+波束+轨迹）\n');
fprintf('⚡ 支持功能：收敛分析、性能评估、参数扫描、可视化\n');
fprintf('💾 数据管理：自动保存、检查点、断点续传\n');
fprintf('📈 可视化：收敛图、热力图、权衡曲线\n\n');
