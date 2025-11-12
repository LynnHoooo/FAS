    %% 设置MATLAB路径，使分文件夹的代码能正常工作
% 在运行任何脚本前，先运行这个脚本

% 获取当前脚本所在目录（fuxian文件夹）
current_dir = fileparts(mfilename('fullpath'));

% 添加所有子文件夹到路径
addpath(fullfile(current_dir, 'core'));
addpath(fullfile(current_dir, 'test'));  
addpath(fullfile(current_dir, 'plot'));
addpath(fullfile(current_dir, 'utils'));
addpath(fullfile(current_dir, 'data'));

fprintf('✅ 路径设置完成！\n');
fprintf('📁 已添加路径:\n');
fprintf('  - core/  (核心算法)\n');
fprintf('  - test/  (测试脚本)\n'); 
fprintf('  - plot/  (绘图脚本)\n');
fprintf('  - utils/ (工具函数)\n');
fprintf('  - data/  (数据文件)\n');
fprintf('\n💡 现在可以从任意文件夹调用其他文件夹的函数\n');
