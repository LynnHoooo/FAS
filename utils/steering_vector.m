function a = steering_vector(arg1, theta)
% steering_vector: 阵列导向矢量（支持新旧两种接口）
% 1) 新接口：a = steering_vector(t, theta)
%    - t: 位置向量 (Na×1)，归一化（以波长为单位）
%    - 使用公式 a = exp(1j * 2π * t * cos(theta))（与流体天线改造一致）
% 2) 旧接口：a = steering_vector(Na, theta)
%    - Na: 天线数（标量），默认半波长等间距 d_lambda=0.5
%    - 使用公式 a = exp(1j * 2π * d_lambda * n * sin(theta))（旧版惯例）

if isvector(arg1) && numel(arg1) > 1
    % 新接口：位置向量 t
    t = arg1(:);
    a = exp(1j * 2 * pi * t * cos(theta));
else
    % 旧接口：给定 Na，默认半波长等间距
    Na = double(arg1);
    d_lambda = 0.5; % 天线间距（半波长）
    n = (0:Na-1)';
    a = exp(1j * 2 * pi * d_lambda * n * sin(theta));
end
end
