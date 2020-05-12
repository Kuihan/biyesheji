function [result] = heading(path, rcurrent, goal)
%这个函数用于计算路径评价指标中的heading部分
%   path是待评价的路径n*2矩阵，rcurrent是机器人现在位置坐标，goal是目标点坐标
[h, w] = size(path);
result = zeros(1, h);
for i = 1: h
    if i == 1
        result(1, i) = (pi - angleCal(rcurrent, path(i, :), goal))/ pi;
    else
        result(1, i) = (pi - angleCal(path(i - 1, :), path(i, :), goal))/ pi;
    end
end
end

