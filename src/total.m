function [result] = total(path, rcurrent)
%这个函数用于计算路径评价指标中的total部分
%   path是待评价的路径n*2矩阵，rcurrent是机器人现在位置坐标
[h, w] = size(path);
result = zeros(h, 1);
stepsize = 1;
for i = 1: h
    result(i, 1) = distance(path(i, :), rcurrent)/ i / stepsize;
end
end

