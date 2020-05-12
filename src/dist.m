function [result] = dist(path,map,do)
%这个函数用于计算路径评价指标中的dist部分
%   path是待评价的路径n*2矩阵,map是地图，do是考虑的最大障碍物范围
result = [];
[h, w] = size(path);
for i = 1: h
    p = path(i, :);
    d = nod(p, map);
    result = [result, d];
end
%result = mapminmax(result, 0, 1);
result = result / do;
end

