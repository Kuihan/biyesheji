function [result] = nod(point,map)
%这个函数用于计算当前点到最近障碍物的距离
%   point为当前点的坐标，map是地图
[h, w] = size(map);
min = 100;
for i = 1: h
    for j = 1: w
        if map(i, j) == 0 && distance(point, [i, j]) < min
            min = distance(point, [i, j]);
        end
    end
end
result = min;
end

