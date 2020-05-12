function [result] = distance(p1,p2)
%这个函数用于计算这个研究中的距离，假定斜相邻和纵横相邻间距相等
%   p1,p2分别为两个点坐标
result = max([abs(p1(1, 1) - p2(1, 1)), abs(p1(1, 2) - p2(1, 2))]);
end

