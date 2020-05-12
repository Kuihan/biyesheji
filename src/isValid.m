function [result] = isValid(loc, map)
%这个函数判断所选择的点是否位于地图内
%   result返回判断结果，0是溢出，1是安全，loc是点的坐标，map是地图
[h, w] = size(map);
x = loc(1, 1);
y = loc(1, 2);
if x < 1 || x > h || y < 1 || y > w || map(x, y) == 0
    result = 0;
else
    result = 1;
end
end

