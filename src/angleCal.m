function [result] = angleCal(p1, d1, p2)
%这个函数用于计算三点组成角的弧度值
%   参数为组成角所需的三个点，其中d1为角的顶点
% 计算两个向量
a = p1 - d1;
b = p2 - d1;
% 用c表示向量a、b所夹的弧度角
c = dot(a, b)/ norm(a, 2)/ norm(b, 2);
result = c;
end

