function [result] = createMatrix(input, row, col)
%这个函数用于生成一个指定行列数且元素均为指定整数的矩阵
%   input是指定元素，row，col分别是行与列
result = zeros(row, col);
for i = 1: row
    for j = 1: col
        result(i, j) = input;
    end
end
end

