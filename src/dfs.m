function [result] = dfs(input, m, n)
% 这个函数用于生成一个从1到m中选取n个数（可重复）的组合，用于路径评价，使用递归方法
% input和result矩阵的size为[m^n, n] 
result = input;
if n == 1
    for i = 1: m
        result(i, 1) = i;
    end
else
    for i = 1: m
        part = createMatrix(i, m^(n - 1), n);
        part(:, 2: n) = dfs(part(:, 2: n), m, n - 1);
        startRow = (i - 1) * m^(n - 1) + 1;
        endRow = i * m^(n - 1);
        result(startRow: endRow, :) = part;
    end
end
end

