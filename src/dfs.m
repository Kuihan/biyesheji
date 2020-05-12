function [result] = dfs(input, m, n)
% ���������������һ����1��m��ѡȡn���������ظ�������ϣ�����·�����ۣ�ʹ�õݹ鷽��
% input��result�����sizeΪ[m^n, n] 
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

