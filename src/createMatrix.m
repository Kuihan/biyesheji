function [result] = createMatrix(input, row, col)
%���������������һ��ָ����������Ԫ�ؾ�Ϊָ�������ľ���
%   input��ָ��Ԫ�أ�row��col�ֱ���������
result = zeros(row, col);
for i = 1: row
    for j = 1: col
        result(i, j) = input;
    end
end
end

