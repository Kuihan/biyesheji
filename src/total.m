function [result] = total(path, rcurrent)
%����������ڼ���·������ָ���е�total����
%   path�Ǵ����۵�·��n*2����rcurrent�ǻ���������λ������
[h, w] = size(path);
result = zeros(h, 1);
stepsize = 1;
for i = 1: h
    result(i, 1) = distance(path(i, :), rcurrent)/ i / stepsize;
end
end

