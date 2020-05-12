function [result] = heading(path, rcurrent, goal)
%����������ڼ���·������ָ���е�heading����
%   path�Ǵ����۵�·��n*2����rcurrent�ǻ���������λ�����꣬goal��Ŀ�������
[h, w] = size(path);
result = zeros(1, h);
for i = 1: h
    if i == 1
        result(1, i) = (pi - angleCal(rcurrent, path(i, :), goal))/ pi;
    else
        result(1, i) = (pi - angleCal(path(i - 1, :), path(i, :), goal))/ pi;
    end
end
end

