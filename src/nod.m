function [result] = nod(point,map)
%����������ڼ��㵱ǰ�㵽����ϰ���ľ���
%   pointΪ��ǰ������꣬map�ǵ�ͼ
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

