function [result] = isValid(loc, map)
%��������ж���ѡ��ĵ��Ƿ�λ�ڵ�ͼ��
%   result�����жϽ����0�������1�ǰ�ȫ��loc�ǵ�����꣬map�ǵ�ͼ
[h, w] = size(map);
x = loc(1, 1);
y = loc(1, 2);
if x < 1 || x > h || y < 1 || y > w || map(x, y) == 0
    result = 0;
else
    result = 1;
end
end

