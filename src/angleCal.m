function [result] = angleCal(p1, d1, p2)
%����������ڼ���������ɽǵĻ���ֵ
%   ����Ϊ��ɽ�����������㣬����d1Ϊ�ǵĶ���
% ������������
a = p1 - d1;
b = p2 - d1;
% ��c��ʾ����a��b���еĻ��Ƚ�
c = dot(a, b)/ norm(a, 2)/ norm(b, 2);
result = c;
end

