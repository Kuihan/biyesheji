function [result] = dist(path,map,do)
%����������ڼ���·������ָ���е�dist����
%   path�Ǵ����۵�·��n*2����,map�ǵ�ͼ��do�ǿ��ǵ�����ϰ��ﷶΧ
result = [];
[h, w] = size(path);
for i = 1: h
    p = path(i, :);
    d = nod(p, map);
    result = [result, d];
end
%result = mapminmax(result, 0, 1);
result = result / do;
end

