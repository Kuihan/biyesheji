function [result] = pathCheck(path)
%�����������ɸѡ������Ҫ��·��
%   path��·�����ϣ�result��ɸѡ���·������
result = [];
[h, w] = size(path);
for i = 1: h
    onePath = path(i, :);
    if (ismember(1, onePath) && ismember(3, onePath)) || (ismember(2, onePath) && ismember(4, onePath))
        continue;
    end
    result = [result; onePath];
end

end

