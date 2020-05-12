function [result] = pathCheck(path)
%这个函数用于筛选掉不必要的路径
%   path是路径集合，result是筛选后的路径集合
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

