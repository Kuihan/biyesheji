function [] = robot2(pic, s1, s2, t1, t2)
% 自编
% 预处理
% test
count = 0;
% 关键字参数处理
ka = 1.0;  %引力增益
kr = 100000.0;  %斥力增益
do = 3;   %障碍物作用范围
limit = 1000000;   %最大尝试次数
max = 0.5 * kr;  %最大力场上限
maxG = 0;   %评价最佳逃脱路线
path = [];
flag3 = 0;  %防止使用DWA方法时反复来回的标记,flag3 == 1表示之前使用过DWA方法后，依然不能逃脱局部极小值
flag4 = 0;
randomTime = 0; %记录使用随机波动方法尝试逃脱局部极小值的次数，随着这个次数增加，机器人尝试波动的范围会变大
randomTime2 = 0;
check = [0, 0]; %防止使用最佳路径后，因为势场原因原路返回
bestPath = [];
% 读入地图
%pic = imread('9.png');

%作图
figure(1);
subplot(2,3,1);imshow(pic);title('原图');
% 图像二值化，1表示道路，0表示障碍物
map = im2bw(pic, 120 / 255);
subplot(2,3,2);imshow(map);title('100X100二值地图'); %左边为原图，右边为二值地图


[h,w] = size(map);
% 设置起点和终点（单机器人情况下）,默认为地图左上角
sx = s1(1, 1);
sy = s1(1, 2);
tx = t1(1, 1);
ty = t1(1, 2);
% 设置起点和终点（机器人2），默认为地图左下角
sx2 = s2(1, 1);
sy2 = s2(1, 2);
tx2 = t2(1, 1);
ty2 = t2(1, 2);
% 创建势场图(机器人1）
px = zeros(1,h * w);
py = zeros(1,h * w);
ip = zeros(1,h * w);
img_potential = map * 0.0;
for i = 1: h
    for j = 1: w
        count = count + 1;
        %机器人步数测试，每进行10000步行走会通报一次
        if rem(count, 10000) == 0
            %count
        end
        % 目标点的引力场ug
        ug = 0.5 * ka * norm([i, j] - [tx, ty])^2;
        % 障碍物的斥力场uo，引力场uog（用于模拟动物运动）
        % 引力场uog定义：只考虑与机器人相距最近的障碍点存在引力场，在一个中范围内受引力场影响
        % 在斥力场内，由于引力与斥力相差较大，故引力场忽略不计
        % 最近障碍物的距离min，障碍物斥力最大影响范围do
        min = do;
        for x = i - do: i + do
            if x < 1
                continue;
            end
            if x > h
                continue;
            end
            for y = j - do: j + do
                if y < 1
                    continue;
                end
                if y > w
                    continue;
                end
                if map(x, y) == 0
                    dmin = norm([i, j] - [x, y]);
                    if dmin < min
                        min = dmin;
                    end
                end
            end
        end
        % 如果当前点位正好有障碍物，即与最近障碍物的距离为0，则把该处势场最大化
        if min == 0.0
            uo = max;
            %如果当前点位没有障碍物，但在一定的范围内存在障碍物，则机器人在该点处会受到斥力作用，增大势场
        elseif min < do
            %执行公式1
            uo = 0.5 * kr * (1.0 / min - 1.0 / do)^2;
            if uo > max
                uo = max;
            end
        else
            %执行公式2
            uo = 0.0;
        end
        img_potential(i, j) = ug + uo;
        px(1, count) = i;
        py(1, count) = j;
        ip(1, count) = img_potential(i, j);
    end
end
[px, py] = meshgrid(1: 1: w, 1: 1: h);
subplot(2, 3, 3); mesh(px, py, img_potential); title('机器人1势场图');xlabel('像素行'), ylabel('像素列'), zlabel('此处势场');
% 创建势场图(机器人2）
px2 = zeros(1,h * w);
py2 = zeros(1,h * w);
ip2 = zeros(1,h * w);
img_potential2 = map * 0.0;
for i = 1: h
    for j = 1: w
        % 目标点的引力场ug
        ug = 0.5 * ka * norm([i, j] - [tx2, ty2])^2;
        % 障碍物的斥力场uo，引力场uog（用于模拟动物运动）
        % 引力场uog定义：只考虑与机器人相距最近的障碍点存在引力场，在一个中范围内受引力场影响
        % 在斥力场内，由于引力与斥力相差较大，故引力场忽略不计
        % 最近障碍物的距离min，障碍物斥力最大影响范围do
        min = do;
        for x = i - do: i + do
            if x < 1
                continue;
            end
            if x > h
                continue;
            end
            for y = j - do: j + do
                if y < 1
                    continue;
                end
                if y > w
                    continue;
                end
                if map(x, y) == 0
                    dmin = norm([i, j] - [x, y]);
                    if dmin < min
                        min = dmin;
                    end
                end
            end
        end
        % 如果当前点位正好有障碍物，即与最近障碍物的距离为0，则把该处势场最大化
        if min == 0.0
            uo = max;
            %如果当前点位没有障碍物，但在一定的范围内存在障碍物，则机器人在该点处会受到斥力作用，增大势场
        elseif min < do
            %执行公式1
            uo = 0.5 * kr * (1.0 / min - 1.0 / do)^2;
            if uo > max
                uo = max;
            end
        else
            %执行公式2
            uo = 0.0;
        end
        img_potential2(i, j) = ug + uo;
        px2(1, count) = i;
        py2(1, count) = j;
        ip2(1, count) = img_potential2(i, j);
    end
end
[px2, py2] = meshgrid(1: 1: w, 1: 1: h);
subplot(2, 3, 4); mesh(px2, py2, img_potential2); title('机器人2势场图');xlabel('像素行'), ylabel('像素列'), zlabel('此处势场');
% 梯度下降法
rcurrent = [sx, sy];
goal = [tx, ty];
potential_current = img_potential(sx, sy);
direction = [1, 0; 0, 1; -1, 0; 0, -1; 1, 1; 1, -1; -1, 1; -1, -1];
directionDWA = [1, 0; 0, 1; -1, 0; 0, -1];
roadx = sx;
roady = sy;
num = 0;

%机器人2
rcurrent2 = [sx2, sy2];
goal2 = [tx2, ty2];
roadx2 = sx2;
roady2 = sy2;
potential_current2 = img_potential2(sx2, sy2);

% 局部极小值检测相应参数
alpha = 1;
beta = 2;
gama = 2;
m = 4;
n = 8;

while norm(rcurrent - goal) > 0.0 && norm(rcurrent2 - goal2) > 0.0
    %最大搜索次数检测
    if num < limit
        num = num + 1;
        if norm(rcurrent - goal) > 0.0
            % 机器人1
            potential_temp = potential_current;
            rtemp = rcurrent;
            %计算动态势场
            potential_dynamic = img_potential;
            for i = rcurrent2(1, 1) - do: rcurrent2(1, 1) + do
                for j = rcurrent2(1, 2) - do: rcurrent2(1, 2) + do
                    if isValid([i, j], map)
                        potential_dynamic(i, j) = max;
                    end
                end
            end
            %判断使用普通人工势场法是否还能继续移动
            flag = 0;
            for i = 1: 8
                testx = rtemp(1, 1) + direction(i, 1);
                testy = rtemp(1, 2) + direction(i, 2);
                if testx < 1
                    continue
                end
                if testx > h
                    continue
                end
                if testy < 1
                    continue
                end
                if testy > w
                    continue
                end
                if potential_current > potential_dynamic(testx, testy) && norm(check - [testx, testy]) ~= 0
                    potential_current = potential_dynamic(testx, testy);
                    rcurrent = [testx, testy];
                    flag = 1;
                end
            end
            % 记录所有已经过的点，用于做图,check记录路径中的上一个点
            if flag == 1
                check = rtemp;
                roadx = [roadx, rcurrent(1, 1)];
                roady = [roady, rcurrent(1, 2)];
            end
            
            % 局部极小值检测，之后使用DWA算法逃出局部极小值
            if flag == 0
                n = 8;
                % 使用DWA重复的情况,用随机波动解决
                if flag3 == 1
                    randomTime = randomTime + 1;
                    n = n * randomTime;
                    flag3 = 0;
                    for i = 1: m
                        flag2 = 1;
                        nowDirection = directionDWA(i, :);
                        path = zeros(n, 2);
                        for j = 1: n
                            if j == 1
                                path(j, :) = rcurrent + nowDirection;
                            else
                                path(j, :) = path(j - 1, :) + nowDirection;
                            end
                            if ~isValid(path(j, :), map)
                                flag2 = 0;
                                break;
                            end
                        end
                        if flag2 == 0
                            continue;
                        end
                        rcurrent = path(n, :);
                        check = path(n - 1, :);
                        for j = 1: n
                            roadx = [roadx, path(j, 1)];
                            roady = [roady, path(j, 2)];
                        end
                        break;
                    end
                    flag = 1;
                end
                if flag == 0
                    flag3 = 1;
                    % 当前局部最小值点为rcurrent
                    % 生成全部路径矩阵, 行数m^n, 列数n
                    allDirection = dfs(zeros(m^n, n), m, n);
                    allDirection = pathCheck(allDirection);
                    [h3, w3] = size(allDirection);
                    maxG = 0;
                    for i = 1: h3
                        flag2 = 1;
                        directionGroup = allDirection(i, :);
                        % 计算路径上点的实际位置，用一个n*2矩阵path来表示
                        path = zeros(n, 2);
                        for j = 1: n
                            nowDirection = directionDWA(directionGroup(1, j), :);
                            if j == 1
                                path(j, :) = rcurrent + nowDirection;
                                if ~isValid(path(j, :), map)
                                    flag2 = 0;
                                end
                            else
                                path(j, :) = path(j - 1, :) + nowDirection;
                                if ~isValid(path(j, :), map)
                                    flag2 = 0;
                                end
                            end
                        end
                        % 计算heading，即后一层圆上的点，前一层圆上的点，目标点组成的夹角的补角除以180度的结果, n*1
                        % 计算dist，即每一层圆上的一个点与最近障碍物的距离, n*1
                        % 计算total，即每一层圆上的一个点与机器人现处位置之间的距离, n*1
                        if flag2 == 0
                            continue;
                        end
                        G = alpha * heading(path, rcurrent, [tx, ty]) + beta * dist(path, map, do + n) + gama * total(path, rcurrent);
                        if G > maxG
                            maxG = G;
                            bestPath = path;
                            flag = 1;
                        end
                    end
                    % 直接使用最佳路径，移动到路径末端
                    % 增加判定，防止走出地图外
                    if flag == 0
                        disp('寻路失败');
                        break;
                    end
                    rcurrent = bestPath(n, :);
                    check = bestPath(n - 1, :);
                    for i = 1: n
                        roadx = [roadx, bestPath(i, 1)];
                        roady = [roady, bestPath(i, 2)];
                    end
                end
            end
        end
        %机器人2
        if norm(rcurrent2 - goal2) > 0.0
            potential_temp2 = potential_current2;
            rtemp2 = rcurrent2;
            %判断是否还能继续移动
            flag = 0;
            for i = 1: 8
                testx = rtemp2(1, 1) + direction(i, 1);
                testy = rtemp2(1, 2) + direction(i, 2);
                if testx < 1
                    continue
                end
                if testx > h
                    continue
                end
                if testy < 1
                    continue
                end
                if testy > w
                    continue
                end
                if potential_current2 > img_potential2(testx, testy)
                    potential_current2 = img_potential2(testx, testy);
                    rcurrent2 = [testx, testy];
                    flag = 1;
                end
            end
            % 记录所有已经过的点，用于做图,check记录路径中的上一个点
            if flag == 1
                roadx2 = [roadx2, rcurrent2(1, 1)];
                roady2 = [roady2, rcurrent2(1, 2)];
            end
            
            % 局部极小值检测，之后使用DWA算法逃出局部极小值
            if flag == 0
                n = 8;
                % 使用DWA重复的情况,用随机波动解决
                if flag4 == 1
                    randomTime2 = randomTime2 + 1;
                    n = n * randomTime2;
                    flag4 = 0;
                    for i = 1: m
                        flag2 = 1;
                        nowDirection = directionDWA(i, :);
                        path = zeros(n, 2);
                        for j = 1: n
                            if j == 1
                                path(j, :) = rcurrent2 + nowDirection;
                            else
                                path(j, :) = path(j - 1, :) + nowDirection;
                            end
                            if ~isValid(path(j, :), map)
                                flag2 = 0;
                                break;
                            end
                        end
                        if flag2 == 0
                            continue;
                        end
                        rcurrent2 = path(n, :);
                        for j = 1: n
                            roadx2 = [roadx2, path(j, 1)];
                            roady2 = [roady2, path(j, 2)];
                        end
                        break;
                    end
                    continue;
                end
                flag4 = 1;
                % 当前局部最小值点为rcurrent2
                % 生成全部路径矩阵, 行数m^n, 列数n
                allDirection = dfs(zeros(m^n, n), m, n);
                allDirection = pathCheck(allDirection);
                [h3, w3] = size(allDirection);
                maxG = 0;
                for i = 1: h3
                    flag2 = 1;
                    directionGroup = allDirection(i, :);
                    % 计算路径上点的实际位置，用一个n*2矩阵path来表示
                    path = zeros(n, 2);
                    for j = 1: n
                        nowDirection = directionDWA(directionGroup(1, j), :);
                        if j == 1
                            path(j, :) = rcurrent2 + nowDirection;
                            if ~isValid(path(j, :), map)
                                flag2 = 0;
                            end
                        else
                            path(j, :) = path(j - 1, :) + nowDirection;
                            if ~isValid(path(j, :), map)
                                flag2 = 0;
                            end
                        end
                    end
                    % 计算heading，即后一层圆上的点，前一层圆上的点，目标点组成的夹角的补角除以180度的结果, n*1
                    % 计算dist，即每一层圆上的一个点与最近障碍物的距离, n*1
                    % 计算total，即每一层圆上的一个点与机器人现处位置之间的距离, n*1
                    if flag2 == 0
                        continue;
                    end
                    G = alpha * heading(path, rcurrent2, [tx2, ty2]) + beta * dist(path, map, do + n) + gama * total(path, rcurrent2);
                    if G > maxG
                        maxG = G;
                        bestPath = path;
                        flag = 1;
                    end
                end
                % 直接使用最佳路径，移动到路径末端
                % 增加判定，防止走出地图外
                if flag == 0
                    continue;
                end
                rcurrent2 = bestPath(n, :);
                for i = 1: n
                    roadx2 = [roadx2, bestPath(i, 1)];
                    roady2 = [roady2, bestPath(i, 2)];
                end
                continue;
            end
        end
    else
        disp('out of limit');
        break;
    end
end
result = pic;
[h2, w2] = size(roadx);
inf = ['机器人1已经到达目标点，步数为' num2str(w2) '步.'];
disp(inf);
for i = 1: w2
    result(roadx(1, i), roady(1, i), :) = [255, 0, 0];
end

result2 = pic;
[h4, w4] = size(roadx2);
inf2 = ['机器人2已经到达目标点，步数为' num2str(w4) '步.'];
disp(inf2);

for i = 1: w4
    result(roadx2(1, i), roady2(1, i), :) = [0, 255, 0];
end

% 用特殊符号标记起点和终点

subplot(2, 3, 5);imshow(result);title('机器人路径规划');
end
