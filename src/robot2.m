function [] = robot2(pic, s1, s2, t1, t2)
% �Ա�
% Ԥ����
% test
count = 0;
% �ؼ��ֲ�������
ka = 1.0;  %��������
kr = 100000.0;  %��������
do = 3;   %�ϰ������÷�Χ
limit = 1000000;   %����Դ���
max = 0.5 * kr;  %�����������
maxG = 0;   %�����������·��
path = [];
flag3 = 0;  %��ֹʹ��DWA����ʱ�������صı��,flag3 == 1��ʾ֮ǰʹ�ù�DWA��������Ȼ�������Ѿֲ���Сֵ
flag4 = 0;
randomTime = 0; %��¼ʹ��������������������Ѿֲ���Сֵ�Ĵ�������������������ӣ������˳��Բ����ķ�Χ����
randomTime2 = 0;
check = [0, 0]; %��ֹʹ�����·������Ϊ�Ƴ�ԭ��ԭ·����
bestPath = [];
% �����ͼ
%pic = imread('9.png');

%��ͼ
figure(1);
subplot(2,3,1);imshow(pic);title('ԭͼ');
% ͼ���ֵ����1��ʾ��·��0��ʾ�ϰ���
map = im2bw(pic, 120 / 255);
subplot(2,3,2);imshow(map);title('100X100��ֵ��ͼ'); %���Ϊԭͼ���ұ�Ϊ��ֵ��ͼ


[h,w] = size(map);
% ���������յ㣨������������£�,Ĭ��Ϊ��ͼ���Ͻ�
sx = s1(1, 1);
sy = s1(1, 2);
tx = t1(1, 1);
ty = t1(1, 2);
% ���������յ㣨������2����Ĭ��Ϊ��ͼ���½�
sx2 = s2(1, 1);
sy2 = s2(1, 2);
tx2 = t2(1, 1);
ty2 = t2(1, 2);
% �����Ƴ�ͼ(������1��
px = zeros(1,h * w);
py = zeros(1,h * w);
ip = zeros(1,h * w);
img_potential = map * 0.0;
for i = 1: h
    for j = 1: w
        count = count + 1;
        %�����˲������ԣ�ÿ����10000�����߻�ͨ��һ��
        if rem(count, 10000) == 0
            %count
        end
        % Ŀ����������ug
        ug = 0.5 * ka * norm([i, j] - [tx, ty])^2;
        % �ϰ���ĳ�����uo��������uog������ģ�⶯���˶���
        % ������uog���壺ֻ��������������������ϰ����������������һ���з�Χ����������Ӱ��
        % �ڳ������ڣ�����������������ϴ󣬹����������Բ���
        % ����ϰ���ľ���min���ϰ���������Ӱ�췶Χdo
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
        % �����ǰ��λ�������ϰ����������ϰ���ľ���Ϊ0����Ѹô��Ƴ����
        if min == 0.0
            uo = max;
            %�����ǰ��λû���ϰ������һ���ķ�Χ�ڴ����ϰ����������ڸõ㴦���ܵ��������ã������Ƴ�
        elseif min < do
            %ִ�й�ʽ1
            uo = 0.5 * kr * (1.0 / min - 1.0 / do)^2;
            if uo > max
                uo = max;
            end
        else
            %ִ�й�ʽ2
            uo = 0.0;
        end
        img_potential(i, j) = ug + uo;
        px(1, count) = i;
        py(1, count) = j;
        ip(1, count) = img_potential(i, j);
    end
end
[px, py] = meshgrid(1: 1: w, 1: 1: h);
subplot(2, 3, 3); mesh(px, py, img_potential); title('������1�Ƴ�ͼ');xlabel('������'), ylabel('������'), zlabel('�˴��Ƴ�');
% �����Ƴ�ͼ(������2��
px2 = zeros(1,h * w);
py2 = zeros(1,h * w);
ip2 = zeros(1,h * w);
img_potential2 = map * 0.0;
for i = 1: h
    for j = 1: w
        % Ŀ����������ug
        ug = 0.5 * ka * norm([i, j] - [tx2, ty2])^2;
        % �ϰ���ĳ�����uo��������uog������ģ�⶯���˶���
        % ������uog���壺ֻ��������������������ϰ����������������һ���з�Χ����������Ӱ��
        % �ڳ������ڣ�����������������ϴ󣬹����������Բ���
        % ����ϰ���ľ���min���ϰ���������Ӱ�췶Χdo
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
        % �����ǰ��λ�������ϰ����������ϰ���ľ���Ϊ0����Ѹô��Ƴ����
        if min == 0.0
            uo = max;
            %�����ǰ��λû���ϰ������һ���ķ�Χ�ڴ����ϰ����������ڸõ㴦���ܵ��������ã������Ƴ�
        elseif min < do
            %ִ�й�ʽ1
            uo = 0.5 * kr * (1.0 / min - 1.0 / do)^2;
            if uo > max
                uo = max;
            end
        else
            %ִ�й�ʽ2
            uo = 0.0;
        end
        img_potential2(i, j) = ug + uo;
        px2(1, count) = i;
        py2(1, count) = j;
        ip2(1, count) = img_potential2(i, j);
    end
end
[px2, py2] = meshgrid(1: 1: w, 1: 1: h);
subplot(2, 3, 4); mesh(px2, py2, img_potential2); title('������2�Ƴ�ͼ');xlabel('������'), ylabel('������'), zlabel('�˴��Ƴ�');
% �ݶ��½���
rcurrent = [sx, sy];
goal = [tx, ty];
potential_current = img_potential(sx, sy);
direction = [1, 0; 0, 1; -1, 0; 0, -1; 1, 1; 1, -1; -1, 1; -1, -1];
directionDWA = [1, 0; 0, 1; -1, 0; 0, -1];
roadx = sx;
roady = sy;
num = 0;

%������2
rcurrent2 = [sx2, sy2];
goal2 = [tx2, ty2];
roadx2 = sx2;
roady2 = sy2;
potential_current2 = img_potential2(sx2, sy2);

% �ֲ���Сֵ�����Ӧ����
alpha = 1;
beta = 2;
gama = 2;
m = 4;
n = 8;

while norm(rcurrent - goal) > 0.0 && norm(rcurrent2 - goal2) > 0.0
    %��������������
    if num < limit
        num = num + 1;
        if norm(rcurrent - goal) > 0.0
            % ������1
            potential_temp = potential_current;
            rtemp = rcurrent;
            rcurrent
            %���㶯̬�Ƴ�
            potential_dynamic = img_potential;
            for i = rcurrent2(1, 1) - do: rcurrent2(1, 1) + do
                for j = rcurrent2(1, 2) - do: rcurrent2(1, 2) + do
                    if isValid([i, j], map)
                        potential_dynamic(i, j) = max;
                    end
                end
            end
            %�ж�ʹ����ͨ�˹��Ƴ����Ƿ��ܼ����ƶ�
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
            % ��¼�����Ѿ����ĵ㣬������ͼ,check��¼·���е���һ����
            if flag == 1
                check = rtemp;
                roadx = [roadx, rcurrent(1, 1)];
                roady = [roady, rcurrent(1, 2)];
            end
            
            % �ֲ���Сֵ��⣬֮��ʹ��DWA�㷨�ӳ��ֲ���Сֵ
            if flag == 0
                n = 8;
                % ʹ��DWA�ظ������,������������
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
                    % ��ǰ�ֲ���Сֵ��Ϊrcurrent
                    % ����ȫ��·������, ����m^n, ����n
                    allDirection = dfs(zeros(m^n, n), m, n);
                    allDirection = pathCheck(allDirection);
                    [h3, w3] = size(allDirection);
                    maxG = 0;
                    for i = 1: h3
                        flag2 = 1;
                        directionGroup = allDirection(i, :);
                        % ����·���ϵ��ʵ��λ�ã���һ��n*2����path����ʾ
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
                        % ����heading������һ��Բ�ϵĵ㣬ǰһ��Բ�ϵĵ㣬Ŀ�����ɵļнǵĲ��ǳ���180�ȵĽ��, n*1
                        % ����dist����ÿһ��Բ�ϵ�һ����������ϰ���ľ���, n*1
                        % ����total����ÿһ��Բ�ϵ�һ������������ִ�λ��֮��ľ���, n*1
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
                    % ֱ��ʹ�����·�����ƶ���·��ĩ��
                    % �����ж�����ֹ�߳���ͼ��
                    if flag == 0
                        disp('Ѱ·ʧ��');
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
        %������2
        if norm(rcurrent2 - goal2) > 0.0
            potential_temp2 = potential_current2;
            rtemp2 = rcurrent2;
            %�ж��Ƿ��ܼ����ƶ�
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
            % ��¼�����Ѿ����ĵ㣬������ͼ,check��¼·���е���һ����
            if flag == 1
                roadx2 = [roadx2, rcurrent2(1, 1)];
                roady2 = [roady2, rcurrent2(1, 2)];
            end
            
            % �ֲ���Сֵ��⣬֮��ʹ��DWA�㷨�ӳ��ֲ���Сֵ
            if flag == 0
                n = 8;
                % ʹ��DWA�ظ������,������������
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
                % ��ǰ�ֲ���Сֵ��Ϊrcurrent2
                % ����ȫ��·������, ����m^n, ����n
                allDirection = dfs(zeros(m^n, n), m, n);
                allDirection = pathCheck(allDirection);
                [h3, w3] = size(allDirection);
                maxG = 0;
                for i = 1: h3
                    flag2 = 1;
                    directionGroup = allDirection(i, :);
                    % ����·���ϵ��ʵ��λ�ã���һ��n*2����path����ʾ
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
                    % ����heading������һ��Բ�ϵĵ㣬ǰһ��Բ�ϵĵ㣬Ŀ�����ɵļнǵĲ��ǳ���180�ȵĽ��, n*1
                    % ����dist����ÿһ��Բ�ϵ�һ����������ϰ���ľ���, n*1
                    % ����total����ÿһ��Բ�ϵ�һ������������ִ�λ��֮��ľ���, n*1
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
                % ֱ��ʹ�����·�����ƶ���·��ĩ��
                % �����ж�����ֹ�߳���ͼ��
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
inf = ['������1�Ѿ�����Ŀ��㣬����Ϊ' num2str(w2) '��.'];
disp(inf);
for i = 1: w2
    result(roadx(1, i), roady(1, i), :) = [255, 0, 0];
end

result2 = pic;
[h4, w4] = size(roadx2);
inf2 = ['������2�Ѿ�����Ŀ��㣬����Ϊ' num2str(w4) '��.'];
disp(inf2);

for i = 1: w4
    result(roadx2(1, i), roady2(1, i), :) = [0, 255, 0];
end

% ��������ű�������յ�

subplot(2, 3, 5);imshow(result);title('������·���滮');
end