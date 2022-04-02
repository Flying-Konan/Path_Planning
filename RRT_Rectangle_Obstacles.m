clc
clear
close all

%% 生成障碍物
ob1 = [];
ob2 = [];
ob3 = [];
for i=0:1:10
    for j=-10:1:-5
        ob1(end+1,:) = [i,j];
    end
end
for i=-5:1:0
    for j=5:1:15
        ob2(end+1,:) = [i,j];
    end
end
for i=-5:1:0
    for j=-2:1:2
        ob3(end+1,:) = [i,j];
    end
end

%% 初始值设置
goal = [7, 5];
x_start = [-10, -10];
grow_distance = 3; % 生长步长
goal_radius = 3; % 在目标点为圆心，2m内就停止
tree = x_start;

flag = 1;
count = 1;

node.child=[];
node.parent=[];
%% RRT算法
while true
    rd_x = 20 * rand() - 10;
    rd_y = 35 * rand() - 15;
    [angle, min_idx] = cal_distance(rd_x, rd_y, tree);
    new_node_x = tree(min_idx,1)+grow_distance*cos(angle);
    new_node_y = tree(min_idx,2)+grow_distance*sin(angle);
    new_node = [new_node_x, new_node_y];
    if((new_node_x >= ob1(1,1) && new_node_x <= ob1(end,1) && ... % 排除在障碍物范围内的新点
        new_node_y >= ob1(1,2) && new_node_y <= ob1(end,2)) || ...
        (new_node_x >= ob2(1,1) && new_node_x <= ob2(end,1) && ...
         new_node_y >= ob2(1,2) && new_node_y <= ob2(end,2)) || ...
         (new_node_x >= ob3(1,1) && new_node_x <= ob3(end,1) && ...
         new_node_y >= ob3(1,2) && new_node_y <= ob3(end,2)))
        continue
    else
        for i=tree(min_idx,1):0.01:new_node_x    % 判断生长之后路径与障碍物有无交叉部分
            for j=tree(min_idx,2):0.01:new_node_y
                if((i >= ob1(1,1) && i <= ob1(end,1) && j >= ob1(1,2) && j <= ob1(end,2)) || ...
                   (i >= ob2(1,1) && i <= ob2(end,1) && j >= ob2(1,2) && j <= ob2(end,2)) || ...
                   (i >= ob3(1,1) && i <= ob3(end,1) && j >= ob3(1,2) && j <= ob3(end,2)))
                    flag = 0;
                    break
                end
            end
        end
        if (flag == true)
            tree(end+1,:) = new_node;
            node(count).parent = [tree(min_idx,1), tree(min_idx,2)];
            node(count).child = new_node;
            plot(rd_x, rd_y, '.g');hold on
            plot(new_node_x, new_node_y,'.r');hold on
            plot([tree(min_idx,1),new_node_x], [tree(min_idx,2),new_node_y],'-r');
        end
        flag = 1;
        count = count + 1;
    end
%% 显示
    rectangle('Position',[ob1(1,1),ob1(1,2),ob1(end,1)-ob1(1,1),ob1(end,2)-ob1(1,2)],...
                                        'Curvature', [0 0], 'FaceColor','k') 
    rectangle('Position',[ob2(1,1),ob2(1,2),ob2(end,1)-ob2(1,1),ob2(end,2)-ob2(1,2)],...
                                        'Curvature', [0 0], 'FaceColor','k') 
    rectangle('Position',[ob3(1,1),ob3(1,2),ob3(end,1)-ob3(1,1),ob3(end,2)-ob3(1,2)],...
                                        'Curvature', [0 0], 'FaceColor','k') 
    plot(x_start(1,1), x_start(1,2), '^b');hold on
    plot(goal(1,1), goal(1,2), '^m');hold on
    rectangle('Position',[goal(1,1)-goal_radius,goal(1,2)-goal_radius,...
                        2*goal_radius,2*goal_radius],'Curvature',[1,1]);
    set(gca,'XLim',[-20 25]); % X轴的数据显示范围
    set(gca,'XTick',[-20:5:25]); % 设置要显示坐标刻度
    set(gca,'YLim',[-20 25]); % Y轴的数据显示范围
    set(gca,'XTick',[-20:5:25]); % 设置要显示坐标刻度
    grid on
    title('RRT');
    xlabel('横坐标 x'); 
    ylabel('纵坐标 y');
    pause(0.07);
    if (sqrt((new_node_x - goal(1,1))^2 + (new_node_y- goal(1,2))^2) <= goal_radius)
        disp('find goal!');
        break
    end
    
end

%% 绘制最优路径
temp = node(end).parent;
trajectory = node(end).child;
trajectory(end+1,:) = node(end).parent;
for i=length(node):-1:1
    if(size(node(i).child,2) ~= 0 & node(i).child == temp)
        temp = node(i).parent;
        trajectory(end+1,:) = node(i).parent;
    end
end
plot(trajectory(:,1), trajectory(:,2), '-b','LineWidth',2);
pause(2);

% %% 最优路径平滑化（贝塞尔曲线法）
% for i=1:2:size(trajectory,1)-2
% P_t = [];
%     for t=0:0.01:1
%         P_1_1 = (1 - t)*trajectory(i,:) + t*trajectory(i+1,:);
%         P_1_2 = (1 - t)*trajectory(i+1,:) + t*trajectory(i+2,:);
%         P_t(end+1,:) = (1 - t)*P_1_1 + t*P_1_2;
%     end
%     if(rem(size(trajectory,1),2)==0)
%         plot([trajectory(end,1),trajectory(end-1,1)],[trajectory(end,2),trajectory(end-1,2)],'-k','LineWidth',2);
%     end
%     plot(P_t(:,1), P_t(:,2),'-k','LineWidth',2);
% end
% 
% %% 最优路径平滑化（牛顿插值,不适用）
% for i=1:1:size(trajectory,1)-1
%     P_t = [];
%     x_1 = min(trajectory(i,1),trajectory(i+1,1));
%     x_2 = max(trajectory(i,1),trajectory(i+1,1));
%     if(x_1 == trajectory(i,1))
%         y_1 = trajectory(i,2);
%         y_2 = trajectory(i+1,2);
%     else
%         y_1 = trajectory(i+1,2);
%         y_2 = trajectory(i,2);
%     end
%     for x=x_1:0.01:x_2
%         P_t(end+1,:) = [x, y_1 + ((y_2 - y_1)/(x_2 - x_1)) * (x - x_1)];
%     end
%     plot(P_t(:,1), P_t(:,2),'-k','LineWidth',2);
% end

%% 最优路径平滑化（B样条曲线）
k = 4;      % k阶k-1次B样条，3次B样条具有连续二阶导数，最合适
P = trajectory';
n = size(P,2) - 1;
flag = 2;   % flag=1均匀B样条，flag=2准均匀B样条
path=[];

if (flag == 1)
    NodeVector = linspace(0, 1, n+k+1); % 产生均匀节点,一般不用
    for u = (k-1)/(n+k+1) : 0.001 : (n+2)/(n+k+1)
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path = [path; [p_u(1,1),p_u(2,1)]];
    end
else
    NodeVector = U_quasi_uniform(n, k-1); % 准均匀B样条的节点矢量
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
        end
        p_u = P * Bik;
        path=[path; [p_u(1),p_u(2)]];
    end
end
plot(path(:,1),path(:,2),'-k','LineWidth',2);hold on
