clc
clear
close all

%% 设置障碍物（矩形）
% ob1 = [-5, -20, 10, 20]; % 起始点坐标及宽和高
% ob2 = [-5, 5, 10, 10];
% ob3 = [-20, 5, 10, 10];
% ob4 = [-20,-20, 10,20];
% ob1 = [-10,-10,1,70];
% ob2 = [-10,-10,70,1];
% ob3 = [-10,60,70,1];
% ob4 = [60,-10,1,71];
% ob5 = [20,-10,1,51];
% ob6 = [40,20,1,41];
% ob = [ob1;ob2;ob3;ob4;ob5;ob6];
ob1 = [0,4,14,1];
ob2 = [3,4,1,15];
ob3 = [6,8,1,6];
ob4 = [6,13,12,1];
ob5 = [11,8,1,10];
ob6 = [17,4,1,15];
ob7 = [17,18,3,1];

ob = [ob1;ob2;ob3;ob4;ob5;ob6;ob7];

% 左右边界
x_left_limit = 0; 
x_right_limit = 20;
y_left_limit = 0;
y_right_limit = 20;

% 膨胀半径
extend_area = 0;
%% 初始值设置
x_start = [4, 1];
resolution = 1 ;
goal = [14, 15];

openlist.node= [];
openlist.parent = [];
openlist.cost = [];

closelist.node = x_start;
closelist.parent = [];
closelist.cost = [];

current.node = x_start;
current.cost = 0;

[openlist] = expand(current, openlist, closelist,resolution, ob,...
    x_left_limit, x_right_limit,y_left_limit,y_right_limit,extend_area); % 先找到起始点的子节点，从左下角开始，每列从下至上8个点

while true
%% 显示
    for i=1:1:size(ob,1)        % 绘制障碍物及膨胀半径
        rectangle('Position',[ob(i,1), ob(i,2), ob(i,3), ob(i,4)],'Curvature', [0 0], 'FaceColor','k');
%         c = [0,500,1000,500];
%         
%         patch([ob(i,1)-extend_area, ob(i,1), ob(i,1),ob(i,1)-extend_area],... 
%         [ob(i,2)-extend_area, ob(i,2) , ob(i,2)+ob(i,4), ob(i,2)+ob(i,4)+extend_area], c);
%         
%         patch([ob(i,1)-extend_area, ob(i,1)+ob(i,3)+extend_area, ob(i,1)+ob(i,3),ob(i,1)],...
%         [ob(i,2)-extend_area, ob(i,2)-extend_area , ob(i,2), ob(i,2)], c); 
%        
%         patch([ob(i,1), ob(i,1)+ob(i,3),ob(i,1)+ob(i,3)+extend_area,ob(i,1)-extend_area],...
%         [ob(i,2)+ob(i,4), ob(i,2)+ob(i,4) , ob(i,2)+ob(i,4)+extend_area, ob(i,2)+ob(i,4)+extend_area], c); 
%         
%         patch([ob(i,1)+ob(i,3), ob(i,1)+ob(i,3)+extend_area, ob(i,1)+ob(i,3)+extend_area,ob(i,1)+ob(i,3)],...
%         [ob(i,2), ob(i,2)-extend_area , ob(i,2)+ob(i,4)+extend_area, ob(i,2)+ob(i,4)], c); 
    end
    hold on

    plot(x_start(1,1), x_start(1,2), '^k');hold on
    plot(goal(1,1), goal(1,2), '^m');hold on
    plot(openlist.node(:,1), openlist.node(:,2), 'xb');hold on
    set(gca,'XLim',[x_left_limit x_right_limit]); % X轴的数据显示范围
    set(gca,'XTick',[x_left_limit:resolution:x_right_limit]); % 设置要显示坐标刻度
    set(gca,'YLim',[y_left_limit y_right_limit]); % Y轴的数据显示范围
    set(gca,'YTick',[y_left_limit:resolution:y_right_limit]); % 设置要显示坐标刻度
    grid on
    title('A-star');
    xlabel('横坐标 x'); 
    ylabel('纵坐标 y');
    pause(0.1);
%% 算法部分
    [g] = cal_g(openlist, current);  % 当前节点到起点代价
    [h] = cal_h(openlist, goal);     % 当前节点到终点代价，默认曼哈顿距离
    f = g + h;
    [~, min_idx] = min(f);
    current.node = openlist.node(min_idx,:);
    current.cost = openlist.cost(min_idx,:);
    
    closelist.parent(end+1,:) = openlist.parent(min_idx,:);
    closelist.node(end+1,:) = current.node;    % 新的父节点移入closelist
    closelist.cost(end+1,:) = current.cost;
    
    openlist.node(min_idx,:) = [];        % 新的父节点从openlist中移除
    openlist.parent(min_idx,:) = [];
    openlist.cost(min_idx,:) = [];
    
    [openlist] = expand(current, openlist, closelist, resolution, ob,...
        x_left_limit, x_right_limit,y_left_limit,y_right_limit,extend_area);
    
     if(current.node == goal)
        disp('Find goal!');
        break
     end
end

%% 绘制最佳路径
trajectory = closelist.node(end,:);
temp = closelist.parent(end,:);
for i=size(closelist.node,1):-1:2
    if(closelist.node(i,:) == temp)
        temp = closelist.parent(i-1,:);
        trajectory(end+1,:) = closelist.node(i,:);
    end
end
trajectory(end+1,:) = closelist.node(1,:);          % node和parent长度不同，要把起点也加上
plot(trajectory(:,1), trajectory(:,2),'-r','LineWidth',2);

%% 平滑化
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