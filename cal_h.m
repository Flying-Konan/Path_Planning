function [h] = cal_h(openlist, goal)
    for i=1:1:size(openlist.node, 1)
        dx = abs(openlist.node(i,1) - goal(1,1));
        dy = abs(openlist.node(i,2) - goal(1,2));
        % 曼哈顿距离
        h(i,1) = dx + dy;
        % 欧氏距离
%         h = sqrt(dx^2 + dy^2);
    end
end