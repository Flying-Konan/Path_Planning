function [angle, min_idx] = cal_distance(rd_x, rd_y, tree)
    distance = [];
    i = 1;
    while i<=size(tree,1)
        dx = rd_x - tree(i,1);
        dy = rd_y - tree(i,2);
        d = sqrt(dx^2 + dy^2);
        distance(i) = d;
        i = i+1;
    end
    [~, min_idx] = min(distance);
    angle = atan2(rd_y - tree(min_idx,2),rd_x - tree(min_idx,1));
end