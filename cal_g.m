function [g] = cal_g(openlist, current)
    for i=1:1:size(openlist.node, 1)
        g(i,1) = current.cost + sqrt((openlist.node(i,1) - current.node(1,1))^2 + ...
                                    (openlist.node(i,2) - current.node(1,2))^2);
    end
end