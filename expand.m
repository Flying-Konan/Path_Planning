%% 搜寻8个子节点
function [openlist] = expand(current, openlist, closelist,resolution, ob,...
          x_left_limit, x_right_limit,y_left_limit,y_right_limit,extend_area)
    flag = 1;
    for i=-resolution:resolution:resolution
        for j=-resolution:resolution:resolution
            flag = 1;
            if (i == 0 & j == 0)
                continue
            end
            x = current.node(1,1) + i;
            y = current.node(1,2) + j;
            
            for k=1:1:size(ob,1)        % 判断是否碰撞到障碍物，为保证安全设置1m的膨胀范围
                if (x>=ob(k,1)-extend_area && x<=ob(k,1)+ob(k,3)+extend_area && ...
                    y>=ob(k,2)-extend_area && y<=ob(k,2)+ob(k,4)+extend_area)
                    flag = 0;
                    break
                end
            end
            
            for k=1:1:size(ob,1)        % 判断是否超出显示范围
                if ( x<=x_left_limit | x>=x_right_limit | ...
                        y<=y_left_limit | y>=y_right_limit )
                    flag = 0;
                    break
                end
            end
            
            for k=1:1:size(closelist.node,1)    % 判断是否在closelist内
                if (x == closelist.node(k,1) && y ==closelist.node(k,2))
                    flag = 0;
                    break
                end
            end
            
            if(size(openlist.node,1) >= 1)
                for k=1:1:length(size(openlist.node,1)) % 若有重复的点检查cost
                    if (x == openlist.node(k,1) && y == openlist.node(k,2))
                        g1 = current.cost + sqrt((openlist.node(k,1) - ...
                        current.node(1,1))^2 + (openlist.node(k,2) - current.node(1,2))^2);
                        g2 = openlist.cost(k,:);
                        if(g1 >= g2)
                            continue
                        else
                            openlist.cost(k,:) = g1;
                        end
                    end
                end
            end
            
            for k=1:1:size(openlist.node,1)	% 重复的点更新完cost后不再重复添加进openlist
                if (x == openlist.node(k,1) && y == openlist.node(k,2))
                    flag = 0;
                    break
                end
            end
            
            if(flag == 1)
                openlist.node(end+1,:) = [x, y];
                openlist.parent(end+1,:) = current.node;
                openlist.cost(end+1,:) = current.cost + ...
                    sqrt((x - current.node(1,1))^2+(y - current.node(1,2))^2);
            end
        end
    end
end