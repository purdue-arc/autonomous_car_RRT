function [path, length] = evaluateTree(state_tree, parents, goal, radius)
    num_nodes = size(state_tree, 1);
    % Find the nodes in the goal
    goal_nodes = [];
    for i = 1:num_nodes
        % find Archimedes distance
        dist = sqrt((goal(1) - state_tree(i, 1))^2 + (goal(2) - state_tree(i, 2))^2);
        if dist < radius
            goal_nodes(size(goal_nodes, 1) + 1, :) = [i, dist];
        end
    end
    
    if size(goal_nodes, 1) == 0
        path = [];
        length = 0;
        return;
    end
    
    length = 0;
    paths = transpose(goal_nodes(:, 1));
    while true % This is probably bad practice . . .
        for i = 1:size(paths, 2)
            curr_parent = parents(paths(length + 1, i));
            if curr_parent ~= 0
                paths(length + 2, i) = curr_parent;
            else
                % Found shortest path
                % It is possible that some tie and are later ignored . . .
                path = transpose(flip(paths(:, i)));
                return;
            end
        end
        length = length + 1;
    end