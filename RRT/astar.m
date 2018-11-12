function [path, length] = astar(state_tree, parents, goal, radius)
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
    
   goal_nodes
   path = 0;
   length = 0;
end