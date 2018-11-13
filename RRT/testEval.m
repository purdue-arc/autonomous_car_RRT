state_tree = [[0 0];[0 1];[0 2];[1 0];[1 1];[1 2];[2 0];[2 1];[2 2]];
parents = [0 1 2 1 4 5 4 7 8];
goal = [2 2];
radius = 1.1;
[path, length] = evaluateTree(state_tree, parents, goal, radius);

path
length