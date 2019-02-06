function num_parents = countParents(tree, index)
%COUNTPARENTS Recursive function to count parents in tree
    if index ~= 0
        num_parents = 1 + countParents(tree, tree(index));
    else
        num_parents = -1;
    end
end

