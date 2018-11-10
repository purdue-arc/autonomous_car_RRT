function [updatedTree, newChildren] = extend(tree, x_rand, childrenIndices)
    numNodes = size(tree, 1);
    dt = 0.1;
    length = 0.4;
    width = 0.2;
    mass = 4;
    inertia = 1;
    
    cf = 0.1; %front cornering stiffness coeff
    cr = 0.1; %rear cornering stiffness coeff
    lf = length/2; %distance from center of gravity to front wheel
    lr = length/2; %distance from center of gravity to rear wheel

    steeringAngle = x_rand(1);
    vx = x_rand(2);
    newChildren = [];
    %TODO: we need to add a way to store which node to run our random control vector
    %This runs through whole tree and finds nearest point
    for j = childrenIndices
        minDis = 1000000;
        state = tree(j,:);
        index = 1;
        for i = 1:1:numNodes
            if(i ~= j)
                dist = sqrt( (state(1)-tree(i,1))^2 + (state(2)-tree(i,2))^2 );
                if (dist < minDis)
                    minDis = dist;
                    index = i;
                end
            end
        end
        
        index
        x_new = integrater(tree(index, :), dt, length, width, mass, vx, cf, cr, lf, lr, inertia, steeringAngle);

        numNodes = numNodes + 1;
        newChildren = [newChildren, numNodes];
        newChildren
        plot(x_new(1), x_new(2), '*');
        line([x_new(1), tree(index,1)], [x_new(2), tree(index,2)]);
        tree(numNodes, :) = x_new;
    end
    
    
    
    
    updatedTree = tree;
end
