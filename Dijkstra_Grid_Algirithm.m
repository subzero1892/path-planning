

function [route,num_of_cells_visited] = DijkstraGrid (input_map, start_coords, dest_coords)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dijkstra's algorithm on a grid.
% Inputs :
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. 
%    num_of_cells_visited: the total number of nodes visited during search
%    (excluding destination node).


% map codes to be used
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Extract no of rows and cols from input grid
[nrows, ncols] = size(input_map);

% map - a matrix that tracks the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Get linear indices of start and dest nodes relative to map
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% distanceFromStart - a matrix that tracks the distance of visited cells
% Initialize to infinity
distanceFromStart = Inf(nrows,ncols);

% parent - For each cell this matrix holds the index of its parent 
parent = zeros(nrows,ncols);


distanceFromStart(start_node) = 0;

% keep track of number of cells visited
num_of_cells_visited = 0;


    % function to update distance from start of nodes when traversing grid
    function update (i,j,d,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (distanceFromStart(i,j) > d) )
            distanceFromStart(i,j) = d;
            map(i,j) = 4;
            parent(i,j) = p;
        end
    end

% Main Loop
while true
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distanceFromStart(current) = Inf; % remove this node from further consideration
    
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distanceFromStart), current);
    

    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.   
    visit_and_update_neighbours (i,j,nrows,ncols,min_dist,current);
    % random test

    num_of_cells_visited = num_of_cells_visited + 1;
        
end

%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
end
end
