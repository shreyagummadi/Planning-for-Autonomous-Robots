function [final_route] = DynamicMazeAstar(startx,starty)

C = 1.3; % Reasonable clearance
rs = 1.77; % Turtlebot2 robot radius


% Create obstacle space 
map = robotics.BinaryOccupancyGrid(250,150,1); %defining a 250x150 space
                                                    
% Define 20 random point across the map and define them as obstacles to 
% simulate unpredictable dynamic environment

xmin = 0;
xmax = 250;
n = 20;
x = xmin+rand(1,n)*(xmax-xmin);

ymin = 0;
ymax = 150;
n = 20;
y = ymin+rand(1,n)*(ymax-ymin);

xy = [x' y'];
setOccupancy(map,xy,1);

% Defining the maze
% Horizontal Obstacles

xx = 0:40;
yy = 110*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 40:55;
yy = 80*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 20:50;
yy = 50*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 10:50;
yy = 30*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 20:40;
yy = 15*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 80:95;
yy = 65*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 110:150;
yy = 130*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 100:125;
yy = 100*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 95:125;
yy = 20*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 125:155;
yy = 60*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 155:175;
yy = 80*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 220:250;
yy = 120*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 225:250;
yy = 60*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 160:175;
yy = 30*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);

xx = 205:225;
yy = 30*ones(size(xx));
xy = [xx' yy'];
setOccupancy(map,xy,1);
%%%%%%%%%%%%%

% Vertical Obstacles

yy = 120:150;
xx = 10*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 110:130;
xx = 40*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 50:110;
xx = 20*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 30:50;
xx = 50*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 15:30;
xx = 30*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 80:150;
xx = 55*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 50:80;
xx = 80*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 110:150;
xx = 150*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 100:120;
xx = 100*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 20:100;
xx = 125*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 0:20;
xx = 95*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 60:80;
xx = 155*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 80:110;
xx = 175*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 0:30;
xx = 175*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 30:70;
xx = 225*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);

yy = 100:150;
xx = 200*ones(size(yy));
xy = [xx' yy'];
setOccupancy(map,xy,1);


%%%%%%%%%%%%%%%%%%%%%%
inflate(map, C + rs)

map = occupancyMatrix(map);


input_map = map;

figure
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 0.9, 0.96]);
imshow(~map)
axis on
hold on


% [x_coords,y_coords] = ginput(2); %input location of start point then goal point
% x_coords = round(x_coords)
% y_coords = round(y_coords)


x_coords(1) = startx*10;
y_coords(1) = 150 - starty*10;

[x_coords(2),y_coords(2)] = ginput(1); %input location of start point then goal point
x_coords = round(x_coords);
y_coords = round(y_coords);

close all

%% Astar Algorithm initialization
% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - orange  - on list
% 5 - yellow - start
% 6 - green - destination
% 7 - grey - chosen route

cmap = [1 1 1; 
    0 0 0; 
    1 0 0; 
    1 165/255 0; 
    1 1 0; 
    0 1 0; 
    0.5 0.5 0.5];


colormap(cmap);

% variable to control if the map is being visualized on every
% iteration
mapAnim = false;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), y_coords(1), x_coords(1));  
dest_node  = sub2ind(size(map), y_coords(2),  x_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% meshgrid will `replicate grid vectors' nrows and ncols to produce
% a full grid

parent = zeros(nrows,ncols);


% 
maxside = max(nrows,ncols);
[X, Y] = meshgrid (1:maxside); %create an nxn meshgrid where n is the larger side

xd = y_coords(2);%dest_coords(1);   % 8
yd = x_coords(2);%dest_coords(2);   % 9

% Evaluate Heuristic function, H, for each grid cell
% Manhattan distance
H = abs(X - xd) + abs(Y - yd);

% Initialize cost arrays
f = Inf(nrows,ncols);
g = Inf(nrows,ncols);

g(start_node) = 0;       % distance between current node and start location
f(start_node) = H(start_node); % sum of g value + heurestic value of node

% keep track of the number of nodes that are expanded
numExpanded = 0;

q = 1;
% Main Loop
while true
           
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;

    if (mapAnim)
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 0.9, 0.96]);
        image(1.5, 1.5, map);
        grid on;
        axis image;

        drawnow;
    end
    
    % Find the node with the minimum f value
    [min_f, current] = min(f(:));
    
    if ((current == dest_node) || isinf(min_f))
        break;
    end
    
    % Update input_map
    map(current) = 3;
    f(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(f), current);

    % Visit all of the neighbors around the current node and update the
    % entries in the map, f, g and parent arrays

    numExpanded=numExpanded+1;
    reqd_neigh=[[i-1,j],[i+1,j],[i,j-1],[i,j+1],[i-1,j+1],[i+1,j+1],[i+1,j-1],[i-1,j-1]]; % 8-connected actions
    for t=1:8
        element=reqd_neigh(2*t-1:2*t);
        m=element(1);
        p=element(2);
        mm = m;
        pp = p;
        if m>0 && p>0 && m<nrows+1 && p<ncols+1
            if map(m,p)==1
                if g(m,p)>g(current)+1
                    g(m,p)=g(current)+1;
                    f(m,p)=g(m,p)+H(pp,mm);
                    parent(m,p)=current;
                    map(m,p)=4;
                    
                end
            elseif map(m,p)==6
                    g(m,p)=g(current)+1;
                    f(m,p)=g(m,p)+H(pp,mm);
                    parent(m,p)=current;
            end
        end
    end
end

%% Construct route from start to dest by following the parent links
if (isinf(f(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        
        route = [parent(route(1)), route];
    end

    % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1
        
        [yw(q),xw(q)] = ind2sub(size(map),route(k));
        yw(q) = 150 - yw(q);
        q = q + 1;
        
        final_route = [xw' yw'];
        
        map(route(k)) = 7;
        pause(0.001);
        set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 0.9, 0.96]);
        image(1.5, 1.5, map);
        grid on;
        axis image;       
    end
end


end

