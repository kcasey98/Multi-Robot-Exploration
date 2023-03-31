close all; clearvars; clc
rng("default");
rng(28);

%%%%%%%%%%%%%%%%%%%%% Initializations %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% occupancy map thresholds explained
% 0-0.2: explored
% 0.2-0.6: frontier
% 0.6-0.8: unexplored
% 0.8-1.0: blocked

global dim;
dim = 30; % dimension of gridworld (Q-space)
n = 4; % number of robots
depth = 3; % lidar scan
z = zeros(dim,dim); % initialize map matrix

% assign initial positions
vacant_map = ones(dim,dim)*0.7; % 0.65 is unoccupied threshold

% initial positions
initial_pos = [1,29; 2,28; 3, 29; 2,26];
map = occupancyMap(vacant_map, RES = 1); % builds the map 
setOccupancy(map, initial_pos, 0.1); % 0.2 is occupied threshold

% Add obstacles
%%%%%%%%%%%%%% LARGE BLOCKS %%%%%%%%%%%%%%%%%%%%%%%%%
% obstacle_centres = [5,5; 25,25; 5,25; 25,5; 15, 15];
% obstacles = obstacle_centres(:,:);
% for i = 1:length(obstacle_centres)
%     obstacles = [obstacles; find_neighbors(obstacle_centres(i,:), 2,map)]; 
% end

%%%%%%%%%%%%% HOUSE LAYOUT %%%%%%%%%%%%%%%%%%%%%%%%%
% obstacles = [0:5; repmat(8, 1, 6)]';
% obstacles = [obstacles; [8:11; repmat(8, 1, 4)]'];
% obstacles = [obstacles; [13:15; repmat(8, 1, 3)]'];
% obstacles = [obstacles; [repmat(15, 1, 13); 18:30]'];
% obstacles = [obstacles; [repmat(26, 1, 13); 14:26]'];
% obstacles = [obstacles; [26:30; repmat(14, 1, 5)]'];
%obstacles = layout(dim, dim, 'rand_blocks');


%%%%%%%%%%%%%% NEW LAYOUT %%%%%%%%%%%%%%%%%%%%%%%%%%
obstacles = [0:5; repmat(8, 1, 6)]';
obstacles = [obstacles; [8:12; repmat(8, 1, 5)]'];
obstacles = [obstacles; [repmat(10, 1, 9); 0:8]'];
obstacles = [obstacles; [15:27; repmat(8, 1, 13)]'];
obstacles = [obstacles; [repmat(17, 1, 9); 0:8]'];
obstacles = [obstacles; [repmat(15, 1, 13); 18:30]'];
obstacles = [obstacles; [18:26; repmat(18, 1, 9)]'];
obstacles = [obstacles; [repmat(26, 1, 13); 14:26]'];
obstacles = [obstacles; [repmat(26, 1, 2); 29:30]'];
obstacles = [obstacles; [26:30; repmat(14, 1, 5)]'];

%%%%%%%%%%%%%% NEW LARGER LAYOUT %%%%%%%%%%%%%%%%%%%%%%%%%%
% obstacles = [0:0.5:5; repmat(8, 1, 11)]';
% obstacles = [obstacles; [8:0.5:12; repmat(8, 1, 9)]'];
% obstacles = [obstacles; [repmat(10, 1, 17); 0:0.5:8]'];
% obstacles = [obstacles; [15:0.5:27; repmat(8, 1, 25)]'];
% obstacles = [obstacles; [repmat(17, 1, 17); 0:0.5:8]'];
% obstacles = [obstacles; [repmat(15, 1, 25); 18:0.5:30]'];
% obstacles = [obstacles; [18:0.5:26; repmat(18, 1, 17)]'];
% obstacles = [obstacles; [repmat(26, 1, 25); 14:0.5:26]'];
% obstacles = [obstacles; [repmat(26, 1, 3); 29:0.5:30]'];
% obstacles = [obstacles; [26:0.5:30; repmat(14, 1, 9)]'];
% obstacles = obstacles*2;

% Set obstacle occupancy
setOccupancy(map, obstacles, 0.9);

%%%%%%%%%%%%% Get the coordinates of the occupied cells %%%%%%%%%%%%%
current_map = occupancyMatrix(map);
[row, col] = find(current_map < 0.2); 

 % Get robot positions as coordinates
points = grid2world(map,[row,col]);

%%%%%%%%%%%%%% Build binary map for A star  and in_view functions %%%%%%%%%%%%%%%%%%%%
%%%%%%% bmap 1 for astar %%%%%%%%%%
%%% Find obstacles
[row,col] = find(current_map > 0.8); % only walls are obstacles
new_coord = [row,col];
rws = size(new_coord);
oness = ones(rws(1),1);
new_coord = [new_coord(:,2),oness*(dim+1) - new_coord(:,1)];
obs_cells = grid2world(map,new_coord); % obstacles
% % binary
bmap1 = binaryOccupancyMap(zeros(dim,dim)); %initialize binary
setOccupancy(bmap1, obs_cells, 1); % set obstacles



%%%%%%% in_view map: bmap2 %%%%%%%%%
obs = grid2world(map,[row,col]);
% % binary
bmap2 = binaryOccupancyMap(zeros(dim,dim));
setOccupancy(bmap2, obs, 1); % set obstacles


%%%%% set frontier points and explored region
[frontier_cells, map] = set_frontier_and_neighbors(map,bmap2,points,n, depth);
pause(1);


%%%%%%%%%%%%%% Plot Initial State of Gridworld %%%%%%%%%%%%%%%%%%%%%%
figure(1)
sz = depth*2 + 1;
show(map);
hold on

% plot the level sets
plot_level_set(dim)

% plot the lidar region boundary
for i = 1:n
    rectangle('Position', [points(i,1)-sz/2 points(i,2)-sz/2 sz sz], 'EdgeColor', 'r');
end

% plot the robot positions
scatter(points(:,1), points(:,2),'b')
bounds = [0, 0;0, dim;dim, 0;dim, dim];
% plot voronoi boundaries
[vorvx,pos] = voronoi(points, bounds, n, 2);
for i = 1:size(pos,1)
plot(vorvx{i}(:,1),vorvx{i}(:,2),'-g')
end
grid on
set(gca, 'XTick', 0:1:dim, 'YTick', 0:1:dim)
set(gca, 'XTickLabel', [], 'YTickLabel', [])
hold off
pause(0.5);

%%%%%%%%%%%%%%%%% Planning for Exploration %%%%%%%%%%%%%%%%%%%%%

sigma = 4; % variance for distance component
vel = 1; % velocity
% loop through time

% keep track of trajectories
x_trajectories = points(:,1);
y_trajectories = points(:,2);

%keep track of explored area
explored_area = [];

% keep track of density
explored_density = [];

%keep track of deltas
prev_deltas = [0 0 0 0];
delta_t = [0 0 0 0];
deltas = delta_t;

for t = 1:10000
    current_map = occupancyMatrix(map);
    % finding the frontier
    [row,col] = find(current_map > 0.2 & current_map < 0.6);

    % track explored area
    [expl_x,expl_y] = find(current_map < 0.2);
    explored_area = [explored_area,size(expl_x,1)];
    
    % convert grid to world coord for all discovered cells
    explored = grid2world(map,[expl_x,expl_y]);

    explored_density = [explored_density, sum_of_density(explored)];

%     [unx,uny] = find(current_map > 0.6 & current_map < 0.8);
        
    % check if map is fully observed %%%%%% @kyle
    if isempty([row,col])
        break;
    end
    
    % convert frontier grid to world coordinates
    frontier_cells = grid2world(map,[row,col]);
    
    % assign frontier cells to voronoi regions
    qx = repmat(frontier_cells(:,1), [1, n]);
    qy = repmat(frontier_cells(:,2), [1, n]);
    siz = size(qx);
    len = siz(1); % no. of frontier cells
    rx = repmat(points(:,1)', [len, 1]);
    ry = repmat(points(:,2)', [len, 1]);
    dist_matrix = (qx-rx).^2 + (qy-ry).^2;
    [~, assigned_region] = min(dist_matrix, [], 2);

    next_positions = [];
    next_deltas = [];
    
    %%%%%%%%%%%%%%%% Finding next position for robots ################
    % iterating through the robots
    for i = 1:n
        rob = points(i,:);
        delta = delta_t(t,i);
        p_delt = prev_deltas(i);

        % find which frontier cells are closer to robot i
        indices = find(assigned_region == i); 
        rob_frontier = frontier_cells(indices,:);

        %stop slow
        siz = size(rob_frontier,1);
        for qindex = 1:siz
            q = rob_frontier(qindex,:); % frontier cell
            if ~in_view(q,rob,bmap2) % it's not visible
                [ang,q] = astar_transform(bmap1,dim,rob,q,i,bmap2,map); % do transform
                rob_frontier(qindex,:) = q; %replace q with new transform
            end
        end

        % if no frontier points for robot: Stay at current pos 
        % need to get rid of this if considering all frontiers a little
        if isempty(rob_frontier)
            next_positions = [next_positions; rob];
            next_deltas = [next_deltas;delta];
            continue;
        end
        
        %%%%%%%%%% Optimization to find best delta %%%%%%%%
        perf = 0; % performance
        siz = size(rob_frontier);
        num_fr = siz(1);
        q_attraction = zeros(num_fr,1); % this keeps track of attraction of frontier cells
        for qindex = 1:num_fr
            q = rob_frontier(qindex,:);

            % set feature density
            % Option 1: Uniform density
            density = 1; 
            % Option 2: Mixture of gaussians
            
            %density = density_function(q);

            % take into account delta
            ang = atan2(q(2) - rob(2), q(1) - rob(1));
            alpha = ang - delta;
            alpha = wrap_angle(alpha);
            theta = 1;
            %perf = perf + exp(-((alpha - 99*alpha/100)^2)/(2*(theta^2)))*exp(-norm(q-rob,2)^2/(2*(sigma^2)))*density;
            
            %perf = exp(-norm(q-rob,2)^2/(2*(sigma^2)))*density;
            %perf = (1/norm(q-rob))*density;

            %new
            diff_delt = delta - p_delt;
            theta_2 = 1;
            perf = perf + exp(-(diff_delt^2)/(2*(theta_2^2)))*exp(-(alpha^2)/(2*(theta^2)))*exp(-norm(q-rob,2)^2/(2*(sigma^2)))*density;


            q_attraction(qindex) = perf;
        end

        % Initialize Forces
        Fx = 0;
        Fy = 0;
        

        % Stay away from other robots you can see in certain radius
        other_robots = [];
        for r = 1:size(points,1)
            or = points(r,:);
            if r~=i && in_view(or,rob,bmap2)
                other_robots = [other_robots;or];
            end
        end
        coeff2 = ones(size(other_robots,1),2)*10; %tune
        Fx2 = 0; Fy2 = 0;
        if ~isempty(other_robots)
            [Fx2,Fy2] = obforce(rob(1),rob(2),other_robots(:,1),other_robots(:,2),6,coeff2);
        end

        % Go towards goal
        alpha = -20000; %tune
        beta = -19200; %tune
        [Fx_goal,Fy_goal] = obforce(rob(1),rob(2),rob_frontier(:,1),rob_frontier(:,2),4*dim,[alpha*q_attraction,beta*q_attraction]);

        % Find forces  
        Fx = Fx + Fx2 + Fx_goal;
        Fy = Fy + Fy2 + Fy_goal;
        theta_robot = atan2(Fy, Fx);

        v = 1;
        %make sure it stays on grid
        while true
            if check_bounds_and_collision(map, rob, vel, theta_robot, dim)
                next_pos = [vel*round(cos(theta_robot)) + rob(1), vel*round(sin(theta_robot)) + rob(2)];
                break;
            else 
                theta_robot = theta_robot + pi/4;
            end
        end
        next_positions = [next_positions; next_pos];
        next_deltas = [next_deltas; theta_robot];

    end

    % save old positions
    prev_positions = points;
    % update robot positions
    points = next_positions;
    % add new points to trajectory
    x_trajectories = [x_trajectories,points(:,1)];
    y_trajectories = [y_trajectories,points(:,2)];


    % save old deltas
    prev_deltas = deltas;
    % update robot deltas
    deltas = next_deltas;
    % add new deltas
    delta_t = [delta_t;deltas'];

    % mark new explored area to map and plot
    [frontier_cells, map] = set_frontier_and_neighbors(map,bmap2,points,n, depth); 
    
    %%%%%%%%% Plot everything %%%%%%%%
    figure(1)
    % loop through the points
    sz = depth * 2 + 1;
    show(map);
    hold on
    % plot the level sets
    plot_level_set(dim)
    for i = 1:n
        rectangle('Position', [points(i,1)-sz/2 points(i,2)-sz/2 sz sz], 'EdgeColor', 'r');
        plot(x_trajectories(i,:),y_trajectories(i,:),'LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',10);
    end
    scatter(points(:,1), points(:,2),'b')

    bounds = [0 0;0 dim+1;dim+1 -1;dim+1 dim+1];
    [vorvx,pos] = voronoi(points,bounds,n,2);
    for i = 1:size(pos,1)
        plot(vorvx{i}(:,1),vorvx{i}(:,2),'-g')
    end
    grid on
    set(gca, 'XTick', 0:1:dim, 'YTick', 0:1:dim)
    set(gca, 'XTickLabel', [], 'YTickLabel', [])
    hold off
    pause(0.01);
end

%%% Show time
disp(t)

%%%%%%%%%%%%% Plot data for analysis %%%%%%%%%%%%%%%

% Plot Area over Time
figure(2)
plot(0:1:t-1,explored_area,'LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
% Add a title and legend
title('Explored Area Over Time');
xlabel('Time')
ylabel('Explored Area')

% Plot Density over Time
figure(3)
plot(0:1:t-1,explored_density,'LineWidth',2,'MarkerEdgeColor','k','MarkerFaceColor','g','MarkerSize',10)
% Add a title and legend
title('Explored Density Over Time');
xlabel('Time')
ylabel('Explored Density')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save the variables to a .mat file
t_a = t;
area_a = explored_area;
density_a = explored_density;

% Save to MAT file
save('af_house_upperleft1.mat', 't_a', 'area_a', 'density_a');

%%%%%%%%%%%%%%%%% Helper Functions %%%%%%%%%%%%%%%%%%%%%


%%%%%%% Potential fields %%%%%%

% repel force for walls and for other robots, also gives attraction force
% for frontier cells
function [Fx,Fy] = obforce(x_robot,y_robot,x_obstacle,y_obstacle,r_repel,beta_array)
    Fx = 0; 
    Fy = 0;
    for i = 1:length(x_obstacle)
        alpha = beta_array(i,1);
        beta = beta_array(i,2);
        dist_obstacle = sqrt((x_robot - x_obstacle(i))^2 + (y_robot - y_obstacle(i))^2);
        if dist_obstacle < r_repel
            Fx = Fx + alpha*(x_robot - x_obstacle(i))*(1/dist_obstacle - 1/r_repel)^2;
            Fy = Fy + beta*(y_robot - y_obstacle(i))*(1/dist_obstacle - 1/r_repel)^2;
        end
    end
end

%%%%%%%%%%%% End of potential specific %%%%%%%%%%%%

% Wrap alpha to [-pi, pi]
function angle = wrap_angle(alpha)
    angle = alpha;
    if alpha > pi
        angle = alpha - 2*pi;
    elseif alpha < -pi
        angle = alpha + 2*pi;
    end
end

% Check if next position is within bounds
function safe_motion = check_bounds_and_collision(map, rob, vel, delta, dim)
    limit = 0.5;
    x_new = vel*round(cos(delta)) + rob(1);
    y_new = vel*round(sin(delta)) + rob(2);
    no_collision = getOccupancy(map, [x_new, y_new]) < 0.8;
    in_bounds = x_new >= limit && x_new <= dim-limit && y_new >= limit && y_new <= dim-limit; 
    safe_motion = no_collision && in_bounds;
end

% check if point is in bounds
function in_bounds = check_bounds(x_new,y_new)
    limit = 0.5;
    dim = 30;
    in_bounds = x_new >= limit && x_new <= dim-limit && y_new >= limit && y_new <= dim-limit; 
end
    
% Finds the neighbors of a cell
function neighbors = find_neighbors(cell, depth,bmap2)
    x = cell(1);
    y = cell(2);
    neighbors = [];
    for i = -depth:1:depth
        for j = -depth:1:depth
            q = [x + i, y + j];
            %&& in_view(q,cell,map)
            if ~(i ==0 && j == 0) && check_bounds(q(1),q(2)) && in_view(q,cell,bmap2)
                neighbors = [neighbors; x + i, y + j];
            end
        end
    end
    neighbors = [neighbors;cell];
end

% Finds the frontier points of a cell
function frontier = find_frontier(cell, depth,bmap2)
    x = cell(1);
    y = cell(2);
    dist = depth + 1;
    frontier = [];
    for i = -dist:1:dist
        for j = -dist:1:dist
            q = [x + i, y + j];
            %  && in_view(q,cell,map)
            if (abs(i) == dist || abs(j) == dist) && check_bounds(q(1),q(2)) && in_view(q,cell,bmap2)
                frontier = [frontier; x + i, y + j];
            end
        end
    end
end

% Fills explored region and sets frontier 
function [frontier_cells, map] = set_frontier_and_neighbors(map,bmap2, points, n, depth) 
    % mark new explored area to map and plot
    frontier_cells = [];
    explored_cells = [];
    for k = 1:n
        explored_cells = [explored_cells;find_neighbors(points(k,:), depth,bmap2)]; % get explored cells
        frontier_k = find_frontier(points(k,:), depth,bmap2); % get frontiers 
        frontier_cells = [frontier_cells;frontier_k];
    end
    
    % add to explored set (check for obstacles)
    possibly_explored = getOccupancy(map, explored_cells);
    actual_explored = possibly_explored < 0.8; % not a wall
    explored_cells = explored_cells(actual_explored, :);
    setOccupancy(map, explored_cells, 0.1); % explored

    % add to frontier set 
    possible_frontier = getOccupancy(map, frontier_cells); 
    actual_frontier = possible_frontier > 0.6 & possible_frontier < 0.8; % unexplored and not blocked
    frontier_cells = frontier_cells(actual_frontier,:);
    if ~isempty(frontier_cells)
        setOccupancy(map, frontier_cells, 0.4); % set frontier
    end
end

% Find the density function (Mixture of gaussians)
function density = density_function(q)
    density = 0;
    centres = [3,3;7,20;25,3;20,25];
    sigma = 3;
    % Sum of gaussians
    for j = 1:4
        c = centres(j,:);
        density = density + exp(-norm(c-q, 2)^2/(2 * sigma));
    end
    density = density * 50;
end

% Calculates the sum of the densities over time
function density_sum = sum_of_density(q_array)
    siz = size(q_array,1);
    density_sum = 0;
    for i = 1:siz
        q = q_array(i,:);
        density_sum = density_sum + density_function(q);
        %density_sum = density_sum + 1; % use this when density is 1
    end
end

% Plot the contour
function plot_level_set(dim)
    % Define the range of x and y values
    x = linspace(0,dim,dim);
    y = linspace(0,dim,dim);
    % Create a grid of x and y values
    [X,Y] = meshgrid(x,y);
    
    % Evaluate the density function at each point on the grid
    Z = zeros(size(X));
    for i = 1:numel(X)
        q = [X(i), Y(i)];
        Z(i) = density_function(q);
    end
    % Plot the contour
    contour(X,Y,Z,[0.1, 1.0, 10.0], "--", 'Color', 'k')
end

%find if cell is in view of robot
function view = in_view(q,rob,bmap)
    ang = atan2(q(2) - rob(2), q(1) - rob(1));
    dist = norm(q-rob,2);
    intpts = rayIntersection(bmap,[rob(1),rob(2),0],ang,dist);
    view = false;
    if all(isnan(intpts))
        view = true;
    end
end

% A star transformation function
function [ang,q_new] = astar_transform(bmap,dim,rob,q,i,bmap2,map)
    % Create plannerAStarGrid object with map
    planner = plannerAStarGrid(bmap);
    
    % shift 0.5
    rob = rob + [0.5 0.5];
    q = q + [0.5 0.5]; 

    % Find path between two grid coordinates
    pathRowCol = plan(planner, rob, q);

    % # steps of path
    d = size(pathRowCol,1) - 1;

    %rob2 = pathRowCol(round(sizes/2),:);
    % follow first step
    rob2 = pathRowCol(2,:);

    % angle of first segment on path
    ang = atan2(rob2(2) - rob(2), rob2(1) - rob(1));

    % new q 
    q_new = d*round([cos(ang),sin(ang)]) + rob - [0.5 0.5];

    %%%%%% got rid of rounding cause it doesnt work well
    q_new = [min(dim -0.5,q_new(1)),min(dim-0.5,q_new(2))]; %right and top bound
    q_new = [max(0.5,q_new(1)),max(0.5,q_new(2))]; %left and bottom bound
end
