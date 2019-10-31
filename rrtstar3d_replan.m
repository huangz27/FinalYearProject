%clearvars
%close all
%q_start_coord = [307.5216  263.7932  203.2480];
%q_goal_coord = [473   341   285];
%obs_coord = [321.7731  247.8487  247.8487];

function replanned_path = rrtstar3d_replan(q_start_coord, q_goal_coord, obs_coord, obs_velocity_vector);
%% DECLARATION OF THE WORKSPACE: Indoor Environment
%% Declare the obstacles:
%obstacle 1: Wall with a small slit defined using meshes
vertices1 = [750, 0, 0; 750, 0, 1000; 750, 1000, 1000; 750, 1000, 0;...
    800, 1000, 0; 800, 1000, 1000; 800, 0, 1000; 800, 0, 0;...
    750, 500, 400; 750, 500, 700; 750, 800, 700; 750, 800, 300;...
    800, 800, 300; 800, 800, 700; 800, 500, 700; 800, 500, 400];

faces1 = [1 2 7 8; 2 3 6 7; 3 4 5 6; 4 1 8 5; 1 9 10 2; 10 11 3 2; 11 12 4 3;...
    12 9 1 4; 8 16 15 7; 15 14 6 7; 14 13 5 6; 13 16 8 5; 9 10 15 16;...
    10 11 14 15; 11 12 13 14; 9 16 13 12;];

% obstacle 2 : Wall
vertices2 = [200, 300, 0; 250, 300, 0; 250, 1000, 0; 200, 1000, 0;...
    200, 300, 1000; 250, 300, 1000; 250, 1000, 1000; 200, 1000, 1000];
faces2 = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
% obstacle 3: pillar in the room
cylinder = [500 500 0 500 500 800 50];

%Moving obstacle
%obs = drawCube([obs_coord(1),obs_coord(2),obs_coord(3), 30]);
%obs_vertices = obs.Vertices;    %obstacle vertices
%obs_faces = obs.Faces;    %obstacle faces
%delete(obs);

%% Max and min for random point
max_x = floor(max([q_start_coord(1),q_goal_coord(1)])) + 50;
min_x = floor(min([q_start_coord(1),q_goal_coord(1)])) - 50;

max_y = floor(max([q_start_coord(2),q_goal_coord(2)])) + 50;
min_y = floor(min([q_start_coord(2),q_goal_coord(2)])) - 50;

max_z = floor(max([q_start_coord(3),q_goal_coord(3)])) + 50;
min_z = floor(min([q_start_coord(3),q_goal_coord(3)])) - 50;

next = 1;

EPS = 30;
numNodes = 1000;

%% Declare the Start and Goal points using the structure:
q_start.coord = q_start_coord;
q_start.parent = 0;
q_start.cost = 0;
q_goal.coord = floor(q_goal_coord);
q_goal.cost = 0;


nodes(1) = q_start;


for i = 1:1:numNodes
    q_rand = [randi([min_x, max_x]) randi([min_y, max_y]) randi([min_z, max_z])];
    % plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
        if pdist([nodes(j).coord; q_goal.coord]) < 30
            break
        end
    end
    
    if pdist([nodes(j).coord; q_goal.coord]) < 30
        break
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    
    
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer3d(q_rand, q_near.coord, val, EPS);
    
    %% Check the intersection of the obstacles with the line between closest
    %% node and the new node
    %%method 1
    %{
    linecheck=[q_near.coord q_new.coord];
    points1 = intersectLineMesh3d(linecheck, vertices1, faces1)
    points2 = intersectLineMesh3d(linecheck, vertices2, faces2)
    points3 = intersectLineCylinder(linecheck, cylinder)
    
    if ~isempty(points1) || ~isempty(points2) || ~isempty(points3) || pdist([obs_coord; q_new.coord]) < 70
        % loopcount = loopcount + 1;
        continue
    end
    %}
    %%method 2
    
    linecheck=[q_near.coord q_new.coord];
    
    distanceNew = pdist([obs_coord; q_new.coord]);
    if distanceNew < 100
    if distanceNew >= 60  
        distance = pdist([obs_coord; q_near.coord]);
        direction_vector = obs_velocity_vector / sqrt(obs_velocity_vector(1)^2 + obs_velocity_vector(2)^2 + obs_velocity_vector(3)^2);
        if distance < 60
          
          vector_v = [q_new.coord - q_near.coord];  
          path_direction_vector = vector_v / pdist([q_near.coord; q_new.coord]);
            
          angle = acos(dot(direction_vector,path_direction_vector));
          
            if angle > 2*pi/3 
            continue
            end
          %  vector_v = [q_new.coord - q_near.coord];
          % vector_w = [obs_coord - q_near.coord];
            
         %   c1 = dot(vector_v,vector_w);
            %c2 = dot(vector_v,vector_v);
            
        %    if c1 < 0 %-0.5*pdist([q_new.coord; q_near.coord])* pdist([obs_coord; q_near.coord])
                % shortest_distance = 80 %pdist([obs_coord; q_near.coord])
        %    else
         %       continue
        %    end
        else
            %{
            a = (q_new.coord(1) - q_near.coord(1))^2 + (q_new.coord(2) - q_near.coord(2))^2 + (q_new.coord(3) - q_near.coord(3))^2;
            b = -2*((q_new.coord(1) - q_near.coord(1))*(obs_coord(1)-q_near.coord(1)) + ...
                (q_new.coord(2) - q_near.coord(2))*(obs_coord(2)-q_near.coord(2)) + ...
                (q_new.coord(3) - q_near.coord(3))*(obs_coord(3)-q_near.coord(3)));
            c = (obs_coord(1)-q_near.coord(1))^2 + (obs_coord(2)-q_near.coord(2))^2 + (obs_coord(3)-q_near.coord(3))^2 - 60^2;
            
            if b^2-4*a*c > 0
                continue
            end
            %}
            if pdist([q_new.coord; q_goal.coord]) > 80 
            %direction_vector = obs_velocity_vector / sqrt(obs_velocity_vector(1)^2 + obs_velocity_vector(2)^2 + obs_velocity_vector(3)^2);
            %line_vector = [q_new.coord - q_near.coord];
            %projection = dot(direction_vector,line_vector);
            
           % obs_near_vector = [q_near.coord - obs_coord];
           % projection2 = dot(direction_vector, obs_near_vector);
            line_vector = [q_new.coord - obs_coord];
            projection = dot(direction_vector,line_vector);
            projection_point = obs_coord + projection*direction_vector; %+ projection2*direction_vector;
            if pdist([projection_point; q_new.coord]) < 60
            continue
            end
            end
        end
    else
        continue
    end
    end
    % elseif c1 <0
    %     continue
    %{
     elseif c2 <= c1
         shortest_distance = pdist([obs_coord; q_new.coord])
     else
     b = abs(c1)/c2
     point_coordinate = q_near.coord + b*vector_v
     shortest_distance = pdist([obs_coord; point_coordinate])
     end
     if shortest_distance < 60
         continue
     end
         else
             continue
         end
     end
    %}
    %WRONG
    %{
         if pdist([obs_coord; q_new.coord]) < 120
        vector_A = [q_new.coord - q_near.coord]
        vector_B = [obs_coord - q_near.coord] ./ pdist([obs_coord; q_near.coord])
        C = dot(vector_A,vector_B)
        sqrt((pdist([obs_coord; q_new.coord]))^2  - C^2)
        if sqrt((pdist([obs_coord; q_new.coord]))^2  - C^2) < 60
            continue
        end
    end
    %}
    points1 = intersectLineMesh3d(linecheck, vertices1, faces1);
    if ~isempty(points1)
        if points1(:,1) < min(q_near.coord(1), q_new.coord(1)) -30 | points1(:,1) > max(q_near.coord(1), q_new.coord(1)) +30 | points1(:,2) < min(q_near.coord(2), q_new.coord(2)) -30 | points1(:,2) > max(q_near.coord(2), q_new.coord(2)) +30 | points1(:,3) < min(q_near.coord(3), q_new.coord(3)) -30 | points1(:,3) > max(q_near.coord(3), q_new.coord(3)) +30
            points1 = [];
        end
    end
    points2 = intersectLineMesh3d(linecheck, vertices2, faces2);
    if ~isempty(points2)
        if points2(:,1) < min(q_near.coord(1), q_new.coord(1)) -30 | points2(:,1) > max(q_near.coord(1), q_new.coord(1)) +30 | points2(:,2) < min(q_near.coord(2), q_new.coord(2)) -30 | points2(:,2) > max(q_near.coord(2), q_new.coord(2)) +30 | points2(:,3) < min(q_near.coord(3), q_new.coord(3)) -30 | points2(:,3) > max(q_near.coord(3), q_new.coord(3)) +30
            points2 = [];
        end
    end
    points3 = intersectLineCylinder(linecheck, cylinder);
    loop = 0;
    if ~isempty(points3)
        if points3(:,1) < min(q_near.coord(1), q_new.coord(1)) | points3(:,1) > max(q_near.coord(1), q_new.coord(1)) | points3(:,2) < min(q_near.coord(2), q_new.coord(2)) | points3(:,2) > max(q_near.coord(2), q_new.coord(2)) | points3(:,3) < min(q_near.coord(3), q_new.coord(3)) | points3(:,3) > max(q_near.coord(3), q_new.coord(3))
            points3 = [];
        end
    end
    
    if ~isempty(points1) || ~isempty(points2) || ~isempty(points3) %|| loop %pdist([obs_coord; NewNode]) < 60
        % loopcount = loopcount + 1;
        continue
    end
    %}
    %line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
    %drawnow
    %hold on
    q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
    
    % Within a radius of r, find all existing nodes
    q_nearest = [];
    r = 50;
    neighbor_count = 1;
    for j = 1:1:length(nodes)
        if (dist_3d(nodes(j).coord, q_new.coord)) <= r
            q_nearest(neighbor_count).coord = nodes(j).coord;
            q_nearest(neighbor_count).cost = nodes(j).cost;
            neighbor_count = neighbor_count+1;
        end
    end
    
    % Initialize cost to currently known value
    q_min = q_near;
    C_min = q_new.cost;
    
    % Iterate through all nearest neighbors to find alternate lower
    % cost paths
    
    for k = 1:1:length(q_nearest)
        if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
            q_min = q_nearest(k);
            C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
             %line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');
            % hold on
        end
    end
    
    % Update parent to least cost-from node
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_min.coord
            q_new.parent = j;
        end
    end
    
    % Append to nodes
    nodes = [nodes q_new];
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
counter = 1;
while q_end.parent ~= 0
    start = q_end.parent;
    % line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
    % hold on
    
    x_path(counter) = q_end.coord(1);
    y_path(counter) = q_end.coord(2);
    z_path(counter) = q_end.coord(3);
    counter = counter + 1;
    q_end = nodes(start);
    
end
x_path = [x_path q_start_coord(1)]
y_path = [y_path q_start_coord(2)]
z_path = [z_path q_start_coord(3)]
replanned_path = [flip(x_path); flip(y_path); flip(z_path); ones(1,length(y_path)); ones(1,length(y_path))*i]
