clear
for Exp_No = 0:0
    %load pre-planned path
    load('3dpath.mat');
    %% DECLARATION OF THE WORKSPACE: Indoor Environment
    %% Declare the obstacles: x
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
    
    %% Display the workspace:
    fig = figure();
    fig.WindowState = 'maximized';
    subplot(1,2,1)
    axis([0 1000 0 1000 0 1000])
    hold on;
    title('Path execution in 3D with RRT* replanning')
    hold on;
    plot3(xpath,ypath,zpath, '-k');
    hold on
    drawMesh(vertices1, faces1,'c'); hold on; light;
    drawMesh(vertices2, faces2,'g'); hold on; light;
    drawCylinder(cylinder, 'FaceColor','y'); light;
    hold on
    
    %subplot for relative distance plot
    subplot(2,2,2)
    axis([0,inf, 0,inf])
    title('Plot of Relative Distance between drone and obstacle against time ')
    xlabel('Timeslice')
    ylabel('Relative Distance')
    yline(42,'-.r');
    
    
    %Initialisation of obstacle destinations
    Obs_D_x = [100, 355, 400, 100, 285, 355, 160, 650, 650, 235, 715, 400, 300, 250]; %[285, 160];
    Obs_D_y = [100, 275, 200, 100, 230, 275, 100, 450, 520, 200, 560, 300, 250, 200];%[230, 100];%
    Obs_D_z = [200, 180, 500, 100, 200, 180, 135, 450, 400, 170, 410, 300, 200, 170]; %[200, 135];%
    
    %Obstacle start point
    random = randi(length(Obs_D_x));
    obstacle = [Obs_D_x(random),Obs_D_y(random),Obs_D_z(random),30]; %% obstacle at (x, y) with a side of 20
    start_obs = random;
    
    %obstacle speed and robot speed, pixels per frame
    obs_units = 8;
    rob_units = 8;
    
    %Initialisation of robot
    robot = [xpath(1), ypath(1),zpath(1)];
    robot_goal = [xpath(length(xpath)), ypath(length(ypath)),zpath(length(zpath))];
    j=1;
    
    counter_obs = 1;    %counter to plot obstacle position
    counter_rob = 1;    %counter to plot robot position
    complete_obs = 1;   %to tigger timesliced plot coordinates of obstacle
    complete_rob = 1;   %to tigger timesliced plot coordinates of robot
    i = 1;              % for overall while loop, for getframe
    timestep = 0;       %to observe obstacle for 2 timestep
    replan = 0;         %to trigger replan
    b = 1;              %for multicolour plot
    decrement = 0;      %for multiplier
    
    Execution_time = tic;
    while robot ~= robot_goal
        %Generate timesliced plot coordinates of obstacle
        if complete_obs == 1
            
            des_obs = randi(length(Obs_D_x));  %randomly generated obstacle vertice
            magnitude_obs = pdist([Obs_D_x(des_obs),Obs_D_y(des_obs), Obs_D_z(des_obs); Obs_D_x(start_obs),Obs_D_y(start_obs), Obs_D_z(start_obs)]);
            div = ceil(magnitude_obs/obs_units); %scale divisons according to units (speed)
            tx = linspace(Obs_D_x(start_obs),Obs_D_x(des_obs),div); %divide x2-x1 into the number of divisons
            ty = linspace(Obs_D_y(start_obs),Obs_D_y(des_obs),div); %divide y2-y1 into the number of divisons
            tz = linspace(Obs_D_z(start_obs),Obs_D_z(des_obs),div); %divide z2-z1 into the number of divisons
            start_obs = des_obs;   %destination of obstacle to become the start point after obstacle completes its path
            complete_obs = 0;
        end
        
        %Generate timesliced plot coordinates for robot
        if complete_rob == 1
            magnitude_rob = pdist([xpath(j),ypath(j),zpath(j); xpath(j+1),ypath(j+1),zpath(j+1)]);
            div = ceil(magnitude_rob/rob_units);
            robot_tx = linspace(xpath(j), xpath(j+1), div);
            robot_ty = linspace(ypath(j), ypath(j+1), div);
            robot_tz = linspace(zpath(j), zpath(j+1), div);
            j = j+1;
            complete_rob = 0;
            counter_rob = 1;
        end
        subplot(1,2,1)
        %update & plot obstacle position
        if counter_obs < length(tx)    %as long as counter is within array of tx, continue
            obstacle(1) = tx(counter_obs);
            obstacle(2) = ty(counter_obs);
            obstacle(3) = tz(counter_obs);
            obs = drawCube([obstacle(1),obstacle(2),obstacle(3),30]);
            counter_obs = counter_obs + 1;
        elseif magnitude_obs == 0      %when randomly generated vertice is the same position as initial position
            obstacle(1) = Obs_D_x(des_obs);
            obstacle(2) = Obs_D_y(des_obs);
            obstacle(3) = Obs_D_z(des_obs);
            obs = drawCube([obstacle(1),obstacle(2),obstacle(3),30]);
            counter_obs = 1;        %reset counter to 1
            complete_obs = 1;         %generate next timesliced plot coordinates of obstacles
        else                          %when counter will exceeds size of array of tx the next timeslice
            obstacle(1) = tx(counter_obs);
            obstacle(2) = ty(counter_obs);
            obstacle(3) = tz(counter_obs);
            obs = drawCube([obstacle(1),obstacle(2),obstacle(3),30]);
            counter_obs = 1;        %reset counter to 1
            complete_obs = 1;       %generate next timesliced plot coordinates of obstacles
        end
        obs_vertices = obs.Vertices;    %obstacle vertices
        obs_faces = obs.Faces;          %obstacle faces
        
        %update & plot robot position
        if counter_rob < length(robot_tx)      %as long as counter is within array, continue
            robot(1) = robot_tx(counter_rob);
            robot(2) = robot_ty(counter_rob);
            robot(3) = robot_tz(counter_rob);
            Drone = drawSphere(robot(1),robot(2),robot(3),20, 'FaceColor','r');
            counter_rob = counter_rob +1;
            
        elseif counter_rob == length(robot_tx)    %when counter will exceeds size of array in the next timeslice
            robot(1) = robot_tx(counter_rob);
            robot(2) = robot_ty(counter_rob);
            robot(3) = robot_tz(counter_rob);
            Drone = drawSphere(robot(1),robot(2),robot(3),20, 'FaceColor','r');
            counter_rob = 1;                    %reset counter to 1
            complete_rob = 1;                   %generate next timesliced plot coordinates of robot
            
        elseif magnitude_rob == 0     %to account for path replanning where goal of replanned path = node on old path
            Drone = drawSphere(robot(1),robot(2),robot(3),20, 'FaceColor','r');
            complete_rob =1;
        end
        
        F(i) = getframe(fig);
        
        %check for path obstruction within range
        threshold = 150;     %range of to begin tracking obstacle
        
        if pdist([robot(1),robot(2),robot(3); obstacle(1),obstacle(2),obstacle(3)]) < threshold
            timestep = timestep + 1;
            if timestep > 0
                X_robot(timestep) = robot(1);           %%storing variables
                Y_robot(timestep) = robot(2);
                Z_robot(timestep) = robot(3);
                X_obs(timestep) = obstacle(1);
                Y_obs(timestep) = obstacle(2);
                Z_obs(timestep) = obstacle(3);
            end
            if timestep == 2
                if ~pdist([X_obs(1) Y_obs(1) Z_obs(1); X_obs(2) Y_obs(2) Z_obs(2)])
                    timestep = timestep - 1
                end
            end
            
            if timestep == 2
                %vectors are reference wrt to origin
                
                %Robot velocity vector
                robot_velocity_vector = [(X_robot(2) - X_robot(1)),(Y_robot(2)- Y_robot(1)),(Z_robot(2) - Z_robot(1))];
                
                %obstacle velocity vector
                obs_velocity_vector = [(X_obs(2) - X_obs(1)),(Y_obs(2)- Y_obs(1)) ,  (Z_obs(2) - Z_obs(1))];
                
                %check if vectors are (almost) parallel (similar directions)
                Parallel_check = robot_velocity_vector ./ obs_velocity_vector
                
                if Parallel_check > 0 & Parallel_check > (0.80*Parallel_check(1)) & Parallel_check < (1.20*Parallel_check(1))
                    txt = 'moving in similar directions, continue';
                    t = text(0,0,150,txt);
                    status{i} = 'similar'
                    pause(0.01)
                    %delete(t)
                    
                    %double check if drone and obstacle are too close
                    if pdist([robot(1),robot(2),robot(3); obstacle(1),obstacle(2),obstacle(3)]) < 120
                        delete(t)
                        t = text(0,0,150,'too close! Replan!');
                        status{i} = 'similar direction but too close'
                        pause(0.01)
                        %delete(t)
                        replan = 1;
                    end
                else
                    %{
                %check if they are moving towards each other (concept of intersecting lines in 3D space)
                %METHOD 1: exact point of intersection (rarely occurs)
                syms l s
                line_rob = robot' + l*robot_velocity_vector';
                line_obs = obstacle(1:3)' + s*obs_velocity_vector';
                
                eqns = [line_rob(1) == line_obs(1), line_rob(2) == line_obs(2),line_rob(3) == line_obs(3)];
                S = solve(eqns, s,l);
                sol = [S.l; S.s]; %S.l and S.s are the values of parameter 'l' and 's'.
                             
                if sol
                    l = S.l
                    s = S.s
                    line_rob = robot' + l*robot_velocity_vector'
                    line_obs = obstacle(1:3)' + s*obs_velocity_vector'
                    txt = 'impending collision';
                    t = text(0,0,150,txt);
                    pause(1)
                    %delete(t)
                    replan = 1;
                    %}
                    
                    %check if they are moving towards each other (concept of intersecting lines in 3D space)
                    %METHOD 2: linsolve gives the least squared solution Ax = B
                    A = [obs_velocity_vector', -robot_velocity_vector'];
                    B = robot' - obstacle(1:3)';
                    leastsquared = linsolve(A,B)
                    rob_intersection = robot' + leastsquared(2) * robot_velocity_vector'
                    obs_intersection = obstacle(1:3)' + leastsquared(1) * obs_velocity_vector'
                    error = abs(rob_intersection-obs_intersection) ./ obs_intersection
                    
                    if error < 0.08 & pdist([xpath(j),ypath(j),zpath(j); obs_intersection(1),obs_intersection(2),obs_intersection(3)]) < 50 %& pdist([robot(1),robot(2),robot(3); obs_intersection(1),obs_intersection(2),obs_intersection(3)]) < 50
                        txt = 'impending collision, replan!';
                        t = text(0,0,150,txt);
                        status{i} = 'impending collision'
                        pause(0.01)
                        %delete(t)
                        replan = 1;
                        
                    else
                        txt = 'skew lines, continue';
                        t = text(0,0,150,txt);
                        status{i} = 'skew lines'
                        pause(0.01)
                        %delete(t)
                        
                        %check for the case when drone and obstacle are moving
                        %in skew lines but are too close
                        if pdist([robot(1),robot(2),robot(3); obstacle(1),obstacle(2),obstacle(3)]) < 120
                            delete(t)
                            t = text(0,0,150,'too close! Replan!');
                            status{i} = 'skew lines but too close'
                            pause(0.01)
                            %delete(t)
                            replan = 1;
                        end
                    end
                    
                    
                end
                timestep = -1
                F(i) = getframe(fig);
                delete(t);
            end
        end
        
        %replan
        if replan == 1
            obstructed_node = [xpath(j),ypath(j),zpath(j)];
            obstructed_path = [xpath(j:end); ypath(j:end); zpath(j:end)];
            if j+3 < length(xpath)   %Use nodes that are 3 steps away
                node_counter = j+3;
                %to account for replanning cases where nodes are too close, use nodes that are further away
                while pdist([xpath(j),ypath(j),zpath(j); xpath(node_counter),ypath(node_counter),zpath(node_counter)]) < 150 & node_counter < length(xpath)
                    node_counter = node_counter +1;
                end
                replan_goal = [xpath(node_counter) ypath(node_counter) zpath(node_counter)];
            else
                node_counter = j;
                replan_goal = [xpath(j) ypath(j) zpath(j)];  %for cases where obstacle blocks the goal node
            end
            
            %RRT3d replan based on obstacle trajectory
            multiplier(i) = pdist([robot(1),robot(2),robot(3); obstacle(1),obstacle(2),obstacle(3)])/ (obs_units*1.15)   %divide by 10
            
            %start timer
            tic
            while true
                %predict obstacle future location based on multiplier
                obs_coord = [(X_obs(2)+ multiplier(i)*(X_obs(2) - X_obs(1))),(Y_obs(2) + multiplier(i)*(Y_obs(2)- Y_obs(1))), (Z_obs(2) + multiplier(i)*(Z_obs(2)- Z_obs(1)))];
                %obs_coord2 = [(X_obs(2)+ multiplier(i)*(1.765)*(X_obs(2) - X_obs(1))),(Y_obs(2) + multiplier(i)*(1.765)*(Y_obs(2)- Y_obs(1))), (Z_obs(2) + multiplier(i)*(1.765)*(Z_obs(2)- Z_obs(1)))];
                %{
                while pdist([X_robot(2) Y_robot(2) Z_robot(2);obs_coord]) < 60 && decrement == 0
                    multiplier(i) = multiplier(i) + 0.5
                    obs_coord = [(X_obs(2)+ multiplier(i)*(X_obs(2) - X_obs(1))),(Y_obs(2) + multiplier(i)*(Y_obs(2)- Y_obs(1))), (Z_obs(2) + multiplier(i)*(Z_obs(2)- Z_obs(1)))];
                    if multiplier(i) > 20
                        decrement = 1;
                        multiplier(i) = multiplier(i) - 7;
                        break
                    end
                end
                %}
                while pdist([X_robot(2) Y_robot(2) Z_robot(2);obs_coord]) < 42 % && decrement == 1
                    multiplier(i) = multiplier(i) - 0.5
                    obs_coord = [(X_obs(2)+ multiplier(i)*(X_obs(2) - X_obs(1))),(Y_obs(2) + multiplier(i)*(Y_obs(2)- Y_obs(1))), (Z_obs(2) + multiplier(i)*(Z_obs(2)- Z_obs(1)))];
                    if multiplier(i) < -12
                        break
                    end
                end
                
                while pdist([replan_goal;obs_coord]) < 80 & node_counter < length(xpath)
                    node_counter =  node_counter + 1
                    replan_goal = [xpath(node_counter) ypath(node_counter) zpath(node_counter)];
                end
                %{
                while pdist([X_robot(2) Y_robot(2) Z_robot(2);obs_coord2]) < 42 % && decrement == 1
                    multiplier(i) = multiplier(i) - 0.5
                    obs_coord2 = [(X_obs(2)+ multiplier(i)*(1.765)*(X_obs(2) - X_obs(1))),(Y_obs(2) + multiplier(i)*(1.765)*(Y_obs(2)- Y_obs(1))), (Z_obs(2) + multiplier(i)*(1.765)*(Z_obs(2)- Z_obs(1)))];
                    if multiplier(i) < -12
                        break
                    end
                end
                
                while pdist([replan_goal;obs_coord2]) < 80 & node_counter < length(xpath)
                    node_counter =  node_counter + 1
                    replan_goal = [xpath(node_counter) ypath(node_counter) zpath(node_counter)];
                end
                %}
                
                %call rrt3d_replan
                replanned_path{i} = rrtstar3d_replan([X_robot(2) Y_robot(2) Z_robot(2)], replan_goal,obs_coord, obs_velocity_vector);
 
                %make sure replanned path is valid and multiplier is non
                %negative
                if 1 %length(replanned_path{i}(1,:)) > 2 | multiplier(i) < -5 | 1
                    break;
                end
                %prepare for the next loop, reduce multiplier
                multiplier(i) = multiplier(i) -0.5
            end
            Time_Elapse(i) = toc;
            % if rrt3d_replan used another node further down the path
            % if replanned_path{i}(4) > 1
            %     node_counter = node_counter + replanned_path{i}(4)
            % end
            
            %for RRT replan path does not include goal node, for RRT*
            %replan path consist of rounded off goal node value
            replanpath_nodes(i) = length(replanned_path{i}(1,:));
            nodes_explored(i) = replanned_path{i}(5,1);
            
            if length(replanned_path{i}(1,:)) > 2
                %Plot new replanned paths
                hold on
                C = {'r','m','b','c','k'};
                
                
                if isempty(replanned_path{i}) ~=1 & length(replanned_path{i})> 2
                    plot3 ([replanned_path{i}(1,:), xpath(node_counter)] ,[replanned_path{i}(2,:), ypath(node_counter)],[replanned_path{i}(3,:),zpath(node_counter)], 'color', C{b}, 'LineWidth', 2)
                    b = b+1;
                    if b == 5
                        b = 1;
                    end
                end
                replanpathcost(i) = 0;
                %calculating path cost
                    No_of_nodes = size(replanned_path{i});
                for var = 1 : No_of_nodes(2)
                   if var < No_of_nodes(2)
                       replanpathcost(i) = replanpathcost(i) + pdist([replanned_path{i}(1:3, var)' ; replanned_path{i}(1:3, var+1)']);
                   else
                       replanpathcost(i) = replanpathcost(i) + pdist([replanned_path{i}(1:3, var)' ; replan_goal]);
                   end 
                end
       
                hold on
                replan =0;
                decrement = 0;
                
                %rewire, insert new planned path into path
                
                xpath = [xpath(1:j-1), replanned_path{i}(1,:) xpath(node_counter:end)];
                ypath = [ypath(1:j-1), replanned_path{i}(2,:) ypath(node_counter:end)];
                zpath = [zpath(1:j-1), replanned_path{i}(3,:) zpath(node_counter:end)];
                complete_rob = 1
            else
                %planning failed, observe for 1 more timestep
                status{i} = ['replan failed ' status{i}]
                timestep = 1
                X_robot(timestep) = robot(1);
                Y_robot(timestep) = robot(2);
                Z_robot(timestep) = robot(3);
                X_obs(timestep) = obstacle(1);
                Y_obs(timestep) = obstacle(2);
                Z_obs(timestep) = obstacle(3);
                replanpathcost(i) = 0;
            end
        else
            Time_Elapse(i) = 0;
            multiplier(i) = 0;
            replanpath_nodes(i) = 0;
            nodes_explored(i) = 0;
            replanpathcost(i) = 0;
        end
        
        if robot ~= robot_goal | magnitude_rob == 0
            delete(Drone);
            delete(obs);
            
        end
        %data collection
        relativedist(i) = pdist([robot(1),robot(2),robot(3); obstacle(1),obstacle(2),obstacle(3)]);
        
        %update plot live
        subplot(2,2,2)
        hold on;
        plot(i,relativedist(i),'-o')
        
        if relativedist(i) < 42
            status{i} = 'COLLIDED'
            break
        end
        
        i = i+1 %used in replanned path and plotting
        
    end
    Total_time(length(Time_Elapse)) = toc(Execution_time);
    %plotting of time taken for path replanning
    subplot(2,2,4)
    bar(Time_Elapse)
    title('Time taken for path planning ')
    xlabel('Timeslice')
    ylabel('Planning Time')
    
    %saving data into excel
    filename = 'rrtstar_anglebasedonvelocity60degrees.xlsx';
    timestep = (1:length(relativedist))'
    relativedist = relativedist'
    Time_Elapse = Time_Elapse'
    Total_time = Total_time'
    status{i-1} = '';
    status = status'
    multiplier = multiplier'
    replanpath_nodes = replanpath_nodes'
    nodes_explored = nodes_explored'
    replanpathcost = replanpathcost'
    T = table(timestep,relativedist, Time_Elapse, status, replanpath_nodes, replanpathcost, nodes_explored, multiplier, Total_time)
    
    
    %Alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    %Letter = Alphabet(15) %for multiple Letter = Alphabet([9,3,4])
    Letter=xlsColNum2Str(Exp_No*9+1);
    writetable(T,filename, 'Range', [Letter{1} '1'])
    
    %to record a few more frames
    for i = i: i+20
        F(i) = getframe(fig);
    end
    
    %playback & record frames as movie
    %{
fig = figure();
fig.WindowState = 'maximized';
axes('Position',[0 0 1 1])
movie(F)
    %}
    %record in avi format
    %{
v = VideoWriter('Simulation3.avi');
open(v);
writeVideo(v,F)
close(v);
    %}
    num = num2str(Exp_No)
    v = VideoWriter(strcat('rrtstar_anglebasedonvelocity60degrees',num), 'MPEG-4');
    v.FrameRate = 15;
    v.Quality = 100;
    open(v);
    writeVideo(v,F)
    close(v);
    
    close(fig)
    clear
end