% Testing the robot
% https://www.mathworks.com/help/robotics/examples/path-following-for-differential-drive-robot.html

% UAV waypoint follower
% https://www.mathworks.com/help/robotics/ref/uavwaypointfollower-system-object.html

% clc
% clear
% close all
% 
% % Robot start and end locations
% r_start = [1 10];
% r_end = [19 10];
% 
% for map_num = 1:6
%     % Load the map file
%     filename = sprintf('maps/map%d.txt',map_num);
%     simpleMap = readmatrix(filename);
%     
%     % Change the map file to an occupancy grid
%     map = robotics.OccupancyGrid(simpleMap,1);
% 
%     % Use PRM to randomly populate the map with points
%     prm = robotics.PRM(map,1000);
%     prm.ConnectionDistance = 2;
%     
%     % Find the best path from start to end
%     path = findpath(prm,r_start,r_end);
%     
%     % Display the map and path
%     figure(map_num)
%     show(prm)
% end
% 
% robotCurrentLocation = path(1,:);
% robotGoal = path(end,:);
% initialOrientation = 0;
% robotCurrentPose = [robotCurrentLocation initialOrientation];
% 
% robotRadius = 0.4;
% robot = ExampleHelperRobotSimulator('emptyMap',2);
% robot.enableLaser(false);
% robot.setRobotSize(robotRadius);
% robot.showTrajectory(true);
% robot.setRobotPose(robotCurrentPose);
% 
% plot(path(:,1), path(:,2),'k--d')
% xlim([0 20])
% ylim([0 20])
% 
% controller = robotics.PurePursuit;
% controller.Waypoints = path;
% controller.DesiredLinearVelocity = 2;
% controller.MaxAngularVelocity = 2;
% controller.LookaheadDistance = 0.3;
% 
% goalRadius = 0.1;
% distanceToGoal = norm(robotCurrentLocation - robotGoal);
% 
% controlRate = robotics.Rate(10);
% pause(10)
% while( distanceToGoal > goalRadius )
%     
%     % Compute the controller outputs, i.e., the inputs to the robot
%     [v, omega] = controller(robot.getRobotPose);
%     
%     % Simulate the robot using the controller outputs.
%     drive(robot, v, omega);
%     
%     % Extract current location information ([X,Y]) from the current pose of the
%     % robot
%     robotCurrentPose = robot.getRobotPose;
%     
%     % Re-compute the distance to the goal
%     distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
%     
%     waitfor(controlRate);
%     
% end










clc
clear
close all

% Robot start and end locations
r_start = [1 10];
r_end = [19 10];

for map_num = 1:6
    % Load the map file
    filename = sprintf('maps/map%d.txt',map_num);
    map = readmatrix(filename);

    % Use PRM to randomly populate the map with points
    p = PRM(map);
    p.plan('npoints',100);
    
    % Find a path
    path = p.query(r_start, r_end);
    path_num = 0;
    
%     % while we can find a path
%     while not(isequal(path,[r_start,r_end]))
%         path_num = path_num + 1 ;
%     
%         % remove the current path from the edgelist in PRM
%         for edge_num = 1:length(path)
%             e = transpose(path(edge_num,:));
%             for edge_num_prm = 1:length(p.graph.edgelist)
%                 if isequal(e,p.graph.edgelist(:,edge_num_prm))
%                     p.graph.edgelist(:,edge_num_prm) = [];
%                 end
%             end
%         end
%         
%         % find a new path
%         path = p.query(r_start, r_end);
%         disp(fprintf('Found %d possible paths\n',path_num));
     
    % Plot the result
    figure(map_num)
    p.plot()
end

% 
% clc
% clear
% close all
% 
% % Robot start and end locations
% r_start = [1 10];
% r_end = [19 10];
% 
% for map_num = 1:1
%     % Load the map file
%     filename = sprintf('map%d.txt',map_num);
%     map = readmatrix(filename);
% 
%     % Use PRM to randomly populate the map with points
%     p = PRM(map);
%     p.plan('npoints',5,'distthresh',100);
%     path = p.query(r_start, r_end);
%     
%     % Get the vertices, edges
%     V = p.graph.vertexlist;
%     E = p.graph.edgelist;
% 
%     % Remove self edges
%     e_i = 1;
%     while e_i <= length(E(1,:))
%         if isequal(E(1,e_i), E(2, e_i))
%             % Remove element
%             E(:,e_i) = [];
%         else
%             % Go to next element
%             e_i = e_i + 1;
%         end
%     end
%     
%     C = zeros(1,length(E(1,:)));
%     
%     % Calculate the cost of each edge
%     for e_i = 1:length(E(1,:))
%         % Get the vertices location
%         v1 = E(1,e_i);
%         v2 = E(2,e_i);
%         x1 = V(1,v1);
%         x2 = V(1,v2);
%         y1 = V(2,v1);
%         y2 = V(2,v2);
%         % Calculate the euclidian distance
%         euclideanDistance = sqrt((x2-x1)^2+(y2-y1)^2);
%         % Save the distance
%         C(1,e_i) = euclideanDistance;
%     end
% 
%     
%     %[dist,path,pred] = graphshortestpath(G,S,D);
%     
%     figure(map_num)
%     p.plot()
% end


% clc
% clear
% close all
% 
% % Robot start and end locations
% r_start = [1 10];
% r_end = [19 10];
% 
% for map_num = 1:1
%     % Load the map file
%     filename = sprintf('maps/map%d.txt',map_num);
%     map = readmatrix(filename);
% 
%     p = myPRM(map, 10);
%     p.plan()
%     p.getVerticies()
% end