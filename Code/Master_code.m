close all;

%% Define the grid map
global n void
n = 11;
map = struct(); 
map.nodes = 1:n*n; %create a nxn grid
map.coords = zeros(1,2,length(map.nodes)); 

%define a coordinate at each node. lowest at bottom left corner. Increase
%going right, then upward. 
element = 1;
for j = 0:n-1
    for k = 0:n-1
        map.coords(:,:,element) = [k,j];
        element = element + 1;
    end
end

%define edges. define inital map as a fully connected grid with no
%roadblocks
map.edges = zeros(length(map.nodes),length(map.nodes));
for i = 1:length(map.nodes)
    for j = 1:length(map.nodes)
        p1 = map.coords(:,:,i);
        p2 = map.coords(:,:,j);
        dist = norm(p1-p2);
        if dist == 1
            map.edges(i,j) = 1;
        else
            map.edges(i,j) = 0;
        end
    end
end

%define costs - Assume all roads have equal costs
map.costs = ones(length(map.nodes),length(map.nodes));

%add roadblocks 
roadblocks = [72 61;
              61 50;
              60 61;
              61 62;
              50 39;
              17 18;
              49 50;
              50 51];
void = [61 50]; %values we can never reach. Need to specify or A* will crash

map = add_roadblock(map,roadblocks);

%Define routes
%Each row represents a separate route. 1st column is start, 2nd column is
%end, 3rd column is weight
routes = [21 101 1]; 

%check 
plot_map(map,true)


%% Form and solve objective function
routes = [107 101 1;
          57 13 1;
          21 76 1]; 
n_stations = 3;     
      
A = [];
B = [];
Aeq = [];
Beq = [];
lb =[1 1 1 ];
ub =[121 121 121];
x = ga(@(x) objective_function(map,routes,x),n_stations,A,B,Aeq,Beq,lb,ub);
fuel_stops = round(x); %locations must be discrete

cost = objective_function(map,routes,fuel_stops)

%plot solution
plot_solution(map,routes,fuel_stops)

%% Define Objective Function

function [total_distance] = objective_function(map, routes, fuel_stops)
    global void
    
    %fuel_stops is an array that contains node locations
    fuel_stops = round(fuel_stops); %discrete optimization. Round to nearest whole number
    
    %check if fuel_stops are at valid locations
    valid = true;
    for i = 1:length(fuel_stops)
        if ~isempty( find(void == fuel_stops(i),1) )
            valid = false;
        end
    end
    
    if valid
        total_distance = 0;
        for i = 1:size(routes,1)
            distance = route_distance(map, routes(i,1), routes(i,2), fuel_stops);
            total_distance = total_distance + distance*routes(i,3);
        end
    else
        total_distance = 99999999;
    end
end

function [distance, path] = route_distance(map, point_a, point_b, fuel_stops)
    %Assume in order for a truck to complete any route, it must stop at a
    %fuel station along the way.
    
    %calculate all permutations of fuel stops - save the shortest
    shortest_path = []; 
    shortest_dist = [];
    for i = 1:length(fuel_stops)
        path_a2f = A_star(point_a, fuel_stops(i), map);
        path_f2b = A_star(fuel_stops(i), point_b, map);
        path = [path_a2f path_f2b];
        distance = length(path)-1; %number of edges required to complete path
        if i == 1 %store the first element
           shortest_path = path; 
           shortest_dist = distance;
        end
        if distance < shortest_dist %check if a new path is shorter
           shortest_path = path; 
           shortest_dist = distance;
        end
    end
    distance = shortest_dist;
    path = shortest_path;
end

%% A_star functions
function [path] = A_star(start, goal, map)
    global n
   
    queue = [start start];
    % Tracks where an element came from. First element is current node, 2nd 
    % element is where node came from
    came_from = containers.Map(start, start);
    cost_so_far = containers.Map(start, 0);

    while ~isempty(queue)
        current = queue(1,1); %grab highest priority node and pop it off
        queue = queue(2:end,:);

        if current == goal %early exit
           break  
        end

        neighbors = find_neighbors(map,current);
        for i = 1:size(neighbors,1)
            %previous_nodes = cell2mat(keys(came_from));
            next = neighbors(i,1);
            cost = neighbors(i,2);
            new_cost = cost_so_far(current) + cost;
            if ~isKey(cost_so_far,next) || new_cost < cost_so_far(next)
                cost_so_far(next) = new_cost;
                priority = new_cost + heuristic(map, next, goal);
                queue = priority_queue(queue, next, priority);
                came_from(next) = current;
            end
        end 
    end
    
    came_from_keys = cell2mat(keys(came_from));

    %find path to a goal
    path = [];

    current = goal;
    while current ~= start
        path(end+1) = current;
        index = current == came_from_keys;
        current = came_from(came_from_keys(index));
    end
    path(end+1) = current;
    path = flip(path);
    
    %plot the expansion - helpful for debugging
%     if isempty( find( index == 1))
%         figure;
%         hold on;
%         axis equal
%         xlim([0, n-1]);
%         ylim([0, n-1]);
%         for i = 1:length(came_from)
%             p1 = map.coords(:,:,came_from(came_from_keys(i)));
%             p2 = map.coords(:,:,came_from_keys(i));
%             dp = p2 - p1;
%             quiver( p1(1), p1(2), dp(1), dp(2),'b','MaxHeadSize',1);
%         end
%     end
end

function [neighbors] = find_neighbors(map,node)
    %finds neighbors next to node (1st column)
    %also returns the cost of travel (2nd column)
    neighbors = [];
    for i = 1:size(map.edges,1)
        if map.edges(node,i) == 1
            neighbors(end+1,:) = [i map.costs(node,i)];
        end
    end
end

function [new_queue] = priority_queue(queue, node, priority)
    new_queue = [queue; node priority];
    new_queue = sortrows(new_queue, 2);
end

function [distance] = heuristic(map,node,goal)
    p1 = map.coords(:,:,node);
    p2 = map.coords(:,:,goal);
    distance = abs(p1(1)-p2(1)) + abs(p1(2)-p2(2));
end

%% Plotting Functions

function [] = plot_solution(map,routes,fuel_stops)

    plot_map(map,false)
    route_colors = ['ob'; 'og'; 'or'; 'ok'; 'om'; 'oc'; 'oy'];
    route_colors2 = ['b'; 'g'; 'r'; 'k'; 'm'; 'c'; 'y'];
    for i = 1:size(routes,1)
        plot_route(map, routes(i,:), route_colors(i,:))
        [distance, path] = route_distance(map, routes(i,1), routes(i,2), fuel_stops);
        plot_path(map, path, route_colors2(i,:))
    end

    for i = 1:length(fuel_stops)
        plot_fuel(map,fuel_stops(i),'*k')
    end

end


function [] = plot_path(map, path, color)
    %expects p
    for i = 2:length(path)
        p1 = map.coords(:,:,path(i-1));
        p2 = map.coords(:,:,path(i));
        dp = p2 - p1;
        quiver( p1(1), p1(2), dp(1), dp(2),color,'MaxHeadSize',1);
    end
end

function [] = plot_route(map, routes, color)
    %expect routes to be only one row
    p1 = map.coords(:,:,routes(1));
    p2 = map.coords(:,:,routes(2));
    plot(p1(1),p1(2),color,'MarkerSize',10,'LineWidth',2);
    text(p1(1)+0.1,p1(2)+0.1,'Start','VerticalAlignment','bottom','HorizontalAlignment','left')
    plot(p2(1),p2(2),color,'MarkerSize',10,'LineWidth',2);
    text(p2(1)+0.1,p2(2)+0.1,'End','VerticalAlignment','bottom','HorizontalAlignment','left')
end

function [] = plot_fuel(map, fuel_stop, color)
    %expect routes to be only one row
    p1 = map.coords(:,:,fuel_stop(1));
    plot(p1(1),p1(2),color,'MarkerSize',10,'LineWidth',2);
    text(p1(1)+0.1,p1(2)+0.1,'Fuel','VerticalAlignment','bottom','HorizontalAlignment','left')
end

function [] = plot_map(map,label_node)
    %plot grid to check if valid
    global n
    figure;
    hold on;
    axis equal
    xlim([0, n-1]);
    ylim([0, n-1]);
    for i = 1:length(map.nodes)
        for j = 1:length(map.nodes)
            if map.edges(i,j) == 1
                p1 = map.coords(:,:,i);
                p2 = map.coords(:,:,j);
                plot([p1(1) p2(1)],[p1(2) p2(2)],'k');
            end
        end
    end
    if label_node
        %label each node appropriately
        for i = 1:length(map.nodes)
            p = map.coords(:,:,i);
            plot(p(1),p(2));
            text(p(1),p(2),num2str(i),'VerticalAlignment','bottom','HorizontalAlignment','left')
        end
    end
end

%% Map functions

function [new_map] = add_roadblock(map,x)
    %adds a road block between nodes
    %expects x to be an N x 2 matrix, with each row corresponding to
    %a closed edge
    new_map = map;
    for i = 1:size(x,1)
        node1 = x(i,1);
        node2 = x(i,2);
        new_map.edges(node1,node2) = 0;
        new_map.edges(node2,node1) = 0;
    end
end

function [new_map] = add_cost(map,x,cost)
    %adds a road block between nodes
    %expects x to be an N x 2 matrix, with each row corresponding to
    %a closed edge
    new_map = map;
    for i = 1:size(x,1)
        node1 = x(i,1);
        node2 = x(i,2);
        new_map.costs(node1,node2) = cost;
        new_map.costs(node2,node1) = cost;
    end
end

