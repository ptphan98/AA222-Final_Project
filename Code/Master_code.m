close all;
map = struct(); 

%% create nodes, definite coordinates, and add roadblocks
global n
n = 11;
map.nodes = 1:n*n; %create a 11x11 grid
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


%add roadblocks
roadblocks = [72 61;
              61 50;
              60 61;
              61 62;
              50 39;
              17 18];
real_map = add_roadblock(map,roadblocks);

%check 
plot_map(real_map,true)


%% breath_first_search
% start = [13];
% goal = 97; 
% map = real_map;
% 
% queue = start;
% % Tracks where an element came from. First element is current node, 2nd 
% % element is where node came from
% came_from = [start, 0];
% 
% while ~isempty(queue)
%     current = queue(1);
%     queue = queue(2:end);
%     
%     if current == goal
%        break  
%     end
%     for next = find_neighbors(map,current)
%         previous_nodes = came_from(:,1);
%         if isempty(find( next == previous_nodes,1 ))
%             queue(end+1) = next;
%             came_from = [came_from; [next current]];
%         end
%     end 
% end
% 
% %plot the expansion
% figure;
% hold on;
% axis equal
% xlim([0, n-1]);
% ylim([0, n-1]);
% for i = 2:size(came_from,1)
%     p1 = map.coords(:,:,came_from(i,2));
%     p2 = map.coords(:,:,came_from(i,1));
%     dp = p2 - p1;
%     quiver( p1(1), p1(2), dp(1), dp(2),'b','MaxHeadSize',1);
% end
% 
% %find path to a goal
% path = [];
% 
% current = goal;
% while current ~= start
%     path(end+1) = current;
%     current = came_from(find(current == came_from(:,1)),2);
% end
% path(end+1) = current;
% path = flip(path);
% 
% %plot path
% plot_map(map,false)
% for i = 2:length(path)
%     path(i)
%     path(i-1)
%     p1 = map.coords(:,:,path(i-1));
%     p2 = map.coords(:,:,path(i));
%     dp = p2 - p1;
%     quiver( p1(1), p1(2), dp(1), dp(2),'b','MaxHeadSize',1);
% end

%% Dijkstra’s Algorithm with equal weights
start = 13;
goal = 97; 
map = real_map;

queue = start;
% Tracks where an element came from. First element is current node, 2nd 
% element is where node came from
came_from = [start, 0];

while ~isempty(queue)
    current = queue(1);
    queue = queue(2:end);
    
    if current == goal
       break  
    end
    for next = find_neighbors(map,current)
        previous_nodes = came_from(:,1);
        if isempty(find( next == previous_nodes,1 ))
            queue(end+1) = next;
            came_from = [came_from; [next current]];
        end
    end 
end

%plot the expansion
figure;
hold on;
axis equal
xlim([0, n-1]);
ylim([0, n-1]);
for i = 2:size(came_from,1)
    p1 = map.coords(:,:,came_from(i,2));
    p2 = map.coords(:,:,came_from(i,1));
    dp = p2 - p1;
    quiver( p1(1), p1(2), dp(1), dp(2),'b','MaxHeadSize',1);
end

%find path to a goal
path = [];

current = goal;
while current ~= start
    path(end+1) = current;
    current = came_from(find(current == came_from(:,1)),2);
end
path(end+1) = current;
path = flip(path);

%plot path
plot_map(map,false)
for i = 2:length(path)
    path(i)
    path(i-1)
    p1 = map.coords(:,:,path(i-1));
    p2 = map.coords(:,:,path(i));
    dp = p2 - p1;
    quiver( p1(1), p1(2), dp(1), dp(2),'b','MaxHeadSize',1);
end


%% functions
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

function [neighbors] = find_neighbors(map,node)
    %finds neighbors next to node
    neighbors = [];
    for i = 1:size(map.edges,1)
        if map.edges(node,i) == 1
            neighbors(end+1) = i;
        end
    end
end

function [queue] = put_queue(t)

end