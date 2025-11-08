function floydsimple()
% FLOYD_WARSHALL_DRONE_VISUAL
% Visual Floyd–Warshall simulation with weighted graph and single drone.
% Shows weights on edges, simulates thinking phase, then animates flight
% along the shortest path with a green trail.

clc; clear; close all;

%% --- Node Setup ---
nodes = [1 1;   % A
         3 2;   % B
         5 1;   % C
         7 3;   % D
         2 5;   % E
         4 6;   % F
         6 5;   % G
         8 6];  % H

labels = {'A','B','C','D','E','F','G','H'};
n = length(nodes);

% --- Define edges and weights (symmetric graph) ---
W = inf(n);
for i = 1:n
    W(i,i) = 0;
end

addEdge(1,2,2.2);   % A-B
addEdge(2,3,2.3);   % B-C
addEdge(3,4,2.4);   % C-D
addEdge(1,5,4.1);   % A-E
addEdge(5,6,2.1);   % E-F
addEdge(6,7,2.0);   % F-G
addEdge(7,8,2.2);   % G-H
addEdge(4,7,1.9);   % D-G
addEdge(2,6,3.2);   % B-F
addEdge(3,7,3.1);   % C-G
addEdge(5,3,3.5);   % E-C

    function addEdge(i,j,w)
        W(i,j) = w;
        W(j,i) = w;
    end

%% --- Figure Setup ---
fig = figure('Name','Floyd–Warshall Shortest Path (Weighted)','Color','w');
hold on; grid on; axis equal;
xlim([0 9]); ylim([0 7]);
xlabel('X'); ylabel('Y');
title('Floyd–Warshall Shortest Path Simulation','FontSize',12);

% Plot nodes
for i = 1:n
    plot(nodes(i,1), nodes(i,2), 'ko', 'MarkerFaceColor','b', 'MarkerSize',10);
    text(nodes(i,1)+0.15, nodes(i,2)+0.15, labels{i}, 'FontWeight','bold');
end

% Plot static edges and weights
edgeHandles = [];
for i = 1:n
    for j = i+1:n
        if isfinite(W(i,j)) && W(i,j) > 0
            edgeHandles(end+1) = plot([nodes(i,1), nodes(j,1)], ...
                                      [nodes(i,2), nodes(j,2)], ...
                                      'Color',[0.7 0.7 0.7], 'LineStyle','--');
            % Add weight label
            mid = (nodes(i,:) + nodes(j,:))/2;
            text(mid(1), mid(2), sprintf('%.1f', W(i,j)), ...
                'Color',[0.2 0.2 0.2],'FontSize',9,'HorizontalAlignment','center');
        end
    end
end

%% --- Algorithm Setup ---
startNode = 1; % A
endNode   = 8; % H

[D,P] = initializeDP(W);
textHandle = text(0.5,6.5,'Running Floyd–Warshall...','FontSize',11,'Color','b');

% Create blue lines for updates (so we can remove later)
tempLines = [];

%% --- Simulated Floyd–Warshall Calculation ---
for k = 1:n
    for i = 1:n
        for j = 1:n
            if D(i,k) + D(k,j) < D(i,j)
                % Visualize this update temporarily in blue
                tempLines(end+1) = plot([nodes(i,1), nodes(j,1)], ...
                    [nodes(i,2), nodes(j,2)], 'b-', 'LineWidth', 2);
                pause(0.03);

                % Update tables
                D(i,j) = D(i,k) + D(k,j);
                P(i,j) = P(k,j);
            end
        end
    end
end

pause(0.3);

% Remove temporary blue testing lines
delete(tempLines);

%% --- Highlight Shortest Path ---
path = reconstructPath(P, startNode, endNode);

for k = 1:length(path)-1
    i = path(k); j = path(k+1);
    plot([nodes(i,1), nodes(j,1)], [nodes(i,2), nodes(j,2)], ...
        'Color',[0 0.8 0], 'LineWidth',3.5); % green path
end

set(textHandle,'String',sprintf('Shortest Path Found: %s', ...
    strjoin(labels(path),' → ')), 'Color','k');

fprintf('\nShortest Path: %s\n', strjoin(labels(path),' → '));
fprintf('Total Distance = %.2f units\n', D(startNode,endNode));

%% --- Drone Animation ---
drone = plot(nodes(startNode,1), nodes(startNode,2), 'ro', ...
              'MarkerFaceColor','g', 'MarkerSize',10);
trailX = []; trailY = [];

textProgress = text(0.5,6.1,'','FontSize',11,'Color',[0 0.6 0]);

for k = 2:length(path)
    x1 = nodes(path(k-1),1); y1 = nodes(path(k-1),2);
    x2 = nodes(path(k),1);   y2 = nodes(path(k),2);
    steps = 100;
    x = linspace(x1,x2,steps);
    y = linspace(y1,y2,steps);
    for t = 1:steps
        drone.XData = x(t);
        drone.YData = y(t);
        trailX = [trailX, x(t)];
        trailY = [trailY, y(t)];
        plot(trailX, trailY, 'Color',[0 0.7 0], 'LineWidth',1.5);
        pause(0.02);
    end
    set(textProgress,'String',sprintf('Moving: %s → %s', ...
        labels{path(k-1)}, labels{path(k)}));
end

set(textProgress,'String',sprintf('Arrived at %s!', labels{endNode}), 'Color','r');
plot(nodes(endNode,1), nodes(endNode,2), 'ro', 'MarkerFaceColor','r', 'MarkerSize',12);

end


%% === Helper Functions ===
%%
function [D,P] = initializeDP(W)
n = size(W,1);
D = W; P = zeros(n);
for i = 1:n
    for j = 1:n
        if i~=j && isfinite(W(i,j))
            P(i,j) = i;
        end
    end
end
end

function path = reconstructPath(P,i,j)
if P(i,j)==0
    if i==j
        path = i;
    else
        path = [];
    end
else
    path = [reconstructPath(P,i,P(i,j)) j];
end
end