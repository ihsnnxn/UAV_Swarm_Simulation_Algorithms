function UAVBoidsFlockingSimuDarkV8()
% Parameters
numUAVs = 5;
formationType = 'V'; % Options: 'V', 'line'
windEnabled = true;

% Time
t = linspace(0, 45, 100);

% Lead UAV path (sinusoidal)
zLead = 20 * ones(size(t));
xLead = t * 10;            % extend path in X (wider view)
yLead = 25 * sin(0.3 * t); % smoother, visible sine wave

% Wind parameters
windAmplitude = 0.5;
windFrequency = 0.1;

% Formation offsets
offsets = generateFormationOffsets(numUAVs, formationType);

% UAV model
model = createAircraftModel();

% Create figures with black background
%fig3D = figure('Name','3D UAV Simulation','NumberTitle','off','Color','k');
figMap = figure('Name','2D Latitude-Longitude Map','NumberTitle','off','Color','k');

% Storage for trails
trailX = zeros(numUAVs, length(t));
trailY = zeros(numUAVs, length(t));

% --- Fixed zoom settings ---
xRange2D = [min(xLead)-20, max(xLead)+50];
yRange2D = [min(yLead)-20, max(yLead)+40];

%xRange3D = [min(xLead)-50, max(xLead)+20];
%yRange3D = [min(yLead)-50, max(yLead)+30];
%zRange3D = [min(zLead)-50, max(zLead)+20];

% Main animation loop
for i = 1:length(t)
    leadPos = [xLead(i), yLead(i), zLead(i)];

    if i < length(t)
        dir = [xLead(i+1)-xLead(i), yLead(i+1)-yLead(i), zLead(i+1)-zLead(i)];
    else
        dir = [1, 0, 0];
    end
    dir = dir / norm(dir);

    uavPositions = zeros(numUAVs, 3);
    for j = 1:numUAVs
        offset = offsets(j, :);
        pos = leadPos + offset * getFormationMatrix(dir);

        if windEnabled
            windShift = windAmplitude * sin(2 * pi * windFrequency * t(i) + j);
            pos = pos + [0, windShift, 0];
        end

        uavPositions(j,:) = pos;
        trailX(j,i) = pos(1);
        trailY(j,i) = pos(2);
    end

    %% --- 3D Figure ---
%%  figure(fig3D); clf;
%%  hold on; axis equal; grid on;
%%  set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');

    % Draw each UAV
%%  for j = 1:numUAVs
%%      drawAircraftModel(model, uavPositions(j,:), dir, 'w');
%%  end

    % Global camera (zoomed out to see all UAVs)
%%  view(35, 20);
%%  xlim(xRange3D);
%%  ylim(yRange3D);
%%  zlim(zRange3D);

%%  title(sprintf('3D UAV Formation  t = %.1fs', t(i)), 'Color', 'w');
%%  xlabel('Longitude', 'Color', 'w');
%%  ylabel('Latitude', 'Color', 'w');
%%  zlabel('Altitude (m)', 'Color', 'w');

    % Show trail paths faintly
%%  for j = 1:numUAVs
%%      plot3(trailX(j,1:i), trailY(j,1:i), zLead(1:i), '--', 'Color', [0.3 0.7 1 0.5]);
%%  end

%%  drawnow;

    %% --- 2D Map Figure ---
    figure(figMap); clf;
    hold on; axis equal; grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

    % Full trails visible
    for j = 1:numUAVs
        plot(trailX(j,1:i), trailY(j,1:i), '--', 'Color', [0.3 0.7 1]);
        plot(trailX(j,i), trailY(j,i), 'wo', 'MarkerFaceColor', 'w');
    end

    % Fixed wider view (see full sine motion)
    xlim(xRange2D);
    ylim(yRange2D);

    title(sprintf('Latitude-Longitude Map  t = %.1fs', t(i)), 'Color', 'w');
    xlabel('Longitude', 'Color', 'w');
    ylabel('Latitude', 'Color', 'w');

    legend(arrayfun(@(n) sprintf('UAV %d',n), 1:numUAVs, 'UniformOutput', false), ...
        'TextColor','w','Location','northeastoutside');

    drawnow;
    pause(0.01);
end
end

%% Helper: Get local rotation matrix based on direction
%%
%%
function R = getFormationMatrix(direction)
    z = [0 0 1];
    y = cross(z, direction); y = y / norm(y);
    x = cross(y, z);
    R = [x; y; z];
end

%% Helper: Create aircraft geometry
function model = createAircraftModel()
    model.fuselageLength = 3;
    model.fuselageRadius = 3;
    model.wingSpan = 4;
    model.tailSpan = 4;
    model.tailOffset = -0.7;
end

%% Helper: Draw UAV as mini fixed-wing aircraft
function drawAircraftModel(model, pos, dir, color)
    up = [0 0 1];
    right = cross(up, dir); right = right / norm(right);
    up = cross(dir, right);
    R = [right' up' dir'];

    T = @(p) (R * p')' + pos;

    % Fuselage
    [X, Y, Z] = cylinder(model.fuselageRadius, 12);
    Z = Z * model.fuselageLength - model.fuselageLength / 2;
    for i = 1:numel(X)
        pt = T([X(i), Y(i), Z(i)]);
        X(i) = pt(1); Y(i) = pt(2); Z(i) = pt(3);
    end
    surf(X, Y, Z, 'FaceColor', color, 'EdgeColor', 'none');

    % Wings
    wing = [0 0 0; -model.wingSpan/2 0 0; model.wingSpan/2 0 0];
    wing = T(wing);
    plot3(wing(:,1), wing(:,2), wing(:,3), '-', 'LineWidth', 3, 'Color', color);

    % Tail
    tail = [model.tailOffset 0 0; model.tailOffset -model.tailSpan/2 0; model.tailOffset model.tailSpan/2 0];
    tail = T(tail);
    plot3(tail(:,1), tail(:,2), tail(:,3), '-', 'LineWidth', 2, 'Color', color);
end

%% Helper: Generate relative offsets for formation
function offsets = generateFormationOffsets(N, type)
    offsets = zeros(N, 3);
    switch type
        case 'V'
            offsets(1,:) = [0, 0, 0];
            count = 1;
            for i = 1:floor((N-1)/2)
                count = count + 1;
                offsets(count, :) = [-i * 2.5, -i * 3, 0];
                if count < N
                    count = count + 1;
                    offsets(count, :) = [-i * 2.5, i * 3, 0];
                end
            end
        case 'line'
            for i = 1:N
                offsets(i, :) = [-i * 2.5, 0, 0];
            end
    end
end