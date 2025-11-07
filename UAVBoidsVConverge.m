function UAVBoidsVConverge()
% UAVBoidsVFlockingV3
% 5 UAVs start from scattered positions, converge into a V formation,
% then maintain that V while following a sinusoidal path.
% Dark background + smooth flocking motion.

%% === Parameters ===
numUAVs = 5;
formationType = 'V';
windEnabled = true;

% Time
t = linspace(0, 50, 300); % longer simulation for convergence + flight

% Lead UAV path (sinusoidal)
xLead = t * 20;             % forward motion (speed)
yLead = 30 * sin(0.25 * t); % sinusoidal lateral motion
zLead = 20 * ones(size(t));

% Wind parameters
windAmplitude = 0.5;
windFrequency = 0.1;

% Generate formation offsets
offsets = generateFormationOffsets(numUAVs, formationType);

% --- Randomized starting positions ---
rng(2); % reproducible randomness
startSpread = 500; % how far away they start
startPositions = (rand(numUAVs,3) - 0.5) .* [startSpread, startSpread, 0];

% Visualization setup
figMap = figure('Name','UAV V Formation (2D Map)','NumberTitle','off','Color','k');
xRange2D = [min(xLead)-100, max(xLead)+100];
yRange2D = [min(yLead)-100, max(yLead)+100];

% Trails
trailX = zeros(numUAVs, length(t));
trailY = zeros(numUAVs, length(t));

% Convergence time (fraction of total)
convergeFraction = 0.25; 
convergeSteps = floor(convergeFraction * length(t));

%% === MAIN SIMULATION LOOP ===
for i = 1:length(t)
    leadPos = [xLead(i), yLead(i), zLead(i)];

    % Compute heading direction
    if i < length(t)
        dir = [xLead(i+1)-xLead(i), yLead(i+1)-yLead(i), zLead(i+1)-zLead(i)];
    else
        dir = [1, 0, 0];
    end
    dir = dir / norm(dir);
    R = getFormationMatrix(dir);

    % Compute target V-formation positions
    targetPositions = zeros(numUAVs, 3);
    for j = 1:numUAVs
        offset = offsets(j, :);
        targetPositions(j,:) = leadPos + offset * R;
    end

    % Interpolate UAV positions between random start and formation
    uavPositions = zeros(numUAVs, 3);
    for j = 1:numUAVs
        if i < convergeSteps
            % Blend from start position → formation
            alpha = i / convergeSteps;
            pos = (1-alpha) * startPositions(j,:) + alpha * targetPositions(j,:);
        else
            pos = targetPositions(j,:);
        end

        % Apply wind disturbance
        if windEnabled
            windShift = windAmplitude * sin(2*pi*windFrequency*t(i) + j);
            pos = pos + [0, windShift, 0];
        end

        uavPositions(j,:) = pos;
        trailX(j,i) = pos(1);
        trailY(j,i) = pos(2);
    end

    %% === Visualization ===
    figure(figMap); clf;
    hold on; axis equal; grid on;
    set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');

    % Plot full leader path
    plot(xLead, yLead, '--', 'Color', [0.2 0.5 1 0.4]);

    % Plot UAVs + trails
    for j = 1:numUAVs
        plot(trailX(j,1:i), trailY(j,1:i), '-', 'Color', [0.3 0.7 1], 'LineWidth', 1.2);
        plot(trailX(j,i), trailY(j,i), 'wo', 'MarkerFaceColor', 'w', 'MarkerSize', 7);
    end

    % View settings
    xlim(xRange2D); ylim(yRange2D);
    title(sprintf('UAVs Converging + Flying in V Formation | t = %.1fs', t(i)), 'Color', 'w');
    xlabel('Longitude', 'Color', 'w');
    ylabel('Latitude', 'Color', 'w');

    drawnow;
    pause(0.02);
end
end


%% === Helper: Get local rotation matrix based on direction ===
%%
function R = getFormationMatrix(direction)
z = [0 0 1];
y = cross(z, direction);
if norm(y) < 1e-6, y = [0 1 0]; end
y = y / norm(y);
x = cross(y, z);
R = [x; y; z];
end


%% === Helper: Generate V or Line formation offsets ===
function offsets = generateFormationOffsets(N, type)
offsets = zeros(N, 3);
switch type
    case 'V'
        offsets(1,:) = [0, 0, 0]; % leader
        count = 1;
        for i = 1:floor((N-1)/2)
            count = count + 1;
            offsets(count, :) = [-i * 20, -i * 12, 0]; % left wing
            if count < N
                count = count + 1;
                offsets(count, :) = [-i * 20, i * 12, 0]; % right wing
            end
        end
    case 'line'
        for i = 1:N
            offsets(i,:) = [-i * 15, 0, 0];
        end
end
end