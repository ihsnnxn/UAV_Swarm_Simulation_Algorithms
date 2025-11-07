function UAVAvoidanceSine()
    %% === PARAMETERS ===
    numUAVs = 20;            % Formation UAV count
    dt = 0.05;               % Simulation step
    totalTime = 120;         % Longer flight
    speedFactor = 58;        % Leader forward speed

    % Formation parameters
    baseSpacing = 17;
    formationAngle = deg2rad(45);
    formationPullStrength = 0.2;
    repulsionStrength = 0.10;
    avoidanceStrength = 0.65;

    % Wave path
    waveAmplitude = 40;
    waveFrequency = 0.02;
    xEnd = 1000;     
    yCenter = 0;

    % === OBSTACLES ===
    obstacles = [
        120,  0, 15;
        220, -10, 12;
        280,  15, 10;
        350,  -8, 18;
        430,   5, 16;
        500, -12, 12;
        600,   8, 20;
        700, -6, 18;
        780,  12, 15;
        850,   0, 22;
        930, -10, 18
    ];

    % === ENEMY DRONES (TOP & BOTTOM) ===
    numEnemies = 10;
    enemySpeed = 65; % downward/upward diagonal
    xSpawn = linspace(0, xEnd, numEnemies); % distributed along X
    ySpawnTop = 200 * ones(1, numEnemies/2);
    ySpawnBottom = -200 * ones(1, numEnemies/2);
    enemyPositions = [xSpawn', [ySpawnTop, ySpawnBottom]'];
    enemyVelocities = zeros(numEnemies, 2);
    for e = 1:numEnemies
        if enemyPositions(e,2) > 0
            enemyVelocities(e,:) = [randn*3, -enemySpeed];  % from top down
        else
            enemyVelocities(e,:) = [randn*3, enemySpeed];   % from bottom up
        end
    end

    %% === INITIAL FORMATION ===
    positions = zeros(numUAVs, 2);
    velocities = zeros(numUAVs, 2);
    for i = 1:numUAVs
        positions(i,:) = [0, -i*2];
        velocities(i,:) = [speedFactor, 0];
    end

    %% === FIGURE SETUP ===
    figure('Color','k'); hold on; grid on; axis equal;
    set(gca,'Color','k','XColor','w','YColor','w');
    title('UAV V-Formation | Sine Path | Avoiding Obstacles & Incoming Drones','Color','w');
    xlabel('X','Color','w'); ylabel('Y','Color','w');
    xlim([0, xEnd]); ylim([-220, 220]);

    % Reference path
    refX = linspace(0, xEnd, 1500);
    refY = yCenter + waveAmplitude * sin(waveFrequency * refX);
    plot(refX, refY, 'w--', 'LineWidth', 1);

    % Start / End labels
    text(0, 0, 'A: Start','Color','g','FontSize',10,'FontWeight','bold');
    text(xEnd, 0, 'B: End','Color','y','FontSize',10,'FontWeight','bold');

    %% === ANIMATION LOOP ===
    for t = 0:dt:totalTime
        % --- LEADER PATH FOLLOWING ---
        xLead = positions(1,1) + speedFactor;
        yLead = yCenter + waveAmplitude * sin(waveFrequency * xLead);
        desiredPos = [xLead, yLead];
        toTarget = desiredPos - positions(1,:);
        toTargetDir = toTarget / max(norm(toTarget), 1e-6);

        % --- LEADER OBSTACLE & ENEMY AVOIDANCE ---
        avoidanceVec = [0,0];
        allThreats = [obstacles(:,1:2), obstacles(:,3)+7]; % obstacle list with safe radius
        for o = 1:size(allThreats,1)
            vecTo = positions(1,:) - allThreats(o,1:2);
            dist = norm(vecTo);
            safeDist = allThreats(o,3);
            if dist < safeDist
                avoidanceVec = avoidanceVec + (vecTo / dist) * (safeDist - dist);
            end
        end
        % include enemies
        for e = 1:numEnemies
            vecToEnemy = positions(1,:) - enemyPositions(e,:);
            distE = norm(vecToEnemy);
            if distE < 20
                avoidanceVec = avoidanceVec + (vecToEnemy / distE) * (20 - distE);
            end
        end

        % Combine
        desiredDir = toTargetDir + avoidanceVec * avoidanceStrength;
        desiredDir = desiredDir / max(norm(desiredDir), 1e-6);

        velocities(1,:) = velocities(1,:) * 0.8 + desiredDir * speedFactor * 0.2;
        positions(1,:) = positions(1,:) + velocities(1,:) * dt;

        % --- FORMATION ---
        headingAngle = atan2(velocities(1,2), velocities(1,1));
        offsets = generateRotatedVOffsets(numUAVs, baseSpacing, formationAngle, headingAngle);

        for i = 2:numUAVs
            desiredPos = positions(1,:) + offsets(i,:);
            toFormation = desiredPos - positions(i,:);
            positions(i,:) = positions(i,:) + toFormation * formationPullStrength;

            % Avoid obstacles
            for o = 1:size(obstacles,1)
                vecToObs = positions(i,:) - obstacles(o,1:2);
                distToObs = norm(vecToObs);
                safeDist = obstacles(o,3) + 7;
                if distToObs < safeDist
                    avoidanceVec = (vecToObs / distToObs) * (safeDist - distToObs);
                    positions(i,:) = positions(i,:) + avoidanceVec * avoidanceStrength;
                end
            end

            % Avoid enemy drones
            for e = 1:numEnemies
                vecToEnemy = positions(i,:) - enemyPositions(e,:);
                distE = norm(vecToEnemy);
                if distE < 20
                    avoidanceVec = (vecToEnemy / distE) * (20 - distE);
                    positions(i,:) = positions(i,:) + avoidanceVec * avoidanceStrength;
                end
            end

            % Repulsion from other UAVs
            for j = 1:numUAVs
                if j ~= i
                    vecToUAV = positions(i,:) - positions(j,:);
                    distToUAV = norm(vecToUAV);
                    if distToUAV < baseSpacing
                        positions(i,:) = positions(i,:) + ...
                            (vecToUAV / distToUAV) * repulsionStrength;
                    end
                end
            end
        end

        % --- ENEMY MOVEMENT ---
        enemyPositions = enemyPositions + enemyVelocities * dt;
        % respawn if off-screen
        for e = 1:numEnemies
            if enemyPositions(e,2) < -250
                enemyPositions(e,:) = [rand*xEnd, 220];
                enemyVelocities(e,:) = [randn*3, -enemySpeed];
            elseif enemyPositions(e,2) > 250
                enemyPositions(e,:) = [rand*xEnd, -220];
                enemyVelocities(e,:) = [randn*3, enemySpeed];
            end
        end

        % === DRAWING ===
        cla;
        plot(refX, refY, 'w--', 'LineWidth', 1);
        for o = 1:size(obstacles,1)
            viscircles(obstacles(o,1:2), obstacles(o,3),'Color','r','LineWidth',1);
        end
        plot(positions(:,1), positions(:,2), 'wo', 'MarkerFaceColor','c', 'MarkerSize',8);
        plot(positions(1,1), positions(1,2), 'wo', 'MarkerFaceColor','y', 'MarkerSize',10);
        plot(enemyPositions(:,1), enemyPositions(:,2), 'ro', 'MarkerFaceColor','m', 'MarkerSize',6);
        text(0, 0, 'A: Start','Color','g','FontSize',10,'FontWeight','bold');
        text(xEnd, 0, 'B: End','Color','y','FontSize',10,'FontWeight','bold');
        xlim([0, xEnd]); ylim([-220, 220]);
        drawnow;
    end
end

%% === FORMATION OFFSET GENERATOR ===
%%
function offsets = generateRotatedVOffsets(numUAVs, spacing, angle, headingAngle)
    offsets = zeros(numUAVs, 2);
    half = floor((numUAVs - 1) / 2);
    idx = 2;
    for k = 1:half
        leftPos = [-spacing*k*cos(angle),  spacing*k*sin(angle)];
        rightPos = [-spacing*k*cos(angle), -spacing*k*sin(angle)];
        offsets(idx,:) = rotatePoint(leftPos, headingAngle);
        idx = idx + 1;
        if idx <= numUAVs
            offsets(idx,:) = rotatePoint(rightPos, headingAngle);
            idx = idx + 1;
        end
    end
end

%% === POINT ROTATION ===
function rotated = rotatePoint(pt, theta)
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated = (R * pt')';
end