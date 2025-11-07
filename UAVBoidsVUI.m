function UAVBoidsVUI()
    %% === MAIN FIGURE ===
    fig = uifigure('Name','UAV Flocking Simulation','Color',[0 0 0],'Position',[100 100 900 600]);

    % --- Controls Panel ---
    uilabel(fig,'Text','UAV Flocking Controls','FontWeight','bold','FontSize',20,...
        'Position',[50 550 250 30],'FontColor','w','BackgroundColor','k');

    uilabel(fig,'Text','Number of UAVs:','Position',[50 500 150 30],'FontColor','w','BackgroundColor','k');
    numSlider = uislider(fig,'Position',[180 515 150 3],'Limits',[3 10],'Value',5);
    numValue = uilabel(fig,'Text','5','Position',[340 500 40 30],'FontColor','w','BackgroundColor','k');

    uilabel(fig,'Text','Speed:','Position',[50 450 100 30],'FontColor','w','BackgroundColor','k');
    speedSlider = uislider(fig,'Position',[180 465 150 3],'Limits',[0.5 100],'Value',1);
    speedValue = uilabel(fig,'Text','1.0x','Position',[340 450 60 30],'FontColor','w','BackgroundColor','k');

    uilabel(fig,'Text','Formation Type:','Position',[50 400 150 30],'FontColor','w','BackgroundColor','k');
    formationDrop = uidropdown(fig,'Items',{'V','Line'},'Value','V','Position',[180 405 100 30]);

    startButton = uibutton(fig,'Text','Start Simulation','FontSize',13,...
        'BackgroundColor',[0.2 0.7 0.2],'FontWeight','bold','Position',[50 340 230 40],...
        'ButtonPushedFcn',@(btn,evt) startSimulation());

    % --- Display Axes ---
    ax = uiaxes(fig,'Position',[450 80 400 450],'Color','k');
    ax.XColor = 'w'; ax.YColor = 'w';
    title(ax,'2D UAV Formation Path','Color','w');
    xlabel(ax,'Longitude','Color','w'); ylabel(ax,'Latitude','Color','w');
    hold(ax,'on');

    % --- Dynamic Labels ---
    numSlider.ValueChangingFcn = @(src,evt) set(numValue,'Text',sprintf('%d',round(evt.Value)));
    speedSlider.ValueChangingFcn = @(src,evt) set(speedValue,'Text',sprintf('%.1fx',evt.Value));

    %% === MAIN CALLBACK ===
    function startSimulation()
        numUAVs = round(numSlider.Value);
        speedScale = speedSlider.Value;
        formationType = formationDrop.Value;

        cla(ax);

        % Simulation Parameters
        t = linspace(0, 100, 200);
        zLead = 20 * ones(size(t));
        xLead = t * 10;
        yLead = 30 * sin(0.3 * t);

        % Fixed ranges for nice viewing
        xRange2D = [min(xLead)-40, max(xLead)+50];
        yRange2D = [min(yLead)-40, max(yLead)+40];

        % Wind optional
        windEnabled = true;
        windAmplitude = 0.5;
        windFrequency = 0.1;

        % Formation
        offsets = generateFormationOffsets(numUAVs, formationType);

        % Initialize scattered starting positions
        uavPositions = [xLead(1) + 40*rand(numUAVs,1)-20, ...
                        yLead(1) + 40*rand(numUAVs,1)-20, ...
                        20*ones(numUAVs,1)];

        trailX = zeros(numUAVs, length(t));
        trailY = zeros(numUAVs, length(t));

        % Animation Loop
        for i = 1:length(t)
            leadPos = [xLead(i), yLead(i), zLead(i)];
            if i < length(t)
                dir = [xLead(i+1)-xLead(i), yLead(i+1)-yLead(i), zLead(i+1)-zLead(i)];
            else
                dir = [1, 0, 0];
            end
            dir = dir / norm(dir);

            % Transition UAVs smoothly to V or Line formation
            for j = 1:numUAVs
                targetPos = leadPos + offsets(j,:) * getFormationMatrix(dir);

                if windEnabled
                    windShift = windAmplitude * sin(2 * pi * windFrequency * t(i) + j);
                    targetPos = targetPos + [0, windShift, 0];
                end

                % Smooth move from scattered start to formation
                alpha = min(1, i / 40);
                uavPositions(j,:) = (1-alpha)*uavPositions(j,:) + alpha*targetPos;

                trailX(j,i) = uavPositions(j,1);
                trailY(j,i) = uavPositions(j,2);
            end

            % Draw on UI Axes
            cla(ax);
            for j = 1:numUAVs
                plot(ax, trailX(j,1:i), trailY(j,1:i), '--', 'Color', [0.3 0.7 1]);
                plot(ax, trailX(j,i), trailY(j,i), 'wo', 'MarkerFaceColor','w');
            end
            xlim(ax,xRange2D); ylim(ax,yRange2D);
            title(ax,sprintf('t = %.1fs', t(i)), 'Color','w');
            drawnow;
            pause(0.01 / speedScale);
        end
    end
end

%% === Helper Functions ===
%%
function R = getFormationMatrix(direction)
    z = [0 0 1];
    y = cross(z, direction); y = y / norm(y);
    x = cross(y, z);
    R = [x; y; z];
end

function offsets = generateFormationOffsets(N, type)
    offsets = zeros(N, 3);
    switch type
        case 'V'
            offsets(1,:) = [0, 0, 0];
            count = 1;
            for i = 1:floor((N-1)/2)
                count = count + 1;
                offsets(count,:) = [-i * 2.5, -i * 3, 0];
                if count < N
                    count = count + 1;
                    offsets(count,:) = [-i * 2.5, i * 3, 0];
                end
            end
        case 'Line'
            for i = 1:N
                offsets(i,:) = [-i * 3, 0, 0];
            end
    end
end