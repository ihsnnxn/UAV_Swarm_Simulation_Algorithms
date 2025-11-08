function HybridFloydRRT()
% HYBRID_FLOYD_RRT
% GUI combining Floyd–Warshall (global) + RRT (local) planners.
% Multiple drones each compute a hybrid path (global node sequence + per-leg RRT)
% and animate simultaneously.
%
% Usage: runin MATLAB.

%% ---------- Planner parameters (tweakable) ----------
% RRT parameters (used per leg)
RRT_maxIter = 800;
RRT_stepSize = 0.6;
RRT_goalBias = 0.05;
RRT_goalTol = 0.4;

% Workspace bounds [xmin xmax ymin ymax]
W = [0 10 0 10];

% Animation params
baseStepsPerSegment = 40;  % RRT segment interpolation points
framePause = 0.02;

%% ---------- Setup GUI ----------
fig = figure('Name','Hybrid Floyd–RRT Drone Simulator V1', ...
    'NumberTitle','off','Position',[80 80 1500 800], ...
    'CloseRequestFcn',@closeFigureSafely);

% Axes (center-left)
ax = axes('Parent',fig,'Position',[0.12 0.12 0.60 0.78]);
axis(ax,[W(1) W(2) W(3) W(4)]);
grid(ax,'on'); hold(ax,'on');
title(ax,'Workspace (Hybrid Planner)','FontSize',13);

% Left: Path & Drone info
uicontrol(fig,'Style','text','String','Drone Paths & Costs','Units','normalized', ...
    'Position',[0.01 0.92 0.1 0.05],'FontWeight','bold','HorizontalAlignment','left');
pathInfoList = uicontrol(fig,'Style','listbox','String',{'No drones yet'},...
    'Units','normalized','Position',[0.01 0.12 0.1 0.8],'FontSize',10,'HorizontalAlignment','left');

% Right: Instructions
uicontrol(fig,'Style','text','String','Instructions','Units','normalized', ...
    'Position',[0.75 0.92 0.24 0.05],'FontWeight','bold','HorizontalAlignment','left');
instr = ['1) Click "Add Nodes" then click points on the graph; press Enter to finish.' newline ...
         '2) Use "Move Node" / "Delete Node" to edit nodes.' newline ...
         '3) Add circle/rect obstacles (they block edges & RRT).' newline ...
         '4) Create Drone(s) (name, start, end, color, marker, speed).' newline ...
         '5) Click "Plan Hybrid" to compute Floyd sequence + RRT per-leg for each drone.' newline ...
         '6) Click "Start All Drones" to animate all drones simultaneously.'];
uicontrol(fig,'Style','text','String',instr,'Units','normalized','Position',[0.75 0.12 0.24 0.8],'HorizontalAlignment','left');

% Controls header
uicontrol(fig,'Style','text','String','Controls','Units','normalized',...
    'Position',[0.74 0.82 0.20 0.05],'FontWeight','bold','HorizontalAlignment','left');

% Buttons (arranged right side)
btnY = 0.76;
btnH = 0.055; btnW = 0.22;
uicontrol(fig,'Style','pushbutton','String','Add Nodes (Click)',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@addNodesCallback);
btnY = btnY - 0.06;
uicontrol(fig,'Style','pushbutton','String','Move Node',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@moveNodeCallback);
btnY = btnY - 0.06;
uicontrol(fig,'Style','pushbutton','String','Delete Node',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@deleteNodeCallback);
btnY = btnY - 0.07;
uicontrol(fig,'Style','pushbutton','String','Add Circle Obstacle',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@addCircleObstacleCallback);
btnY = btnY - 0.06;
uicontrol(fig,'Style','pushbutton','String','Add Rect Obstacle',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@addRectObstacleCallback);
btnY = btnY - 0.06;
uicontrol(fig,'Style','pushbutton','String','Delete Obstacle',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@deleteObstacleCallback);
btnY = btnY - 0.07;
uicontrol(fig,'Style','pushbutton','String','Create Drone',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@createDroneDialogCallback);
btnY = btnY - 0.06;
uicontrol(fig,'Style','pushbutton','String','Delete Selected Drone',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@deleteSelectedDroneCallback);
btnY = btnY - 0.07;
uicontrol(fig,'Style','pushbutton','String','Plan Hybrid',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@planHybridCallback,'BackgroundColor',[0.2 0.6 1],'FontWeight','bold');
btnY = btnY - 0.06;
uicontrol(fig,'Style','pushbutton','String','Start All Drones',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@startAllDronesCallback,'BackgroundColor',[0.2 0.8 0.2],'FontWeight','bold');
btnY = btnY - 0.07;
uicontrol(fig,'Style','pushbutton','String','Reset Defaults',...
    'Units','normalized','Position',[0.74 btnY btnW btnH],'Callback',@resetCallback);

% Node dropdown defaults (for convenience)
uicontrol(fig,'Style','text','String','Start Node (default):','Units','normalized','Position',[0.74 0.34 0.12 0.035],'HorizontalAlignment','left');
startNodeDefault = uicontrol(fig,'Style','popupmenu','String',{'1'},'Units','normalized','Position',[0.86 0.34 0.075 0.035]);

uicontrol(fig,'Style','text','String','End Node (default):','Units','normalized','Position',[0.74 0.30 0.12 0.035],'HorizontalAlignment','left');
endNodeDefault = uicontrol(fig,'Style','popupmenu','String',{'2'},'Units','normalized','Position',[0.86 0.30 0.075 0.035]);

% Drone list UI
uicontrol(fig,'Style','text','String','Drones','Units','normalized','Position',[0.74 0.20 0.22 0.035],'FontWeight','bold','HorizontalAlignment','left');
droneList = uicontrol(fig,'Style','listbox','String',{},'Units','normalized','Position',[0.74 0.05 0.22 0.15],'FontSize',9);

% Table placeholders
uitable(fig,'Data',[],'ColumnEditable',false,'Units','normalized','Position',[0.92 0.55 0.06 0.33],'Tag','DTable','ColumnWidth',{40});

%% ---------- Data storage ----------
% G struct holds world state
G.nodes = [];        % Nx2 coordinates
G.obstacles = [];    % struct array with fields (type,cx,cy,r / xmin,xmax,ymin,ymax, h)
G.origEdges = [];    % NxN Euclidean distance matrix (before blocking)
G.edges = [];        % NxN effective weights (inf if blocked)
G.drones = [];       % drones array: name,start,end,color,marker,speed,hybridPath,cost,h graphics

% internal: last computed hybrid paths per drone are stored in G.drones(k).hybridPath

% initialize default network
resetNetwork();

%% ---------- Nested GUI callback functions ----------

    function addNodesCallback(~,~)
        if ~ishandle(ax), return; end
        try
            [x,y] = ginput;
        catch
            disp('Add nodes cancelled.');
            return;
        end
        if isempty(x), return; end
        for k=1:numel(x)
            G.nodes(end+1,:) = [x(k) y(k)];
        end
        rebuildGraphEdges();
        redrawGraph();
        updateMenusAndDefaults();
    end

    function moveNodeCallback(~,~)
        if isempty(G.nodes), return; end
        disp('Click a node to move, then click new location.');
        try
            [x,y] = ginput(1);
        catch
            return;
        end
        if isempty(x), return; end
        [~,idx] = min(vecnorm(G.nodes - [x y],2,2));
        try
            [nx,ny] = ginput(1);
        catch
            return;
        end
        if isempty(nx), return; end
        G.nodes(idx,:) = [nx ny];
        rebuildGraphEdges();
        redrawGraph();
    end

    function deleteNodeCallback(~,~)
        if isempty(G.nodes), return; end
        disp('Click a node to delete.');
        try
            [x,y] = ginput(1);
        catch
            return;
        end
        if isempty(x), return; end
        [~,idx] = min(vecnorm(G.nodes - [x y],2,2));
        G.nodes(idx,:) = [];
        % remove edges row/col
        if ~isempty(G.origEdges)
            G.origEdges(idx,:) = [];
            G.origEdges(:,idx) = [];
        end
        rebuildGraphEdges();
        % remove drones that referenced deleted node, adjust indices > idx
        keep = true(1,length(G.drones));
        for d=1:length(G.drones)
            s = G.drones(d).start; e = G.drones(d).end;
            if s==idx || e==idx
                keep(d) = false;
            else
                if s>idx, G.drones(d).start = s-1; end
                if e>idx, G.drones(d).end = e-1; end
            end
        end
        G.drones = G.drones(keep);
        redrawGraph();
        updateMenusAndDefaults();
        updateDroneList();
        updatePathInfo();
    end

    function addCircleObstacleCallback(~,~)
        msgbox('Click center then a point on circumference.');
        try
            [cx,cy] = ginput(1);
        catch, return; end
        if isempty(cx), return; end
        try
            [px,py] = ginput(1);
        catch, return; end
        if isempty(px), return; end
        r = hypot(px-cx,py-cy);
        obs.type = 'circle'; obs.cx = cx; obs.cy = cy; obs.r = r;
        obs.h = rectangle('Position',[cx-r,cy-r,2*r,2*r],'Curvature',[1 1],'EdgeColor','r','LineStyle','--','Parent',ax);
        G.obstacles = [G.obstacles; obs];
        % recompute edges (blocking)
        rebuildGraphEdges();
        redrawGraph();
    end

    function addRectObstacleCallback(~,~)
        msgbox('Click first corner then opposite corner.');
        try
            [x1,y1] = ginput(1);
        catch, return; end
        if isempty(x1), return; end
        try
            [x2,y2] = ginput(1);
        catch, return; end
        if isempty(x2), return; end
        xmin = min(x1,x2); xmax = max(x1,x2); ymin = min(y1,y2); ymax = max(y1,y2);
        obs.type = 'rect'; obs.xmin = xmin; obs.xmax = xmax; obs.ymin = ymin; obs.ymax = ymax;
        obs.h = rectangle('Position',[xmin,ymin,xmax-xmin,ymax-ymin],'EdgeColor','r','LineStyle','--','Parent',ax);
        G.obstacles = [G.obstacles; obs];
        rebuildGraphEdges();
        redrawGraph();
    end

    function deleteObstacleCallback(~,~)
        if isempty(G.obstacles)
            msgbox('No obstacles to delete.');
            return;
        end
        msgbox('Click near an obstacle center to delete.');
        try
            [x,y] = ginput(1);
        catch, return; end
        if isempty(x), return; end
        dmin = inf; idxDel = -1;
        for k=1:length(G.obstacles)
            obs = G.obstacles(k);
            if strcmp(obs.type,'circle')
                d = abs(hypot(x-obs.cx,y-obs.cy) - obs.r);
            else
                cx = (obs.xmin + obs.xmax)/2; cy = (obs.ymin + obs.ymax)/2;
                d = hypot(x-cx,y-cy);
            end
            if d < dmin
                dmin = d; idxDel = k;
            end
        end
        if idxDel>0
            try delete(G.obstacles(idxDel).h); catch, end
            G.obstacles(idxDel) = [];
            rebuildGraphEdges();
            redrawGraph();
        end
    end

    function createDroneDialogCallback(~,~)
        if isempty(G.nodes)
            warndlg('Please add nodes first.'); return;
        end
        % modal dialog
        dlg = dialog('Name','Create Drone','Units','normalized','Position',[0.35 0.35 0.3 0.36],'WindowStyle','modal');
        uicontrol(dlg,'Style','text','String','Name:','Units','normalized','Position',[0.05 0.86 0.25 0.10],'HorizontalAlignment','left');
        nameEdit = uicontrol(dlg,'Style','edit','String',sprintf('Drone%d',length(G.drones)+1),'Units','normalized','Position',[0.33 0.86 0.62 0.10]);
        % start/end dropdowns
        n = size(G.nodes,1);
        labels = arrayfun(@(i) sprintf('Node %d',i),1:n,'UniformOutput',false);
        uicontrol(dlg,'Style','text','String','Start Node:','Units','normalized','Position',[0.05 0.72 0.25 0.08],'HorizontalAlignment','left');
        startPopup = uicontrol(dlg,'Style','popupmenu','String',labels,'Units','normalized','Position',[0.33 0.72 0.28 0.08],'Value',max(1,min(get(startNodeDefault,'Value'),n)));
        uicontrol(dlg,'Style','text','String','End Node:','Units','normalized','Position',[0.05 0.60 0.25 0.08],'HorizontalAlignment','left');
        endPopup = uicontrol(dlg,'Style','popupmenu','String',labels,'Units','normalized','Position',[0.33 0.60 0.28 0.08],'Value',max(1,min(get(endNodeDefault,'Value'),n)));
        % color, marker, speed
        uicontrol(dlg,'Style','text','String','Color (r/g/b/#RRGGBB):','Units','normalized','Position',[0.05 0.48 0.5 0.08],'HorizontalAlignment','left');
        colorEdit = uicontrol(dlg,'Style','edit','String','r','Units','normalized','Position',[0.55 0.48 0.4 0.08]);
        uicontrol(dlg,'Style','text','String','Marker (o,s,^,d):','Units','normalized','Position',[0.05 0.36 0.5 0.08],'HorizontalAlignment','left');
        markerPopup = uicontrol(dlg,'Style','popupmenu','String',{'o','s','^','d','p','h'},'Units','normalized','Position',[0.55 0.36 0.4 0.08]);
        uicontrol(dlg,'Style','text','String','Speed (1=base):','Units','normalized','Position',[0.05 0.24 0.5 0.08],'HorizontalAlignment','left');
        speedEdit = uicontrol(dlg,'Style','edit','String','1','Units','normalized','Position',[0.55 0.24 0.4 0.08]);
        % create/cancel
        uicontrol(dlg,'Style','pushbutton','String','Create','Units','normalized','Position',[0.18 0.08 0.26 0.10],...
            'Callback',@createDroneAndClose);
        uicontrol(dlg,'Style','pushbutton','String','Cancel','Units','normalized','Position',[0.56 0.08 0.26 0.10],...
            'Callback',@(src,ev) close(dlg));
        function createDroneAndClose(~,~)
            name = strtrim(get(nameEdit,'String'));
            s = get(startPopup,'Value'); e = get(endPopup,'Value');
            colorStr = strtrim(get(colorEdit,'String'));
            mlist = get(markerPopup,'String'); mval = get(markerPopup,'Value'); markerSym = mlist{mval};
            sp = str2double(get(speedEdit,'String'));
            if isempty(name), name = sprintf('Drone%d',length(G.drones)+1); end
            if isnan(sp) || sp<=0, sp = 1; end
            drone.name = name; drone.start = s; drone.end = e; drone.color = colorStr;
            drone.marker = markerSym; drone.speed = sp;
            drone.hybridPath = []; drone.cost = inf;
            drone.h.marker = []; drone.h.trail = [];
            G.drones = [G.drones; drone];
            close(dlg);
            updateDroneList();
            updatePathInfo();
        end
    end

    function deleteSelectedDroneCallback(~,~)
        sel = get(droneList,'Value');
        if isempty(G.drones) || sel<1 || sel>length(G.drones)
            warndlg('Select a drone to delete.'); return;
        end
        % remove graphics
        if isfield(G.drones(sel),'h')
            try delete(G.drones(sel).h.marker); catch, end
            try delete(G.drones(sel).h.trail); catch, end
        end
        G.drones(sel) = [];
        updateDroneList();
        updatePathInfo();
    end

    function planHybridCallback(~,~)
        if isempty(G.nodes)
            warndlg('Add nodes first.'); return;
        end
        if isempty(G.drones)
            warndlg('Create at least one drone.'); return;
        end

        % Rebuild graph edges from node geometry (Euclidean), then mask blocked edges
        rebuildGraphEdges();

        % run Floyd-Warshall on G.edges (weights: euclidean distances or inf if blocked)
        [D,P] = floydWarshall(G.edges);
        updateTables(D,P);

        % For each drone: find node sequence using Floyd; then for each consecutive pair, run RRT to get continuous subpath
        for d = 1:length(G.drones)
            s = G.drones(d).start; e = G.drones(d).end;
            % reconstruct node-to-node route
            seq = reconstructPath(P,s,e);
            if isempty(seq)
                % no global connectivity
                G.drones(d).hybridPath = [];
                G.drones(d).cost = inf;
                continue;
            end
            % iterate consecutive node pairs and run RRT on each leg
            fullPath = [];
            totalLen = 0;
            failed = false;
            for k = 2:length(seq)
                n1 = seq(k-1); n2 = seq(k);
                p1 = G.nodes(n1,:); p2 = G.nodes(n2,:);
                % run RRT for this leg (start p1, goal p2)
                [legPath, ~] = rrt_planner(p1, p2, G.obstacles, W, RRT_maxIter, RRT_stepSize, RRT_goalBias, RRT_goalTol);
                if isempty(legPath)
                    % try with larger budget once
                    [legPath, ~] = rrt_planner(p1, p2, G.obstacles, W, round(RRT_maxIter*2), RRT_stepSize, RRT_goalBias, RRT_goalTol);
                end
                if isempty(legPath)
                    failed = true;
                    break;
                end
                % stitch: avoid duplication of first point if fullPath not empty
                if isempty(fullPath)
                    fullPath = legPath;
                else
                    fullPath = [fullPath; legPath(2:end,:)]; %#ok<AGROW>
                end
                totalLen = totalLen + path_length(legPath);
            end
            if failed
                G.drones(d).hybridPath = [];
                G.drones(d).cost = inf;
            else
                G.drones(d).hybridPath = fullPath;
                G.drones(d).cost = totalLen;
            end
        end

        % update UI: path info and draw hybrid paths overlay
        updatePathInfo();
        redrawGraph();
        % draw hybrid paths for drones (distinct colors)
        for d = 1:length(G.drones)
            hp = G.drones(d).hybridPath;
            if ~isempty(hp)
                try
                    plot(ax, hp(:,1), hp(:,2), '-','LineWidth',2,'Color',G.drones(d).color);
                catch
                    plot(ax, hp(:,1), hp(:,2), '-','LineWidth',2); % fallback
                end
            end
        end
        drawnow;
    end

    function startAllDronesCallback(~,~)
        if isempty(G.drones)
            warndlg('No drones to animate.'); return;
        end
        % ensure hybrid paths exist (prompt to plan if not)
        needPlan = false;
        for d=1:length(G.drones)
            if isempty(G.drones(d).hybridPath)
                needPlan = true; break;
            end
        end
        if needPlan
            uiwait(msgbox('Some drones have no hybrid path. Please click "Plan Hybrid" first (or re-run).','Plan needed','modal'));
            return;
        end

        % Prepare trajectories for all drones: downsample per speed
        numD = length(G.drones);
        trajectories = cell(1,numD);
        lenPts = zeros(1,numD);
        for d=1:numD
            pts = G.drones(d).hybridPath;
            % optionally interpolate for smoother motion: already continuous from RRT
            trajectories{d} = pts;
            lenPts(d) = size(pts,1);
        end
        maxLen = max(lenPts);
        if isempty(maxLen) || maxLen==0
            warndlg('No path points to animate.'); return;
        end

        % Clear previous drone markers & trails
        for d=1:numD
            if isfield(G.drones(d),'h') && ~isempty(G.drones(d).h)
                try delete(G.drones(d).h.marker); catch, end
                try delete(G.drones(d).h.trail); catch, end
            end
            G.drones(d).h.marker = [];
            G.drones(d).h.trail = [];
        end

        % Create marker/trail for each drone
        for d=1:numD
            pts = trajectories{d};
            if isempty(pts), continue; end
            pos = pts(1,:);
            try
                htrail = plot(ax,pos(1),pos(2),'-','LineWidth',1.5,'Color',G.drones(d).color,'Parent',ax);
                hmarker = plot(ax,pos(1),pos(2),G.drones(d).marker,'MarkerFaceColor',G.drones(d).color,'MarkerEdgeColor','k','MarkerSize',9,'Parent',ax);
            catch
                % fallback color if string invalid
                col = rand(1,3);
                htrail = plot(ax,pos(1),pos(2),'-','LineWidth',1.5,'Color',col,'Parent',ax);
                hmarker = plot(ax,pos(1),pos(2),G.drones(d).marker,'MarkerFaceColor',col,'MarkerEdgeColor','k','MarkerSize',9,'Parent',ax);
                G.drones(d).color = col;
            end
            G.drones(d).h.trail = htrail;
            G.drones(d).h.marker = hmarker;
        end

        % Compute per-drone point-per-frame speed multiplier
        speedMult = zeros(1,numD);
        for d=1:numD
            sp = G.drones(d).speed;
            speedMult(d) = max(1, round(sp));
        end

        % Frame loop: advance each drone by speedMult(d) points per frame
        % Stop when all drones reach end
        idxs = ones(1,numD); % current index in trajectory
        finished = false(1,numD);
        anyActive = true;
        while any(~finished)
            anyActive = false;
            for d=1:numD
                pts = trajectories{d};
                if isempty(pts) || finished(d), continue; end
                anyActive = true;
                idxs(d) = min(size(pts,1), idxs(d) + speedMult(d));
                pos = pts(idxs(d),:);
                if isfield(G.drones(d).h,'marker') && ishandle(G.drones(d).h.marker)
                    set(G.drones(d).h.marker,'XData',pos(1),'YData',pos(2));
                    % extend trail
                    xt = get(G.drones(d).h.trail,'XData'); yt = get(G.drones(d).h.trail,'YData');
                    set(G.drones(d).h.trail,'XData',[xt pos(1)],'YData',[yt pos(2)]);
                end
                if idxs(d) >= size(pts,1)
                    finished(d) = true;
                end
            end
            if ~anyActive, break; end
            pause(framePause);
            drawnow;
        end
    end

    function resetCallback(~,~)
        % clear obstacle graphics
        for k=1:length(G.obstacles)
            try delete(G.obstacles(k).h); catch, end
        end
        G.obstacles = [];
        % clear drone graphics
        for d=1:length(G.drones)
            if isfield(G.drones(d),'h')
                try delete(G.drones(d).h.marker); catch, end
                try delete(G.drones(d).h.trail); catch, end
            end
        end
        G.drones = [];
        resetNetwork();
        redrawGraph();
        updateMenusAndDefaults();
        updateDroneList();
        updatePathInfo();
    end

%% ---------- Helper UI updates ----------

    function updateMenusAndDefaults()
        n = size(G.nodes,1);
        if n==0
            startNodeDefault.String = {'1'}; startNodeDefault.Value = 1;
            endNodeDefault.String = {'1'}; endNodeDefault.Value = 1;
        else
            labels = arrayfun(@(i) sprintf('Node %d',i),1:n,'UniformOutput',false);
            startNodeDefault.String = labels; endNodeDefault.String = labels;
            startNodeDefault.Value = min(startNodeDefault.Value,n);
            endNodeDefault.Value = min(endNodeDefault.Value,n);
        end
    end

    function updateDroneList()
        if isempty(G.drones)
            set(droneList,'String',{},'Value',1);
            return;
        end
        labels = arrayfun(@(i) sprintf('%d: %s (S%d->E%d)', i, G.drones(i).name, G.drones(i).start, G.drones(i).end), 1:length(G.drones), 'UniformOutput', false);
        set(droneList,'String',labels,'Value',min(get(droneList,'Value'),length(labels)));
    end

    function updatePathInfo()
        if isempty(G.drones)
            set(pathInfoList,'String',{'No drones yet'});
            return;
        end
        lines = {};
        for i=1:length(G.drones)
            d = G.drones(i);
            if isempty(d.hybridPath)
                pstr = 'No hybrid path';
                coststr = 'Inf';
            else
                pstr = sprintf('%d pts', size(d.hybridPath,1));
                coststr = sprintf('%.3f', d.cost);
            end
            lines{end+1} = sprintf('Drone %d: %s (S%d->E%d)', i, d.name, d.start, d.end); %#ok<AGROW>
            lines{end+1} = sprintf('  Path: %s   Cost: %s   Color:%s   Marker:%s   Speed:%.2f', pstr, coststr, mat2str_color(d.color), d.marker, d.speed); %#ok<AGROW>
        end
        set(pathInfoList,'String',lines,'Value',1);
    end

    function s = mat2str_color(col)
        % pretty representation for color (string or rgb)
        if ischar(col)
            s = col;
        elseif isnumeric(col) && numel(col)==3
            s = sprintf('[%.2f %.2f %.2f]', col(1), col(2), col(3));
        else
            s = 'color';
        end
    end

    function updateTables(D,P)
        dtab = findobj(fig,'Tag','DTable');
        if isempty(D)
            dtab.Data = {};
        else
            dtab.Data = num2cell(round(D,2));
        end
    end

%% ---------- Core planners: Floyd–Warshall & RRT ----------

    function [D,P] = floydWarshall(W)
        n = size(W,1);
        if n==0, D=[]; P=[]; return; end
        D = W; P = zeros(n);
        for i=1:n
            for j=1:n
                if i~=j && isfinite(W(i,j))
                    P(i,j) = i;
                else
                    P(i,j) = 0;
                end
            end
        end
        for k=1:n
            for i=1:n
                for j=1:n
                    if D(i,k) + D(k,j) < D(i,j)
                        D(i,j) = D(i,k) + D(k,j);
                        P(i,j) = P(k,j);
                    end
                end
            end
        end
    end

    function path = reconstructPath(P,i,j)
        if isempty(P)
            path = []; return;
        end
        if P(i,j) == 0
            if i==j, path = i; else path = []; end
        else
            path = [reconstructPath(P,i,P(i,j)) j];
        end
    end

    function [path, Tree] = rrt_planner(startPt, goalPt, obstacles, bounds, maxIter, stepSize, goalBias, goalTol)
        % Simple RRT between startPt and goalPt within bounds avoiding obstacles.
        xmin = bounds(1); xmax = bounds(2); ymin = bounds(3); ymax = bounds(4);
        Node(1).pt = startPt; Node(1).parent = 0;
        Tree = Node;
        found = false;
        for iter = 1:maxIter
            % sample
            if rand < goalBias
                sample = goalPt;
            else
                sample = [ xmin + rand*(xmax-xmin), ymin + rand*(ymax-ymin) ];
            end
            % nearest
            pts = cell2mat(arrayfun(@(n) n.pt, Tree,'UniformOutput',false)');
            dists = vecnorm(pts - sample,2,2);
            [~, idxNearest] = min(dists);
            nearest = Tree(idxNearest).pt;
            dir = sample - nearest;
            nd = norm(dir);
            if nd == 0, continue; end
            step = min(stepSize, nd);
            newPt = nearest + (dir/nd)*step;
            % collision check segment nearest->newPt
            if ~segmentCollision(nearest,newPt,obstacles)
                newNode.pt = newPt; newNode.parent = idxNearest;
                Tree(end+1) = newNode; %#ok<AGROW>
                if norm(newPt - goalPt) <= goalTol
                    if ~segmentCollision(newPt, goalPt, obstacles)
                        goalNode.pt = goalPt; goalNode.parent = length(Tree);
                        Tree(end+1) = goalNode;
                        found = true; break;
                    end
                end
            end
        end
        if ~found
            path = [];
            return;
        end
        % reconstruct path
        path = [];
        idx = length(Tree);
        while idx ~= 0
            path = [Tree(idx).pt; path]; %#ok<AGROW>
            idx = Tree(idx).parent;
        end
    end

    function tf = segmentCollision(p1,p2, obstacles)
        % True if segment hits any obstacle
        tf = false;
        for kk = 1:length(obstacles)
            obs = obstacles(kk);
            if strcmp(obs.type,'circle')
                if segCircleIntersect(p1,p2,[obs.cx obs.cy], obs.r)
                    tf = true; return;
                end
            else
                rect = [obs.xmin obs.ymin obs.xmax obs.ymax];
                if segRectIntersect(p1,p2,rect)
                    tf = true; return;
                end
            end
        end
        % bounds check (optional)
        if any([p1 p2] < -1)  % trivial guard
            tf = true;
        end
    end

%% ---------- Geometry helpers (segment vs circle/rect) ----------

    function tf = segCircleIntersect(p1,p2,c,r)
        v = p2 - p1;
        w = p1 - c;
        a = dot(v,v); b = 2*dot(v,w); ccoef = dot(w,w) - r^2;
        disc = b^2 - 4*a*ccoef;
        if disc < 0, tf = false; return; end
        t1 = (-b - sqrt(disc)) / (2*a);
        t2 = (-b + sqrt(disc)) / (2*a);
        tf = ( (t1>=0 && t1<=1) || (t2>=0 && t2<=1) || (t1<0 && t2>1) );
    end

    function tf = segRectIntersect(p1,p2,rect)
        xmin = rect(1); ymin = rect(2); xmax = rect(3); ymax = rect(4);
        % trivial reject
        if (p1(1) < xmin && p2(1) < xmin) || (p1(1) > xmax && p2(1) > xmax) || ...
           (p1(2) < ymin && p2(2) < ymin) || (p1(2) > ymax && p2(2) > ymax)
            tf = false; return;
        end
        rectEdges = [xmin ymin xmax ymin; xmax ymin xmax ymax; xmax ymax xmin ymax; xmin ymax xmin ymin];
        tf = false;
        for k=1:4
            q1 = rectEdges(k,1:2); q2 = rectEdges(k,3:4);
            if segmentsIntersect(p1,p2,q1,q2)
                tf = true; return;
            end
        end
        if (p1(1) >= xmin && p1(1) <= xmax && p1(2) >= ymin && p1(2) <= ymax) || ...
           (p2(1) >= xmin && p2(1) <= xmax && p2(2) >= ymin && p2(2) <= ymax)
            tf = true; return;
        end
    end

    function tf = segmentsIntersect(p1,p2,q1,q2)
        o1 = orient(p1,p2,q1); o2 = orient(p1,p2,q2);
        o3 = orient(q1,q2,p1); o4 = orient(q1,q2,p2);
        if o1*o2 < 0 && o3*o4 < 0
            tf = true; return;
        end
        tf = (onSegment(p1,p2,q1) || onSegment(p1,p2,q2) || onSegment(q1,q2,p1) || onSegment(q1,q2,p2));
    end

    function v = orient(a,b,c)
        v = (b(1)-a(1))*(c(2)-a(2)) - (b(2)-a(2))*(c(1)-a(1));
    end

    function tf = onSegment(a,b,c)
        tol = 1e-8;
        if min(a(1),b(1))-tol <= c(1) && c(1) <= max(a(1),b(1))+tol && ...
           min(a(2),b(2))-tol <= c(2) && c(2) <= max(a(2),b(2))+tol
            tf = true; else tf = false;
        end
    end

%% ---------- Graph building & utilities ----------

    function rebuildGraphEdges()
        % Build origEdges from Euclidean distances
        n = size(G.nodes,1);
        if n==0
            G.origEdges = [];
            G.edges = [];
            updateTables([] , []);
            return;
        end
        E = inf(n);
        for i=1:n
            E(i,i) = 0;
            for j=i+1:n
                dist = norm(G.nodes(i,:) - G.nodes(j,:));
                E(i,j) = dist; E(j,i) = dist;
            end
        end
        G.origEdges = E;
        % mask edges that are intersecting obstacles -> set to inf in G.edges
        G.edges = G.origEdges;
        for i=1:n
            for j=i+1:n
                if isfinite(G.origEdges(i,j)) && G.origEdges(i,j)>0
                    if segmentCollision(G.nodes(i,:), G.nodes(j,:), G.obstacles)
                        G.edges(i,j) = inf; G.edges(j,i) = inf;
                    end
                end
            end
        end
    end

    function l = path_length(p)
        if isempty(p), l = inf; return; end
        diffs = diff(p);
        l = sum(sqrt(sum(diffs.^2,2)));
    end

%% ---------- Drawing ----------

    function redrawGraph()
        if ~ishandle(ax), return; end
        cla(ax);
        % draw edges: black if available, dashed gray if blocked but originally present
        n = size(G.nodes,1);
        for i=1:n
            for j=i+1:n
                if ~isempty(G.edges) && isfinite(G.edges(i,j)) && G.edges(i,j)>0
                    line([G.nodes(i,1),G.nodes(j,1)],[G.nodes(i,2),G.nodes(j,2)],'Color','k','Parent',ax);
                    mid = mean([G.nodes(i,:);G.nodes(j,:)]);
                    text(mid(1),mid(2),num2str(round(G.origEdges(i,j),2)),'Color','r','Parent',ax,'FontSize',9);
                else
                    if ~isempty(G.origEdges) && size(G.origEdges,1)>=i && isfinite(G.origEdges(i,j)) && G.origEdges(i,j) > 0
                        line([G.nodes(i,1),G.nodes(j,1)],[G.nodes(i,2),G.nodes(j,2)],'Color',[0.7 0.7 0.7],'LineStyle','--','Parent',ax);
                    end
                end
            end
        end
        % draw nodes
        if ~isempty(G.nodes)
            plot(ax,G.nodes(:,1),G.nodes(:,2),'bo','MarkerFaceColor','b','MarkerSize',8);
            for k=1:size(G.nodes,1)
                text(G.nodes(k,1)+0.12,G.nodes(k,2)+0.12,sprintf('%d',k),'Color','m','FontWeight','bold','Parent',ax);
            end
        end
        % draw obstacles
        for k=1:length(G.obstacles)
            obs = G.obstacles(k);
            if isfield(obs,'h') && ishandle(obs.h)
                % keep
            else
                if strcmp(obs.type,'circle')
                    obs.h = rectangle('Position',[obs.cx-obs.r,obs.cy-obs.r,2*obs.r,2*obs.r],'Curvature',[1 1],'EdgeColor','r','LineStyle','--','Parent',ax);
                else
                    obs.h = rectangle('Position',[obs.xmin,obs.ymin,obs.xmax-obs.xmin,obs.ymax-obs.ymin],'EdgeColor','r','LineStyle','--','Parent',ax);
                end
                G.obstacles(k).h = obs.h;
            end
        end
        drawnow;
    end

%% ---------- Initialization / reset ----------

    function resetNetwork()
        % default triangle nodes
        G.nodes = [2 2; 8 2; 5 8];
        rebuildGraphEdges();
        % clear obstacles
        for k=1:length(G.obstacles)
            try delete(G.obstacles(k).h); catch, end
        end
        G.obstacles = [];
        % clear drones and their graphics
        for d=1:length(G.drones)
            if isfield(G.drones(d),'h')
                try delete(G.drones(d).h.marker); catch, end
                try delete(G.drones(d).h.trail); catch, end
            end
        end
        G.drones = [];
        redrawGraph();
        updateMenusAndDefaults();
        updateDroneList();
        updatePathInfo();
        updateTables([],[]);
    end

    function closeFigureSafely(~,~)
        try delete(fig); catch, end
    end

%% ---------- Final UI init ----------
updateMenusAndDefaults();
updateDroneList();
updatePathInfo();
rebuildGraphEdges();
redrawGraph();

end