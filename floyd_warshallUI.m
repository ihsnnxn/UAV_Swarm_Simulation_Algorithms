function floyd_warshallUI()
    % FLOYD_WARSHALLUI
    % Interactive GUI for creating, editing, and simulating shortest paths
    % using the Floyd–Warshall algorithm with drone animation.

    %% === GUI SETUP ===
    fig = figure('Name','Floyd-Warshall Drone Simulation V10', ...
                 'NumberTitle','off', ...
                 'Position',[100 100 1300 650], ...
                 'CloseRequestFcn',@closeFigureSafely);

    % Main Graph
    ax = axes('Parent',fig,'Position',[0.1 0.1 0.4 0.8]);
    axis(ax,[0 10 0 10]); grid(ax,'on'); hold(ax,'on');
    title(ax,'Network Graph','FontSize',12);

    % Path Info display (left side)
    pathText = uicontrol(fig,'Style','text','String','Shortest Path Info',...
        'Units','normalized','Position',[0.02 0.1 0.07 0.8],...
        'FontSize',10,'HorizontalAlignment','left');

    % Instructions Panel (right side)
    instructionText = uicontrol(fig,'Style','text',...
        'String',['Instructions:' newline ...
        '1️⃣ Click "Add Nodes" then click points and press Enter.' newline ...
        '2️⃣ Click "Define Edges" to connect nodes.' newline ...
        '3️⃣ Select start/end nodes and click "Start Simulation".' newline ...
        '4️⃣ Use "Move Node" or "Delete Node" to modify network.' newline ...
        '5️⃣ Press "Reset" to return to default network.'], ...
        'Units','normalized','Position',[0.8 0.1 0.18 0.8],...
        'FontSize',10,'HorizontalAlignment','left');

    % Controls Panel
    uipanel('Parent',fig,'Title','Controls','FontSize',10,...
            'Position',[0.52 0.55 0.25 0.4]);

    %% === Buttons ===
    uicontrol(fig,'Style','pushbutton','String','Add Nodes (Click)',...
        'Units','normalized','Position',[0.53 0.85 0.22 0.05],...
        'Callback',@addNodesCallback);

    uicontrol(fig,'Style','pushbutton','String','Define Edges',...
        'Units','normalized','Position',[0.53 0.78 0.22 0.05],...
        'Callback',@defineEdgesCallback);

    uicontrol(fig,'Style','pushbutton','String','Move Node',...
        'Units','normalized','Position',[0.53 0.71 0.22 0.05],...
        'Callback',@moveNodeCallback);

    uicontrol(fig,'Style','pushbutton','String','Delete Node',...
        'Units','normalized','Position',[0.53 0.64 0.22 0.05],...
        'Callback',@deleteNodeCallback);

    uicontrol(fig,'Style','pushbutton','String','Start Simulation',...
        'Units','normalized','Position',[0.53 0.57 0.22 0.05],...
        'Callback',@startSimulationCallback);

    uicontrol(fig,'Style','pushbutton','String','Reset to Defaults',...
        'Units','normalized','Position',[0.53 0.50 0.22 0.05],...
        'Callback',@resetCallback);

    %% === Dropdowns ===
    uicontrol(fig,'Style','text','String','Start Node:',...
        'Units','normalized','Position',[0.53 0.43 0.15 0.04]);
    startMenu = uicontrol(fig,'Style','popupmenu','String',{'1'},...
        'Units','normalized','Position',[0.65 0.43 0.10 0.04]);

    uicontrol(fig,'Style','text','String','End Node:',...
        'Units','normalized','Position',[0.53 0.37 0.15 0.04]);
    endMenu = uicontrol(fig,'Style','popupmenu','String',{'2'},...
        'Units','normalized','Position',[0.65 0.37 0.10 0.04]);

    %% === Tables for D and P ===
    uitable(fig,'Data',[], 'ColumnEditable',false,...
        'Units','normalized','Position',[0.52 0.1 0.12 0.25],...
        'Tag','DTable','ColumnWidth',{40});

    uitable(fig,'Data',[], 'ColumnEditable',false,...
        'Units','normalized','Position',[0.65 0.1 0.12 0.25],...
        'Tag','PTable','ColumnWidth',{40});

    %% === Data Initialization ===
    G = struct();
    resetNetwork();

    %% === CALLBACKS ===

    function addNodesCallback(~,~)
        try
            [x,y] = ginput;
        catch
            disp('Node adding cancelled.');
            return;
        end
        if isempty(x), return; end
        for i=1:numel(x)
            G.nodes(end+1,:) = [x(i), y(i)];
        end
        expandEdges();
        redrawGraph();
        updateMenus();
    end

    function defineEdgesCallback(~,~)
        if isempty(G.nodes), return; end
        disp('Click two nodes to define an edge. Press Enter to finish.');
        while true
            try
                [x,y,button] = ginput(1);
            catch
                disp('Edge definition cancelled.');
                break;
            end
            if isempty(x), break; end
            if button~=1, continue; end
            [~,idx1] = min(vecnorm(G.nodes-[x y],2,2));

            try
                [x2,y2,button2] = ginput(1);
            catch
                break;
            end
            if isempty(x2), break; end
            if button2~=1, continue; end
            [~,idx2] = min(vecnorm(G.nodes-[x2 y2],2,2));

            if idx1~=idx2
                defaultW = norm(G.nodes(idx1,:)-G.nodes(idx2,:));
                answer = inputdlg(sprintf('Weight for edge %d-%d:',idx1,idx2),...
                    'Edge Weight',[1 40],{num2str(defaultW)});
                if isempty(answer), continue; end
                w = str2double(answer{1});
                G.edges(idx1,idx2)=w; G.edges(idx2,idx1)=w;
                redrawGraph();
            end
        end
    end

    function moveNodeCallback(~,~)
        if isempty(G.nodes), return; end
        disp('Click a node to move, then click new position.');
        try
            [x,y] = ginput(1);
        catch
            return;
        end
        [~,idx] = min(vecnorm(G.nodes-[x y],2,2));
        try
            [x2,y2] = ginput(1);
        catch
            return;
        end
        G.nodes(idx,:) = [x2 y2];
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
        [~,idx] = min(vecnorm(G.nodes-[x y],2,2));
        G.nodes(idx,:) = [];
        G.edges(idx,:) = [];
        G.edges(:,idx) = [];
        redrawGraph();
        updateMenus();
    end

    function startSimulationCallback(~,~)
        if isempty(G.nodes), return; end
        startN = startMenu.Value;
        endN = endMenu.Value;

        [D,P] = floydWarshall(G.edges);
        updateTables(D,P);

        path = reconstructPath(P,startN,endN);
        if isempty(path)
            msgbox('No path exists!','Error','error');
            set(pathText,'String','No valid path found.');
            return;
        end
        cost = D(startN,endN);
        pathStr = sprintf('Path: %s\nTotal Cost: %.2f',...
            num2str(path), cost);
        set(pathText,'String',pathStr);

        animateDrone(path);
    end

    function resetCallback(~,~)
        resetNetwork();
        redrawGraph();
        updateMenus();
        set(pathText,'String','Shortest Path Info');
    end

    %% === UTILITIES ===
    function resetNetwork()
        G.nodes = [2 2; 8 2; 5 8];
        n = size(G.nodes,1);
        G.edges = inf(n);
        for i=1:n, G.edges(i,i)=0; end
        G.edges(1,2)=3; G.edges(2,3)=4; G.edges(1,3)=5;
        G.edges(2,1)=3; G.edges(3,2)=4; G.edges(3,1)=5;
    end

    function expandEdges()
        n = size(G.nodes,1);
        old = size(G.edges,1);
        if n > old
            newE = inf(n);
            newE(1:old,1:old)=G.edges;
            for i=1:n, newE(i,i)=0; end
            G.edges = newE;
        end
    end

    function redrawGraph()
        cla(ax);
        n = size(G.nodes,1);
        for i=1:n
            for j=i+1:n
                if isfinite(G.edges(i,j)) && G.edges(i,j)>0
                    line([G.nodes(i,1),G.nodes(j,1)],...
                         [G.nodes(i,2),G.nodes(j,2)],...
                         'Color','k');
                    mid=mean([G.nodes(i,:);G.nodes(j,:)]);
                    text(mid(1),mid(2),num2str(G.edges(i,j)),...
                         'Color','r','FontSize',9);
                end
            end
        end
        plot(G.nodes(:,1),G.nodes(:,2),'bo','MarkerFaceColor','b','MarkerSize',8);
        for i=1:n
            text(G.nodes(i,1)+0.2,G.nodes(i,2)+0.2,num2str(i),...
                 'Color','m','FontWeight','bold');
        end
    end

    function updateMenus()
        n = size(G.nodes,1);
        labels = arrayfun(@(i) sprintf('Node %d',i),1:n,'UniformOutput',false);
        startMenu.String = labels;
        endMenu.String = labels;
        startMenu.Value = 1;
        endMenu.Value = min(2,n);
    end

    function updateTables(D,P)
        dTable = findobj(fig,'Tag','DTable');
        pTable = findobj(fig,'Tag','PTable');
        dTable.Data = num2cell(round(D,2));
        pTable.Data = num2cell(P);
    end

    %% === ALGORITHMS ===
    function [D,P] = floydWarshall(W)
        n = size(W,1);
        D = W; P = zeros(n);
        for i=1:n
            for j=1:n
                if i~=j && isfinite(W(i,j))
                    P(i,j)=i;
                else
                    P(i,j)=0;
                end
            end
        end
        for k=1:n
            for i=1:n
                for j=1:n
                    if D(i,k)+D(k,j)<D(i,j)
                        D(i,j)=D(i,k)+D(k,j);
                        P(i,j)=P(k,j);
                    end
                end
            end
        end
    end

    function path = reconstructPath(P,i,j)
        if P(i,j)==0
            if i==j, path=i; else, path=[]; end
        else
            path=[reconstructPath(P,i,P(i,j)) j];
        end
    end

    %% === ANIMATION ===
    function animateDrone(path)
        plot(ax,G.nodes(path,1),G.nodes(path,2),'g--','LineWidth',2);
        drone = plot(ax,G.nodes(path(1),1),G.nodes(path(1),2),...
                     'ro','MarkerFaceColor','g','MarkerSize',10);
        for k=2:length(path)
            x=linspace(G.nodes(path(k-1),1),G.nodes(path(k),1),50);
            y=linspace(G.nodes(path(k-1),2),G.nodes(path(k),2),50);
            for t=1:numel(x)
                if ~ishandle(drone), return; end
                drone.XData=x(t); drone.YData=y(t);
                pause(0.03);
            end
        end
    end

    function closeFigureSafely(~,~)
        try delete(fig); catch, end
    end
end