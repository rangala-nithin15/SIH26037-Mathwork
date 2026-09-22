function V = createDemoViewer(scn, cfg, opts)
%CREATEDEMOVIEWER Build the 3D judge-demonstration viewer for a scenario.
%
%   COMPONENT STATUS: REAL (visualisation). Base MATLAB graphics only.
%
%   V = CREATEDEMOVIEWER(scn, cfg, opts) creates the figure, draws the
%   static world ONCE (terrain, drivable surface taken from the scenario's
%   occupancy grid, road edges, optional painted markings, roadside
%   buildings and trees, parked vehicles and stalls, pothole depressions,
%   goal) and creates the graphics objects that UPDATEDEMOVIEWER moves every
%   frame. Static geometry is never rebuilt while the demo runs.
%
%   HONESTY NOTE: this is a clear, schematic 3D rendering made with MATLAB
%   patch/line/text objects. It is NOT RoadRunner, NOT Unreal Engine and
%   NOT photorealistic. Nothing in it is animated independently: every
%   pose, trajectory, corridor, risk cell and label is taken from the
%   simulation state passed to UPDATEDEMOVIEWER. A photorealistic scene
%   would need RoadRunner (scene authoring) and Automated Driving Toolbox /
%   Simulink 3D Animation or the Unreal Engine co-simulation blocks, none of
%   which this project has installed or uses.
%
%   Layout
%   ------
%     left   : 3D world view (camera modes: chase / overview / top)
%     right  : mini-map, then the decision and perception panel
%     bottom : speed, risk and confidence over time
%
%   Keyboard (click the figure first):
%     1 chase   2 overview   3 top-down
%     f full    p planning   s sensing    m minimal      (visual modes)
%     space pause/resume     q stop the run
%
%   Inputs:
%       scn  - scenario struct from buildScenario()
%       cfg  - config struct from irpscConfig()   (read only, never changed)
%       opts - (optional) struct: .camera ('chase'|'overview'|'top'),
%              .visible (true), .position ([x y w h] pixels), .title (char),
%              .viz (a VIZCONFIG struct; see that file for every setting)
%
%   Outputs:
%       V - viewer struct (graphics handles and viewer state)
%
%   Requires: base MATLAB only.
%
%   See also UPDATEDEMOVIEWER, VIZCONFIG, RUNDEMO, REPLAYDEMO, ACTORMODEL3D.

if nargin < 3, opts = struct(); end
vc = getOpt(opts, 'viz', vizConfig());
if isfield(opts, 'camera') && ~isempty(opts.camera), vc.camera = opts.camera; end
visible  = getOpt(opts, 'visible', true);
figPos   = getOpt(opts, 'position', [40 40 1500 860]);
titleStr = getOpt(opts, 'title', 'IR-PSC autonomous driving demonstration');

V = struct();
V.scn = scn;
V.cfg = cfg;
V.vc  = vc;
V.pal = vc.color;
V.vp  = vehicleParams(cfg);
pal   = V.pal;

if visible, vis = 'on'; else, vis = 'off'; end
V.fig = figure('Name', titleStr, 'NumberTitle', 'off', 'Color', pal.bg, ...
               'Position', figPos, 'Visible', vis, 'MenuBar', 'none', ...
               'InvertHardcopy', 'off', 'KeyPressFcn', @onKey);
setappdata(V.fig, 'viewerCtl', struct('camera', vc.camera, 'paused', false, ...
                                      'quit', false, 'visualMode', vc.visualMode));

% ---------------------------------------------------------------------
% Axes
% ---------------------------------------------------------------------
wideWorld = ~vc.show.hud;
if wideWorld, worldW = 0.99; else, worldW = 0.705; end
if vc.show.timeseries, worldBottom = 0.19; else, worldBottom = 0.02; end

V.ax = axes('Parent', V.fig, 'Units', 'normalized', ...
            'Position', [0.0 worldBottom worldW 1 - worldBottom - 0.005], ...
            'Color', pal.sky, 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
hold(V.ax, 'on');
axis(V.ax, 'equal');
set(V.ax, 'XTick', [], 'YTick', [], 'ZTick', [], 'Box', 'off', ...
          'Projection', 'perspective', 'CameraViewAngleMode', 'manual', ...
          'CameraViewAngle', 38, 'Clipping', 'off');

V.axMap = axes('Parent', V.fig, 'Units', 'normalized', 'Position', [0.715 0.70 0.28 0.29], ...
               'Color', [0.086 0.102 0.130], 'XColor', pal.panelEdge, 'YColor', pal.panelEdge, ...
               'FontSize', 7, 'Box', 'on');
hold(V.axMap, 'on');
axis(V.axMap, 'equal');

V.axHud = axes('Parent', V.fig, 'Units', 'normalized', 'Position', [0.715 0.0 0.28 0.69], ...
               'XLim', [0 1], 'YLim', [0 1], 'Visible', 'off');
hold(V.axHud, 'on');

V.axTs = axes('Parent', V.fig, 'Units', 'normalized', 'Position', [0.045 0.035 0.64 0.13], ...
              'Color', [0.078 0.090 0.110], 'XColor', pal.textMuted, 'YColor', pal.textMuted, ...
              'FontSize', 8, 'Box', 'on', 'YLim', [0 1.05]);
hold(V.axTs, 'on');
xlabel(V.axTs, 'time (s)');

if ~vc.show.minimap,    set(V.axMap, 'Visible', 'off'); end
if ~vc.show.timeseries, set(V.axTs,  'Visible', 'off'); end
if ~vc.show.hud,        set(V.axHud, 'Visible', 'off'); end

% ---------------------------------------------------------------------
% Static world (drawn once, never rebuilt)
% ---------------------------------------------------------------------
drawTerrainAndRoad(V);
if vc.show.markings, drawMarkings(V); end
drawStaticObjects(V);
drawPotholeGround(V);
drawGoal(V);

light('Parent', V.ax, 'Position', [0.35 -0.55 1], 'Style', 'infinite');
light('Parent', V.ax, 'Position', [-0.6 0.4 0.8], 'Style', 'infinite', 'Color', [0.30 0.34 0.42]);

% ---------------------------------------------------------------------
% Dynamic objects (created once here, only updated afterwards)
% ---------------------------------------------------------------------
a = V.ax;
al = vc.alpha;

% Every TRANSLUCENT overlay has FaceLighting 'none': in GNU Octave a lit
% patch ignores FaceAlpha when the axes has a light, so the corridor and the
% predicted-occupancy regions would otherwise hide what lies beneath them.
% -- drivable space ---------------------------------------------------
V.h.corridor   = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                       'FaceColor', pal.corridor, 'FaceAlpha', al.corridor, 'EdgeColor', 'none', ...
                       'FaceLighting', 'none');
V.h.corrEdges  = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.corridorEdge, 'LineWidth', 1.8);
% -- risk and pothole cost -------------------------------------------
V.h.riskCells  = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                       'FaceVertexCData', pal.riskHigh, 'FaceColor', 'flat', ...
                       'FaceAlpha', al.riskCell, 'EdgeColor', 'none', 'FaceLighting', 'none');
V.h.potholeCells = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                       'FaceColor', pal.potholeCost, 'FaceAlpha', al.potholeCost, 'EdgeColor', 'none', ...
                       'FaceLighting', 'none');
% -- prediction -------------------------------------------------------
for i = 1:3
    V.h.predOcc(i) = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                           'FaceColor', pal.predOcc, 'FaceAlpha', al.predOcc * (1.15 - 0.25*i), ...
                           'EdgeColor', pal.predOcc, 'EdgeAlpha', 0.35, 'FaceLighting', 'none');
end
V.h.predPaths  = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.predPath, 'LineStyle', ':', 'LineWidth', 1.4);
% -- paths ------------------------------------------------------------
V.h.prefPath   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.preferred, 'LineStyle', '--', 'LineWidth', 1.3);
V.h.trail      = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.trail, 'LineWidth', 1.6);
V.h.rejected   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.rejected, 'LineStyle', '--', 'LineWidth', 1.5);
V.h.trajRibbon = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                       'FaceColor', pal.traj, 'FaceAlpha', al.trajRibbon, 'EdgeColor', 'none', ...
                       'FaceLighting', 'none');
V.h.trajLine   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.traj, 'LineWidth', 2.6);
V.h.stopWall   = patch('Parent', a, 'Vertices', nan(4,3), 'Faces', [1 2 3 4], ...
                       'FaceColor', pal.stopWall, 'FaceAlpha', al.stopWall, 'EdgeColor', pal.stopWall, ...
                       'FaceLighting', 'none');
% -- perception -------------------------------------------------------
V.h.tracks     = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.track, 'LineWidth', 1.2);
V.h.detCam     = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                      'Marker', 's', 'MarkerSize', 4, 'MarkerFaceColor', pal.detCam, 'MarkerEdgeColor', 'none');
V.h.detLidar   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                      'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', pal.detLidar, 'MarkerEdgeColor', 'none');
V.h.detRadar   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                      'Marker', 'd', 'MarkerSize', 4, 'MarkerFaceColor', pal.detRadar, 'MarkerEdgeColor', 'none');
% -- potholes ---------------------------------------------------------
V.h.potDets    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                      'Marker', 'x', 'MarkerSize', 6, 'LineWidth', 1.4, 'Color', pal.potholeMod);
V.h.potRings   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.potholeMod, 'LineWidth', 2.2);
V.h.potTentative = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                      'Color', pal.textMuted, 'LineWidth', 1.0, 'LineStyle', '--');
% -- ego --------------------------------------------------------------
V.egoModel = actorModel3D('ego', V.vp.length, V.vp.width, true, pal);
V.h.ego    = patch('Parent', a, 'Vertices', V.egoModel.V, 'Faces', V.egoModel.F, ...
                   'FaceVertexCData', V.egoModel.C, 'FaceColor', 'flat', ...
                   'EdgeColor', 'none', 'FaceLighting', 'gouraud');
V.h.egoRing = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                   'Color', pal.egoAccent, 'LineWidth', 1.5);
% -- overlays ---------------------------------------------------------
V.h.banner    = text(0.5, 0.88, '', 'Parent', a, 'Units', 'normalized', ...
                     'HorizontalAlignment', 'center', 'FontSize', 17, 'FontWeight', 'bold', ...
                     'Color', [1 1 1], 'BackgroundColor', pal.stateStop, 'Margin', 5, 'Visible', 'off');
V.h.camText   = text(0.99, 0.975, '', 'Parent', a, 'Units', 'normalized', ...
                     'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
                     'FontSize', 7.5, 'Color', pal.textMuted, 'Interpreter', 'none');
V.h.perfText  = text(0.01, 0.045, '', 'Parent', a, 'Units', 'normalized', ...
                     'HorizontalAlignment', 'left', 'FontSize', 8, 'Color', pal.textMuted, ...
                     'FontName', 'monospaced', 'Interpreter', 'none', ...
                     'Visible', onOff(vc.show.perfHud));
V.h.modeText  = text(0.01, 0.975, '', 'Parent', a, 'Units', 'normalized', ...
                     'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
                     'FontSize', 9, 'FontWeight', 'bold', 'Color', pal.accent, 'Interpreter', 'none');

% Pools, grown on demand (never rebuilt once grown)
V.actorIds = [];  V.actorH = [];  V.actorModels = {};
V.trackLbl = [];  V.potLbl = [];

% ---------------------------------------------------------------------
% Mini-map (static part)
% ---------------------------------------------------------------------
g = scn.grid;
step = max(1, round(0.6 / g.resolution));
free = ~g.occ(1:step:end, 1:step:end);
xm = g.origin(1) + (0:size(free,2)-1) * step * g.resolution;
ym = g.origin(2) + (0:size(free,1)-1) * step * g.resolution;
imagesc(V.axMap, xm, ym, double(free));
colormap(V.axMap, [0.098 0.118 0.106; 0.243 0.259 0.290]);
set(V.axMap, 'YDir', 'normal');
plot(V.axMap, scn.goal(1), scn.goal(2), 'p', 'MarkerSize', 11, ...
     'MarkerFaceColor', [0.933 0.769 0.200], 'MarkerEdgeColor', 'none');
for p = 1:numel(scn.potholes)
    plot(V.axMap, scn.potholes(p).pos(1), scn.potholes(p).pos(2), '.', ...
         'Color', pal.potholeCost, 'MarkerSize', 9);
end
V.h.mapTraj   = plot(V.axMap, NaN, NaN, '-', 'Color', pal.traj, 'LineWidth', 1.8);
V.h.mapTrail  = plot(V.axMap, NaN, NaN, '-', 'Color', pal.trail, 'LineWidth', 1);
V.h.mapActors = plot(V.axMap, NaN, NaN, 'o', 'MarkerSize', 3, ...
                     'MarkerFaceColor', pal.actor.car, 'MarkerEdgeColor', 'none');
V.h.mapEgo    = plot(V.axMap, NaN, NaN, 's', 'MarkerSize', 7, ...
                     'MarkerFaceColor', pal.egoAccent, 'MarkerEdgeColor', 'none');
set(V.axMap, 'XTick', [], 'YTick', []);
title(V.axMap, 'DRIVABLE SPACE (occupancy grid)', 'Color', pal.textMuted, ...
      'FontSize', 7.5, 'FontWeight', 'normal');
V.mapHalfWindow = 60;

% ---------------------------------------------------------------------
% Time series
% ---------------------------------------------------------------------
V.h.tsSpeed = plot(V.axTs, NaN, NaN, '-', 'Color', pal.traj, 'LineWidth', 1.6);
V.h.tsRisk  = plot(V.axTs, NaN, NaN, '-', 'Color', pal.riskHigh, 'LineWidth', 1.4);
V.h.tsConf  = plot(V.axTs, NaN, NaN, '-', 'Color', pal.corridorEdge, 'LineWidth', 1.2);
V.h.tsStop  = plot(V.axTs, NaN, NaN, 'LineStyle', 'none', 'Marker', '.', ...
                   'Color', pal.stateStop, 'MarkerSize', 8);
text(0.005, 0.93, 'speed / max speed (cyan)   risk (red)   confidence (green)   safe stop (red dots)', ...
     'Parent', V.axTs, 'Units', 'normalized', 'Color', pal.textMuted, 'FontSize', 7, ...
     'VerticalAlignment', 'top', 'Interpreter', 'none');

% ---------------------------------------------------------------------
% HUD
% ---------------------------------------------------------------------
V = createHud(V);

% ---------------------------------------------------------------------
% Layer groups (for the visual modes) and the dirty-check cache
% ---------------------------------------------------------------------
V.group.corridor   = [V.h.corridor, V.h.corrEdges];
V.group.risk       = V.h.riskCells;
V.group.potholeCost= V.h.potholeCells;
V.group.prediction = V.h.predOcc;
V.group.predPaths  = V.h.predPaths;
V.group.tracks     = V.h.tracks;
V.group.detections = [V.h.detCam, V.h.detLidar, V.h.detRadar];
V.group.potholes   = [V.h.potDets, V.h.potRings, V.h.potTentative];
V.group.preferred  = V.h.prefPath;
V.group.rejected   = V.h.rejected;
V.group.trail      = V.h.trail;
V.group.trajectory = [V.h.trajRibbon, V.h.trajLine];
V = applyVisualMode(V, vc.visualMode);

V.cache = struct();
V.stats = struct('writes', 0);
V.camPos = [];  V.camTgt = [];  V.camMode = '';  V.camTime = 0;
V.trailPos = zeros(0,3);
V.frameCount = 0;
V.renderIndex = 0;
V.eventLog = {};
V.evState = struct('lastState', '', 'seenTracks', [], 'seenPotholes', [], ...
                   'lastPlanNote', '', 'lastRiskNote', '', 'lastPredNote', '', ...
                   'lastEmergency', false);
V.perf = struct('target', vc.displayFps, 'fps', 0, 'frameMs', 0, 'simMs', 0, 'renderMs', 0);
end

% =========================================================================
function drawTerrainAndRoad(V)
pal = V.pal;
g = V.scn.grid;
res = g.resolution;
x0 = g.origin(1) - res/2;  x1 = g.origin(1) + (g.nCols - 0.5) * res;
y0 = g.origin(2) - res/2;  y1 = g.origin(2) + (g.nRows - 0.5) * res;
pad = 60;
[TX, TY] = meshgrid(linspace(x0-pad, x1+pad, 12), linspace(y0-pad, y1+pad, 6));
Vt = [TX(:), TY(:), -0.06*ones(numel(TX),1)];
[nr, nc] = size(TX);
Ft = zeros((nr-1)*(nc-1), 4);  q = 0;
for c = 1:nc-1
    for r = 1:nr-1
        q = q + 1;
        i1 = (c-1)*nr + r;
        Ft(q,:) = [i1, i1 + nr, i1 + nr + 1, i1 + 1];
    end
end
patch('Parent', V.ax, 'Vertices', Vt, 'Faces', Ft, 'FaceColor', pal.terrain, ...
      'EdgeColor', 'none', 'FaceLighting', 'none');

% Drivable surface: one quad per horizontal run of free cells.
free = ~g.occ;
Vv = zeros(0,3);  Ff = zeros(0,4);
for r = 1:g.nRows
    row = free(r, :);
    if ~any(row), continue; end
    d = diff([false, row, false]);
    starts = find(d == 1);
    stops  = find(d == -1) - 1;
    y = g.origin(2) + (r-1) * res;
    for q = 1:numel(starts)
        xa = g.origin(1) + (starts(q)-1) * res - res/2;
        xb = g.origin(1) + (stops(q)-1)  * res + res/2;
        n0 = size(Vv,1);
        Vv = [Vv; xa y-res/2 0; xb y-res/2 0; xb y+res/2 0; xa y+res/2 0]; %#ok<AGROW>
        Ff = [Ff; n0+1 n0+2 n0+3 n0+4]; %#ok<AGROW>
    end
end
patch('Parent', V.ax, 'Vertices', Vv, 'Faces', Ff, 'FaceColor', pal.road, ...
      'EdgeColor', 'none', 'FaceLighting', 'none');

% Shoulders and a soft road edge where the scenario width is known.
hw = V.scn.roadHalfWidth(:);
if all(isfinite(hw))
    cl = V.scn.centerline;
    th = pathHeading(cl);
    n  = [-sin(th), cos(th)];
    K  = size(cl,1);
    Vs = zeros(0,3);  Fs = zeros(0,4);  EX = [];  EY = [];
    for side = [-1 1]
        inner = cl + side * repmat(hw, 1, 2) .* n;
        outer = cl + side * repmat(hw + 1.3, 1, 2) .* n;
        n0 = size(Vs,1);
        Fs = [Fs; n0 + [(1:K-1).', (2:K).', K + (2:K).', K + (1:K-1).']]; %#ok<AGROW>
        Vs = [Vs; inner, -0.03*ones(K,1); outer, -0.03*ones(K,1)];        %#ok<AGROW>
        EX = [EX; inner(:,1); NaN];  EY = [EY; inner(:,2); NaN];          %#ok<AGROW>
    end
    patch('Parent', V.ax, 'Vertices', Vs, 'Faces', Fs, 'FaceColor', pal.shoulder, ...
          'EdgeColor', 'none', 'FaceLighting', 'none');
    line('Parent', V.ax, 'XData', EX, 'YData', EY, 'ZData', 0.015*ones(size(EX)), ...
         'Color', pal.roadEdge, 'LineWidth', 1.2);
end
end

% =========================================================================
function drawMarkings(V)
mk = V.scn.markings;
hw = V.scn.roadHalfWidth(:);
if ~isstruct(mk) || mk.sTo <= mk.sFrom || ~all(isfinite(hw))
    return;
end
cl = V.scn.centerline;
sCl = pathArcLength(cl);
z = 0.012;
Vv = zeros(0,3);  Ff = zeros(0,4);
for s0 = mk.sFrom:6:mk.sTo-3
    [Vv, Ff] = addStrip(Vv, Ff, cl, s0, s0 + 3, 0, 0.12, z);
end
for sideOff = [-1 1]
    for s0 = mk.sFrom:2:mk.sTo-2
        hwAt = interp1(sCl, hw, s0 + 1, 'linear', 'extrap');
        [Vv, Ff] = addStrip(Vv, Ff, cl, s0, s0 + 2.05, sideOff * (hwAt - 0.25), 0.12, z);
    end
end
patch('Parent', V.ax, 'Vertices', Vv, 'Faces', Ff, 'FaceColor', V.pal.marking, ...
      'EdgeColor', 'none', 'FaceLighting', 'none');
pEnd = frenetToCartesian(cl, mk.sTo + 4, 0);
text(pEnd(1), pEnd(2), 0.3, 'markings end - planner never used them', 'Parent', V.ax, ...
     'Color', V.pal.textMuted, 'FontSize', 8, 'HorizontalAlignment', 'center');
end

function [Vv, Ff] = addStrip(Vv, Ff, cl, sA, sB, d, w, z)
s  = [sA; sB];
pc = frenetToCartesian(cl, s, d);
th = atan2(pc(2,2) - pc(1,2), pc(2,1) - pc(1,1));
n  = [-sin(th), cos(th)] * w / 2;
n0 = size(Vv,1);
Vv = [Vv; pc(1,:) - n, z; pc(2,:) - n, z; pc(2,:) + n, z; pc(1,:) + n, z];
Ff = [Ff; n0+1 n0+2 n0+3 n0+4];
end

% =========================================================================
function drawStaticObjects(V)
%DRAWSTATICOBJECTS Buildings, trees, parked vehicles, stalls, debris -- all
%merged into ONE patch. Octave repaints the whole canvas whenever anything
%moves, and each graphics object adds overhead to every repaint, so fewer
%objects means a cheaper frame. These never move, so nothing is lost.
pal = V.pal;
so = V.scn.staticObjects;
Vall = zeros(0,3);  Fall = zeros(0,4);  Call = zeros(0,3);
for i = 1:numel(so)
    o = so(i);
    switch o.type
        case 'building'
            shade = 1 + 0.10 * (mod(i, 3) - 1);
            m = boxModel([2*o.halfSize(1), 2*o.halfSize(2), o.height], min(pal.building*shade, 1));
            r = boxModel([2*o.halfSize(1)*1.08, 2*o.halfSize(2)*1.08, 0.28], pal.buildingRoof);
            r.V(:,3) = r.V(:,3) + o.height;
            m = mergeModels(m, r);
        case 'tree'
            m = boxModel([0.30, 0.30, o.height*0.55], pal.treeTrunk);
            c = boxModel([2*o.radius*1.5, 2*o.radius*1.5, o.height*0.50], pal.tree);
            c.V(:,3) = c.V(:,3) + o.height*0.45;
            c2 = boxModel([2*o.radius*1.0, 2*o.radius*1.0, o.height*0.30], pal.tree*1.15);
            c2.V(:,3) = c2.V(:,3) + o.height*0.80;
            m = mergeModels(mergeModels(m, c), c2);
        case 'truck'
            m = actorModel3D('truck', 2*o.halfSize(1), 2*o.halfSize(2), false, pal);
            m.C = 0.55 * m.C + 0.45 * repmat(pal.staticObj, size(m.C,1), 1);  % parked: muted
        case 'cart'
            m = actorModel3D('pushcart', 2*o.halfSize(1), 2*o.halfSize(2), false, pal);
            m.C = 0.55 * m.C + 0.45 * repmat(pal.staticObj, size(m.C,1), 1);
        case 'stall'
            m = boxModel([2*o.halfSize(1), 2*o.halfSize(2), 0.9], pal.staticObj);
            c = boxModel([2*o.halfSize(1)*1.2, 2*o.halfSize(2)*1.3, 0.10], [0.529 0.318 0.259]);
            c.V(:,3) = c.V(:,3) + o.height - 0.10;
            m = mergeModels(m, c);
            for sx = [-1 1]
                for sy = [-1 1]
                    p = boxModel([0.07 0.07 o.height], pal.staticObj*0.7);
                    p.V(:,1) = p.V(:,1) + sx * o.halfSize(1);
                    p.V(:,2) = p.V(:,2) + sy * o.halfSize(2);
                    m = mergeModels(m, p);
                end
            end
        case 'debris'
            m = boxModel([2*o.radius, 2*o.radius, o.height], pal.staticObj*0.85);
        otherwise
            if ~isempty(o.halfSize)
                m = boxModel([2*o.halfSize(1), 2*o.halfSize(2), max(o.height, 0.5)], pal.staticObj);
            else
                m = boxModel([2*o.radius, 2*o.radius, max(o.height, 0.5)], pal.staticObj);
            end
    end
    Vw = placeModel(m.V, o.center, o.yaw);
    Fall = [Fall; m.F + size(Vall,1)]; %#ok<AGROW>
    Vall = [Vall; Vw];                 %#ok<AGROW>
    Call = [Call; m.C];                %#ok<AGROW>
end
if ~isempty(Fall)
    patch('Parent', V.ax, 'Vertices', Vall, 'Faces', Fall, 'FaceVertexCData', Call, ...
          'FaceColor', 'flat', 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
end
end

% =========================================================================
function drawPotholeGround(V)
%DRAWPOTHOLEGROUND Dark depressions: all floors in one patch, all sloped rims
%in another (true scenario potholes -- the ground, not a detection).
pal = V.pal;
ph = V.scn.potholes;
if isempty(ph), return; end
nT = 20;
t = linspace(0, 2*pi, nT+1).';  t(end) = [];
Vf = zeros(0,3);  Ff = zeros(0,nT);  Vr = zeros(0,3);  Fr = zeros(0,4);
nxt = [2:nT, 1].';
for p = 1:numel(ph)
    e = [ph(p).length/2 * cos(t), ph(p).width/2 * sin(t)];
    P = placeModel([e, zeros(nT,1)], ph(p).pos, ph(p).yaw);
    inner = [ph(p).pos + 0.75*(P(:,1:2) - ph(p).pos), -ph(p).depth*ones(nT,1)];
    Ff = [Ff; size(Vf,1) + (1:nT)];                                        %#ok<AGROW>
    Vf = [Vf; inner];                                                      %#ok<AGROW>
    r0 = size(Vr,1);
    Fr = [Fr; r0 + [(1:nT).', nxt, nT + nxt, nT + (1:nT).']];             %#ok<AGROW>
    Vr = [Vr; P(:,1:2), 0.005*ones(nT,1); inner];                          %#ok<AGROW>
end
patch('Parent', V.ax, 'Vertices', Vf, 'Faces', Ff, 'FaceColor', pal.potholeFloor, ...
      'EdgeColor', 'none', 'FaceLighting', 'none');
patch('Parent', V.ax, 'Vertices', Vr, 'Faces', Fr, 'FaceColor', pal.potholeRim, ...
      'EdgeColor', 'none', 'FaceLighting', 'none');
end

% =========================================================================
function drawGoal(V)
g = V.scn.goal;
% Column vectors throughout: Octave stores patch X/Y data as columns, and a
% row ZData then fails its size check ("x/y/zdata must have the same
% dimensions"), so the goal was silently not drawn in some renders.
t = linspace(0, 2*pi, 32).';
r = V.scn.goalRadius;
gold = [0.933 0.769 0.200];
patch('Parent', V.ax, 'XData', g(1) + r*cos(t), 'YData', g(2) + r*sin(t), ...
      'ZData', 0.02*ones(size(t)), 'FaceColor', gold, 'FaceAlpha', 0.14, ...
      'EdgeColor', gold, 'LineWidth', 1.5, 'FaceLighting', 'none');
line('Parent', V.ax, 'XData', [g(1); g(1)], 'YData', [g(2); g(2)], 'ZData', [0; 5], ...
     'Color', [0.75 0.78 0.82], 'LineWidth', 2);
patch('Parent', V.ax, 'XData', g(1) + [0; 0; 2.2], 'YData', g(2) + [0; 0; 0], ...
      'ZData', [5; 3.8; 4.4], 'FaceColor', gold, 'EdgeColor', 'none');
text(g(1), g(2), 5.6, 'GOAL', 'Parent', V.ax, 'Color', gold, 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'FontSize', 10);
end

% =========================================================================
function V = createHud(V)
%CREATEHUD Compact, sectioned panel: one row per thing a judge must read.
a = V.axHud;
pal = V.pal;
txt = @(x, y, s, sz, col, w) text(x, y, s, 'Parent', a, 'Units', 'data', 'FontSize', sz, ...
            'Color', col, 'FontWeight', w, 'VerticalAlignment', 'middle', 'Interpreter', 'none');

% Panel background
rectangle('Parent', a, 'Position', [0.015 0.005 0.97 0.985], ...
          'FaceColor', pal.panel, 'EdgeColor', pal.panelEdge, 'LineWidth', 1);

% -- identity ---------------------------------------------------------
txt(0.045, 0.962, 'IR-PSC', 16, pal.accent, 'bold');
txt(0.045, 0.930, 'PREDICTIVE SAFETY CORRIDOR', 7.5, pal.textMuted, 'normal');
V.hud.scen = txt(0.045, 0.905, '', 7, pal.textMuted, 'normal');
line('Parent', a, 'XData', [0.045 0.955], 'YData', [0.890 0.890], 'Color', pal.panelEdge);

% -- state badge ------------------------------------------------------
V.hud.stateBox = rectangle('Parent', a, 'Position', [0.045 0.800 0.91 0.082], ...
                           'FaceColor', pal.stateNormal, 'EdgeColor', 'none');
V.hud.state    = txt(0.075, 0.855, 'CRUISE', 15, [1 1 1], 'bold');
V.hud.stateSub = txt(0.075, 0.819, '', 7, [0.88 0.92 0.96], 'normal');

% -- rows -------------------------------------------------------------
y = 0.772;  dy = 0.049;
V.hud.rowRisk = hudRow(V, a, txt, 'RISK',       y);            y = y - dy;
V.hud.rowConf = hudRow(V, a, txt, 'CONFIDENCE', y);            y = y - dy;
V.hud.rowTtc  = hudRow(V, a, txt, 'TTC',        y, false);     y = y - dy;
V.hud.rowSpd  = hudRow(V, a, txt, 'SPEED',      y, false);     y = y - dy;
V.hud.rowDet  = hudRow(V, a, txt, 'DETECTIONS', y, false);     y = y - dy;
V.hud.rowPot  = hudRow(V, a, txt, 'POTHOLES',   y, false);     y = y - dy;
V.hud.rowPlan = hudRow(V, a, txt, 'PLANNER',    y, false);     y = y - dy;

line('Parent', a, 'XData', [0.045 0.955], 'YData', [y+0.022 y+0.022], 'Color', pal.panelEdge);

% -- pothole detail ---------------------------------------------------
y = y - 0.010;
txt(0.045, y, 'POTHOLE TRACKS', 7.5, pal.accent, 'bold');
for i = 1:V.vc.potholeLines
    y = y - 0.028;
    V.hud.pot(i) = txt(0.045, y, '', 7, pal.text, 'normal');
end

% -- event log --------------------------------------------------------
y = y - 0.042;
line('Parent', a, 'XData', [0.045 0.955], 'YData', [y+0.020 y+0.020], 'Color', pal.panelEdge);
txt(0.045, y, 'DECISION LOG', 7.5, pal.accent, 'bold');
for i = 1:V.vc.eventLines
    y = y - 0.029;
    V.hud.ev(i) = txt(0.045, y, '', 7.5, pal.text, 'normal');
end

% -- footer -----------------------------------------------------------
txt(0.045, 0.048, 'Simulation only. Schematic MATLAB graphics, not RoadRunner.', ...
    6.5, pal.textMuted, 'normal');
txt(0.045, 0.024, 'Simplified sensor, vehicle and scenario models.', ...
    6.5, pal.textMuted, 'normal');

V.hud.reason = txt(0.045, 0.076, '', 7, pal.textMuted, 'normal');
end

function row = hudRow(V, a, txt, label, y, withBar)
%HUDROW One label/value line, optionally with a bar behind the value.
if nargin < 6, withBar = true; end
pal = V.pal;
row.label = txt(0.045, y, label, 7.5, pal.textMuted, 'normal');
if withBar
    rectangle('Parent', a, 'Position', [0.42 y-0.011 0.535 0.022], ...
              'FaceColor', [0.145 0.165 0.196], 'EdgeColor', 'none');
    row.bar = rectangle('Parent', a, 'Position', [0.42 y-0.011 0.001 0.022], ...
                        'FaceColor', pal.corridorEdge, 'EdgeColor', 'none');
    row.value = txt(0.435, y, '', 8.5, [1 1 1], 'bold');
else
    row.bar = [];
    row.value = txt(0.42, y, '', 9, pal.text, 'bold');
end
end

% =========================================================================
function onKey(src, evt)
ctl = getappdata(src, 'viewerCtl');
switch lower(evt.Key)
    case {'1'}, ctl.camera = 'chase';
    case {'2'}, ctl.camera = 'overview';
    case {'3'}, ctl.camera = 'top';
    case {'f'}, ctl.visualMode = 'full';
    case {'p'}, ctl.visualMode = 'planning';
    case {'s'}, ctl.visualMode = 'sensing';
    case {'m'}, ctl.visualMode = 'minimal';
    case {'space'}, ctl.paused = ~ctl.paused;
    case {'q', 'escape'}, ctl.quit = true;
end
setappdata(src, 'viewerCtl', ctl);
end

% =========================================================================
function m = boxModel(sz, col)
hx = sz(1)/2;  hy = sz(2)/2;  hz = sz(3);
m.V = [-hx -hy 0; hx -hy 0; hx hy 0; -hx hy 0; -hx -hy hz; hx -hy hz; hx hy hz; -hx hy hz];
m.F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
m.C = min(repmat(col, 6, 1) .* repmat([0.72; 1.00; 0.86; 0.94; 0.86; 0.94], 1, 3), 1);
end

function m = mergeModels(a, b)
m.V = [a.V; b.V];
m.F = [a.F; b.F + size(a.V,1)];
m.C = [a.C; b.C];
end

function W = placeModel(Vb, pos, yaw)
c = cos(yaw);  s = sin(yaw);
W = [Vb(:,1)*c - Vb(:,2)*s + pos(1), Vb(:,1)*s + Vb(:,2)*c + pos(2), Vb(:,3)];
end

function s = onOff(tf)
if tf, s = 'on'; else, s = 'off'; end
end

function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
