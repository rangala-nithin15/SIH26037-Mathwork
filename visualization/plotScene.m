function h = plotScene(scn, ego, plan, obstacles, opts)
%PLOTSCENE Draw the drivable space, corridor, hazards and planned path.
%
%   COMPONENT STATUS: REAL
%
%   h = PLOTSCENE(scn, ego, plan, obstacles, opts) renders one frame of the
%   simulation.
%
%   This figure is the main demonstration artefact. It is laid out so the
%   IR-PSC argument is visible rather than needing to be asserted:
%
%     - drivable space is drawn as free area, with NO lane markings drawn,
%       because none exist. A judge can see that the planner has no lane to
%       follow.
%     - the extracted corridor boundaries are drawn as dashed lines, showing
%       that road edges were inferred from free space alone.
%     - the corridor centreline (the preferred path) is drawn separately
%       from the final trajectory, so the DEFORMATION between them is
%       visible as a gap. That gap is the contribution, made visual.
%     - predicted occupancy is drawn as growing uncertainty ellipses, so
%       "the system is less sure about the motorcycle than the bus" is
%       something you can see rather than something you have to be told.
%
%   Inputs:
%       scn       - scenario struct from buildScenario()
%       ego       - ego state struct
%       plan      - planner output struct
%       obstacles - obstacle struct array (ground truth or tracks)
%       opts      - (optional) struct with fields:
%                   .showGrid        logical (default true)
%                   .showPredictions logical (default true)
%                   .showCorridor    logical (default true)
%                   .axesHandle      target axes
%                   .followEgo       logical, centre the view on the ego
%
%   Outputs:
%       h - struct of graphics handles, for animation updates
%
%   Example:
%       plotScene(scn, ego, plan, obstacles);
%
%   Requires: base MATLAB only (uses plot, patch, fill).
%
%   See also RUNSCENARIO, PLOTCORRIDOR.

if nargin < 5, opts = struct(); end
showGrid  = getOpt(opts,'showGrid', true);
showPred  = getOpt(opts,'showPredictions', true);
showCorr  = getOpt(opts,'showCorridor', true);
followEgo = getOpt(opts,'followEgo', true);

if isfield(opts,'axesHandle') && ~isempty(opts.axesHandle)
    ax = opts.axesHandle;
else
    ax = gca;
end
cla(ax); hold(ax,'on'); axis(ax,'equal');

h = struct();

% --- Drivable space ------------------------------------------------------
if showGrid && isfield(scn,'grid')
    g = scn.grid;
    [cols, rows] = meshgrid(1:g.nCols, 1:g.nRows);
    X = g.origin(1) + (cols-1)*g.resolution;
    Y = g.origin(2) + (rows-1)*g.resolution;
    freeX = X(~g.occ);
    freeY = Y(~g.occ);
    h.free = plot(ax, freeX, freeY, '.', 'Color', [0.87 0.87 0.87], ...
                  'MarkerSize', 2);
end

% --- Static obstacles in the grid (Phase 2: previously invisible) ---------
if isfield(scn, 'staticObjects')
    for i = 1:numel(scn.staticObjects)
        so = scn.staticObjects(i);
        if ~so.inGrid, continue; end
        if ~isempty(so.halfSize)
            drawRect(ax, so.center, 2*so.halfSize(1), 2*so.halfSize(2), so.yaw, [0.35 0.30 0.25], 0.9);
        else
            tt = linspace(0, 2*pi, 20);
            patch(ax, so.center(1) + so.radius*cos(tt), so.center(2) + so.radius*sin(tt), ...
                  [0.35 0.30 0.25], 'EdgeColor', 'none');
        end
    end
end
if isfield(scn, 'potholes')
    for i = 1:numel(scn.potholes)
        ph = scn.potholes(i);
        tt = linspace(0, 2*pi, 24);
        E = [ph.length/2*cos(tt); ph.width/2*sin(tt)];
        Rm = [cos(ph.yaw) -sin(ph.yaw); sin(ph.yaw) cos(ph.yaw)];
        Pe = (Rm * E).';
        patch(ax, Pe(:,1) + ph.pos(1), Pe(:,2) + ph.pos(2), [0.15 0.12 0.10], 'EdgeColor', [0.9 0.5 0.1]);
    end
end

% --- Corridor ------------------------------------------------------------
if showCorr && isfield(plan,'corridor') && plan.corridor.valid
    c = plan.corridor;
    h.left   = plot(ax, c.left(:,1),  c.left(:,2),  '--', ...
                    'Color',[0.20 0.45 0.75], 'LineWidth', 1.3);
    h.right  = plot(ax, c.right(:,1), c.right(:,2), '--', ...
                    'Color',[0.20 0.45 0.75], 'LineWidth', 1.3);
    h.center = plot(ax, c.center(:,1), c.center(:,2), ':', ...
                    'Color',[0.45 0.45 0.45], 'LineWidth', 1.2);
end

% --- Predicted occupancy -------------------------------------------------
if showPred && isfield(plan,'preds') && ~isempty(plan.preds)
    for k = 1:numel(plan.preds)
        p = plan.preds(k);
        step = max(1, round(numel(p.times)/5));
        for i = 1:step:numel(p.times)
            drawEllipse(ax, p.pos(i,:), ...
                        p.radius + 2*p.sigmaLong(i), ...
                        p.radius + 2*p.sigmaLat(i), ...
                        p.heading(i), [0.95 0.62 0.25], ...
                        0.10 + 0.10*(1 - i/numel(p.times)));
        end
    end
end

% --- Road users ----------------------------------------------------------
for k = 1:numel(obstacles)
    o = obstacles(k);
    if o.vulnerable
        col = [0.85 0.20 0.20];
    else
        col = [0.30 0.30 0.65];
    end
    drawRect(ax, o.pos, o.length, o.width, o.heading, col, 0.55);
    text(ax, o.pos(1), o.pos(2)+1.2, o.class, 'FontSize', 7, ...
         'HorizontalAlignment','center', 'Color', col*0.7);
end

% --- Planned trajectory --------------------------------------------------
if isfield(plan,'traj') && isfield(plan.traj,'pos') && size(plan.traj.pos,1) >= 2
    tr = plan.traj;
    if isfield(plan,'isSafeStop') && plan.isSafeStop
        trajCol = [0.85 0.10 0.10];
        trajStyle = '-';
    else
        trajCol = [0.10 0.65 0.25];
        trajStyle = '-';
    end
    h.traj = plot(ax, tr.pos(:,1), tr.pos(:,2), trajStyle, ...
                  'Color', trajCol, 'LineWidth', 2.2);
end

% --- Ego vehicle ---------------------------------------------------------
% Phase 2 fix: ego.pos is the REAR AXLE. The body rectangle is drawn at the
% body centre (previously it was drawn 1.3 m behind the real vehicle).
if isfield(opts, 'cfg') && ~isempty(opts.cfg)
    vpDraw = vehicleParams(opts.cfg);
else
    vpDraw = vehicleParams(irpscConfig());
end
[~, egoCentre] = egoFootprint(ego.pos, ego.heading, vpDraw);
drawRect(ax, egoCentre, vpDraw.length, vpDraw.width, ego.heading, [0.10 0.35 0.10], 0.85);

% --- Annotation ----------------------------------------------------------
if isfield(plan,'confidence')
    txt = sprintf('v = %.1f m/s   risk = %.2f   confidence = %.2f', ...
                  ego.speed, getOpt(plan,'risk',0), plan.confidence);
    title(ax, txt, 'FontSize', 9, 'FontWeight','normal');
end

xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
grid(ax,'on');

if followEgo
    W = 45;
    xlim(ax, ego.pos(1) + [-W/2, W]);
    ylim(ax, ego.pos(2) + [-W/2, W/2]);
end
hold(ax,'off');
end

% =====================================================================
function drawRect(ax, center, len, wid, yaw, col, alpha)
c = [ len/2  wid/2; len/2 -wid/2; -len/2 -wid/2; -len/2  wid/2];
R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
p = (R * c.').' + center;
patch(ax, 'XData', p(:,1), 'YData', p(:,2), 'FaceColor', col, ...
      'FaceAlpha', alpha, 'EdgeColor', col*0.6, 'LineWidth', 0.8);
end

% =====================================================================
function drawEllipse(ax, center, a, b, yaw, col, alpha)
th = linspace(0, 2*pi, 24);
e  = [a*cos(th); b*sin(th)];
R  = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
p  = (R * e).' + center;
patch(ax, 'XData', p(:,1), 'YData', p(:,2), 'FaceColor', col, ...
      'FaceAlpha', alpha, 'EdgeColor','none');
end

% =====================================================================
function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
