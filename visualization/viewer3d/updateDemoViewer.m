function V = updateDemoViewer(V, frame)
%UPDATEDEMOVIEWER Draw one simulation frame in the 3D demonstration viewer.
%
%   COMPONENT STATUS: REAL (visualisation of simulation state; no simulation)
%
%   V = UPDATEDEMOVIEWER(V, frame) moves every dynamic graphics object to
%   the state carried by `frame` (see RUNSCENARIO, opts.onStep, or
%   REPLAYDEMO). It does not compute anything the simulation did not:
%
%     ego pose            <- frame.ego (bicycle model output)
%     road users (3D)     <- frame.truth (scenario ground truth)
%     wire boxes/labels   <- frame.tracks (multi-object tracker output)
%     sensor dots         <- frame.detections (simulated camera/LiDAR/radar)
%     cyan trajectory     <- frame.traj (what the controller is following:
%                            the IR-PSC plan, or the safe-stop trajectory)
%     preferred path      <- frame.plan.preferredPath (undeformed preference)
%     green corridor      <- frame.plan.corridor (drivable-space corridor)
%     risk cells          <- frame.plan.riskGrid.R (the DP's risk table)
%     pothole-cost cells  <- frame.plan.riskGrid.potholeCost
%     predicted ellipses  <- frame.plan.preds (predicted occupancy, 1/2/3 s)
%     pothole rings       <- frame.potholeTracks (pothole tracker output)
%     HUD and event log   <- frame.action (decision logic), frame.plan
%
%   PERFORMANCE DESIGN (see docs/PHASE2_CHANGES.md)
%   ----------------------------------------------
%   Profiling showed that 76% of render time was spent inside `set`, at
%   about 1 ms per property write in GNU Octave, with ~74 writes per frame.
%   This function therefore:
%     * writes a property only when its value actually CHANGED (V.cache),
%     * updates slow or expensive layers every Nth frame (vc.rate.*),
%     * accepts interpolated "tween" frames (frame.tween = true), where
%       only the ego, road users, trail and camera move,
%     * never creates or destroys graphics objects in steady state.
%
%   Optional fields on `frame`:
%       .tween       true  -> cheap update (ego, actors, camera, trail only)
%       .perf        struct with .target .fps .frameMs .simMs .renderMs
%       .plannerName name shown in the HUD
%
%   Requires: base MATLAB only.
%
%   See also CREATEDEMOVIEWER, VIZCONFIG, APPLYVISUALMODE, RUNDEMO, REPLAYDEMO.

if ~ishandle(V.fig)
    return;
end
V.frameCount  = V.frameCount + 1;
V.renderIndex = V.renderIndex + 1;
V.stats.writes = 0;              % property writes this frame (diagnostic)
vc   = V.vc;
pal  = V.pal;
vp   = V.vp;
ego  = frame.ego;
plan = getf(frame, 'plan', struct());
beh  = getf(plan, 'behaviour', struct());
isTween = isfield(frame, 'tween') && ~isempty(frame.tween) && frame.tween;
first   = (V.frameCount == 1);

% Visual mode may have been changed from the keyboard (f/p/s/m).
ctl = getappdata(V.fig, 'viewerCtl');
if isfield(ctl, 'visualMode') && ~strcmp(ctl.visualMode, vc.visualMode)
    V = applyVisualMode(V, ctl.visualMode);
    vc = V.vc;
    V.cache = struct();          % visibility changed: redraw every layer
end
show = vc.show;

% =====================================================================
% Ego vehicle  (every frame)
% =====================================================================
[~, egoC] = egoFootprint(ego.pos, ego.heading, vp);
V = setCached(V, 'egoV', V.h.ego, {'Vertices', place(V.egoModel.V, egoC, ego.heading, 0)});
ringT = linspace(0, 2*pi, 26);
V = setCached(V, 'egoRing', V.h.egoRing, ...
    {'XData', egoC(1) + 2.4*cos(ringT), 'YData', egoC(2) + 2.4*sin(ringT), ...
     'ZData', 0.04*ones(size(ringT))});

% =====================================================================
% Road users  (every frame: they move)
% =====================================================================
truth = frame.truth;
seen = false(size(V.actorIds));
for i = 1:numel(truth)
    o = truth(i);
    j = find(V.actorIds == o.id, 1);
    if isempty(j)
        m = actorModel3D(o.class, o.length, o.width, false, pal);
        h = patch('Parent', V.ax, 'Vertices', m.V, 'Faces', m.F, 'FaceVertexCData', m.C, ...
                  'FaceColor', 'flat', 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
        V.actorIds(end+1)    = o.id;
        V.actorH(end+1)      = h;
        V.actorModels{end+1} = m;
        seen(end+1) = false; %#ok<AGROW>
        j = numel(V.actorIds);
    end
    V = setCached(V, sprintf('actor%d', o.id), V.actorH(j), ...
        {'Vertices', place(V.actorModels{j}.V, o.pos, o.heading, 0), 'Visible', 'on'});
    seen(j) = true;
end
for j = find(~seen)
    V = setCached(V, sprintf('actorOff%d', V.actorIds(j)), V.actorH(j), {'Visible', 'off'});
end

% =====================================================================
% Ego trail  (every frame; cheap)
% =====================================================================
if show.trail
    V.trailPos = [V.trailPos; egoC, frame.t];
    keep = V.trailPos(:,3) >= frame.t - vc.trailSeconds;
    V.trailPos = V.trailPos(keep, :);
    if size(V.trailPos,1) > vc.trailMaxPoints         % thin out, keep the newest
        V.trailPos = V.trailPos([1:2:end-1, end], :);
    end
    if size(V.trailPos,1) >= 2
        V = setCached(V, 'trail', V.h.trail, ...
            {'XData', V.trailPos(:,1), 'YData', V.trailPos(:,2), ...
             'ZData', 0.05*ones(size(V.trailPos,1),1)});
    end
end

% =====================================================================
% Camera  (every frame; smoothed in SIMULATED time so the smoothing does
% not change when the display rate changes)
% =====================================================================
V = updateCamera(V, ctl, egoC, ego.heading, frame.t);

% A tween frame stops here: everything below belongs to a simulation state
% that has not changed since the last full frame.
if isTween
    V = updatePerf(V, frame);
    return;
end

% =====================================================================
% Drivable corridor
% =====================================================================
if show.corridor && due(V, vc.rate.corridor, first)
    corr = getf(plan, 'corridor', []);
    if isstruct(corr) && isfield(corr, 'valid') && isfield(corr, 'left') && size(corr.left,1) >= 2
        n = size(corr.left,1);
        if isfield(corr, 'blockedIdx') && ~isempty(corr.blockedIdx)
            n = max(2, corr.blockedIdx - 1);
        end
        L = corr.left(1:n,:);  R = corr.right(1:n,:);
        Vc = [L, 0.03*ones(n,1); R, 0.03*ones(n,1)];
        Fc = [(1:n-1).', (2:n).', n + (2:n).', n + (1:n-1).'];
        if corr.valid, cCol = pal.corridor; else, cCol = pal.corridorBad; end
        V = setCached(V, 'corr', V.h.corridor, {'Vertices', Vc, 'Faces', Fc, 'FaceColor', cCol});
        if show.corridorEdge
            V = setCached(V, 'corrEdge', V.h.corrEdges, ...
                {'XData', [L(:,1); NaN; R(:,1)], 'YData', [L(:,2); NaN; R(:,2)], ...
                 'ZData', [0.06*ones(n,1); NaN; 0.06*ones(n,1)]});
        end
    else
        V = setCached(V, 'corr', V.h.corridor, {'Vertices', nan(3,3), 'Faces', [1 2 3]});
        V = setCached(V, 'corrEdge', V.h.corrEdges, {'XData', NaN, 'YData', NaN, 'ZData', NaN});
    end
end

% =====================================================================
% Risk table and pothole cost (the DP's own inputs)
% =====================================================================
rg = getf(plan, 'riskGrid', []);
hasGrid = isstruct(rg) && isfield(rg, 'R') && ~isempty(rg.R) && size(rg.center,1) >= 2;
if show.risk && due(V, vc.rate.risk, first)
    if hasGrid
        [Vr, Fr, Cr] = gridCells(rg, rg.R, 0.12, 0.05, pal, vc.cellStride);
        if isempty(Fr)
            V = setCached(V, 'risk', V.h.riskCells, {'Vertices', nan(3,3), 'Faces', [1 2 3], 'FaceVertexCData', pal.riskHigh});
        else
            V = setCached(V, 'risk', V.h.riskCells, ...
                {'Vertices', Vr, 'Faces', Fr, 'FaceVertexCData', Cr});
        end
    else
        V = setCached(V, 'risk', V.h.riskCells, {'Vertices', nan(3,3), 'Faces', [1 2 3], 'FaceVertexCData', pal.riskHigh});
    end
end
if show.potholeCost && due(V, vc.rate.potholeCost, first)
    if hasGrid && isfield(rg, 'potholeCost') && ~isempty(rg.potholeCost) && any(rg.potholeCost(:) > 0)
        [Vp, Fp] = gridCells(rg, double(rg.potholeCost > 0), 0.5, 0.045, pal, vc.cellStride);
        V = setCached(V, 'potCost', V.h.potholeCells, {'Vertices', Vp, 'Faces', Fp});
    else
        V = setCached(V, 'potCost', V.h.potholeCells, {'Vertices', nan(3,3), 'Faces', [1 2 3]});
    end
end

% =====================================================================
% Predicted occupancy
% =====================================================================
preds = getf(plan, 'preds', []);
if show.prediction && due(V, vc.rate.prediction, first)
    horizons = [1 2 3];
    for hIdx = 1:3
        Ve = zeros(0,3);  Fe = zeros(0,20);
        for k = 1:numel(preds)
            p = preds(k);
            q = find(p.times(:) >= horizons(hIdx) - 1e-6, 1);
            if isempty(q), continue; end
            if isfield(p, 'halfLength') && ~isempty(p.halfLength)
                aL = p.halfLength;  aW = p.halfWidth;
            else
                aL = p.radius;  aW = p.radius;
            end
            % 1-sigma occupancy (the risk model itself uses
            % cfg.risk.nSigmaOccupancy). Very vague predictions are drawn
            % only at 1 s so they do not flood the view.
            a1 = aL + p.sigmaLong(q);
            b1 = aW + p.sigmaLat(q);
            if hIdx > 1 && max(a1, b1) > 5, continue; end
            tt = linspace(0, 2*pi, 21);  tt(end) = [];
            E  = [a1*cos(tt).', b1*sin(tt).', zeros(20,1)];
            n0 = size(Ve,1);
            Ve = [Ve; place(E, p.pos(q,:), p.heading(q), 0.08 + 0.01*hIdx)]; %#ok<AGROW>
            Fe = [Fe; n0 + (1:20)]; %#ok<AGROW>
        end
        key = sprintf('pred%d', hIdx);
        if isempty(Fe)
            V = setCached(V, key, V.h.predOcc(hIdx), {'Vertices', nan(3,3), 'Faces', [1 2 3]});
        else
            V = setCached(V, key, V.h.predOcc(hIdx), {'Vertices', Ve, 'Faces', Fe});
        end
    end
end
if show.predPaths && due(V, vc.rate.prediction, first)
    [PX, PY] = deal([]);
    for k = 1:numel(preds)
        PX = [PX; preds(k).pos(:,1); NaN]; %#ok<AGROW>
        PY = [PY; preds(k).pos(:,2); NaN]; %#ok<AGROW>
    end
    if isempty(PX), PX = NaN; PY = NaN; end
    V = setCached(V, 'predPaths', V.h.predPaths, ...
        {'XData', PX, 'YData', PY, 'ZData', 0.15*ones(size(PX))});
end

% =====================================================================
% Preferred path, rejected candidate, planned trajectory
% =====================================================================
if show.preferred
    pp = getf(plan, 'preferredPath', zeros(0,2));
    if size(pp,1) >= 2
        V = setCached(V, 'pref', V.h.prefPath, ...
            {'XData', pp(:,1), 'YData', pp(:,2), 'ZData', 0.09*ones(size(pp,1),1)});
    else
        V = setCached(V, 'pref', V.h.prefPath, {'XData', NaN, 'YData', NaN, 'ZData', NaN});
    end
end
if show.rejected
    rej = getf(plan, 'rejectedTraj', []);
    if isstruct(rej) && isfield(rej, 'pos') && size(rej.pos,1) >= 2
        V = setCached(V, 'rej', V.h.rejected, ...
            {'XData', rej.pos(:,1), 'YData', rej.pos(:,2), 'ZData', 0.14*ones(size(rej.pos,1),1)});
    else
        V = setCached(V, 'rej', V.h.rejected, {'XData', NaN, 'YData', NaN, 'ZData', NaN});
    end
end

isStop = strcmp(frame.action.state, 'SAFE_STOP');
if isStop
    tcol = pal.trajStop;
elseif flag(beh, 'yielding') || flag(beh, 'following') || flag(beh, 'blockedAhead')
    tcol = pal.trajCaution;
else
    tcol = pal.traj;
end
tr = frame.traj;
if show.trajectory && isstruct(tr) && isfield(tr, 'pos') && size(tr.pos,1) >= 2
    if isfield(tr, 'reachable') && numel(tr.reachable) == size(tr.pos,1)
        use = logical(tr.reachable(:));
    else
        use = true(size(tr.pos,1),1);
    end
    P = tr.pos(use,:);
    if size(P,1) >= 2
        th  = pathHeading(P);
        nrm = [-sin(th), cos(th)] * 0.42;
        m   = size(P,1);
        Vt = [P + nrm, 0.10*ones(m,1); P - nrm, 0.10*ones(m,1)];
        Ft = [(1:m-1).', (2:m).', m + (2:m).', m + (1:m-1).'];
        V = setCached(V, 'ribbon', V.h.trajRibbon, ...
            {'Vertices', Vt, 'Faces', Ft, 'FaceColor', tcol});
        V = setCached(V, 'trajLine', V.h.trajLine, ...
            {'XData', P(:,1), 'YData', P(:,2), 'ZData', 0.17*ones(m,1), 'Color', tcol});
        % Planned stop point: a wall across the corridor.
        vEnd = tr.speed(find(use, 1, 'last'));
        if vEnd <= 0.05
            pS = P(end,:);  hS = th(end);
            nS = [-sin(hS), cos(hS)] * 1.6;
            fS = [cos(hS), sin(hS)] * (vp.frontOverhang + 0.3);
            q1 = pS + fS - nS;  q2 = pS + fS + nS;
            V = setCached(V, 'stopWall', V.h.stopWall, ...
                {'Vertices', [q1 0; q2 0; q2 1.3; q1 1.3], 'Faces', [1 2 3 4]});
        else
            V = setCached(V, 'stopWall', V.h.stopWall, {'Vertices', nan(4,3)});
        end
    end
end

% =====================================================================
% Perception: tracker boxes, labels, raw detections
% =====================================================================
tracks = frame.tracks;
if show.tracks && due(V, vc.rate.tracks, first)
    [X, Y, Z] = deal([]);
    for i = 1:numel(tracks)
        o = tracks(i);
        C = boxCorners(o.pos, o.heading, o.length + 0.3, o.width + 0.3);
        for lev = [0.05 1.9]
            X = [X; C(:,1); C(1,1); NaN]; %#ok<AGROW>
            Y = [Y; C(:,2); C(1,2); NaN]; %#ok<AGROW>
            Z = [Z; repmat(lev, 5, 1); NaN]; %#ok<AGROW>
        end
        for c = 1:4
            X = [X; C(c,1); C(c,1); NaN]; %#ok<AGROW>
            Y = [Y; C(c,2); C(c,2); NaN]; %#ok<AGROW>
            Z = [Z; 0.05; 1.9; NaN]; %#ok<AGROW>
        end
    end
    if isempty(X), X = NaN; Y = NaN; Z = NaN; end
    V = setCached(V, 'tracks', V.h.tracks, {'XData', X, 'YData', Y, 'ZData', Z});

    nLbl = min(numel(tracks), 8);            % label only the nearest few
    if numel(tracks) > nLbl
        d = zeros(numel(tracks),1);
        for i = 1:numel(tracks)
            d(i) = hypot(tracks(i).pos(1) - egoC(1), tracks(i).pos(2) - egoC(2));
        end
        [~, ord] = sort(d);
        ord = ord(1:nLbl);
    else
        ord = 1:nLbl;
    end
    V.trackLbl = ensureTextPool(V, V.trackLbl, nLbl, pal.track, 7.5);
    for i = 1:nLbl
        o = tracks(ord(i));
        V = setCached(V, sprintf('tlbl%d', i), V.trackLbl(i), ...
            {'Position', [o.pos, 2.4], 'Visible', 'on', ...
             'String', sprintf('T%d %s %.1f m/s', o.id, o.class, o.speed)});
    end
    for i = nLbl+1:numel(V.trackLbl)
        V = setCached(V, sprintf('tlblOff%d', i), V.trackLbl(i), {'Visible', 'off'});
    end
end

if show.detections && due(V, vc.rate.detections, first)
    det = frame.detections;
    V = setPoints(V, 'detCam',   V.h.detCam,   det.camera, 0.9);
    V = setPoints(V, 'detLidar', V.h.detLidar, det.lidar,  0.6);
    V = setPoints(V, 'detRadar', V.h.detRadar, det.radar,  1.2);
end

% =====================================================================
% Potholes
% =====================================================================
pt = frame.potholeTracks;
if show.potholes && due(V, vc.rate.potholes, first)
    V = setPoints(V, 'potDets', V.h.potDets, frame.potholeDets, 0.25);
    [RX, RY, RZ, TX, TY, TZ] = deal([]);
    plannerPot = getf(plan, 'potholes', []);
    V.potLbl = ensureTextPool(V, V.potLbl, numel(pt), pal.potholeMod, 7.5);
    for i = 1:numel(pt)
        tt = linspace(0, 2*pi, 22).';
        E = [(pt(i).length/2 + 0.25) * cos(tt), (pt(i).width/2 + 0.25) * sin(tt), zeros(22,1)];
        W = place(E, pt(i).pos, pt(i).yaw, 0.06);
        if pt(i).confirmed
            RX = [RX; W(:,1); NaN]; RY = [RY; W(:,2); NaN]; RZ = [RZ; W(:,3); NaN]; %#ok<AGROW>
            action = '';
            if ~isempty(plannerPot)
                li = find([plannerPot.id] == pt(i).id, 1);
                if ~isempty(li) && ~strcmp(plannerPot(li).action, 'none')
                    action = [' -> ' upper(plannerPot(li).action)];
                end
            end
            V = setCached(V, sprintf('plbl%d', i), V.potLbl(i), ...
                {'Position', [pt(i).pos, 1.1], 'Visible', 'on', 'Color', sevColor(pal, pt(i).severity), ...
                 'String', sprintf('%s %.0f cm%s', upper(pt(i).severity), 100*pt(i).depth, action)});
        else
            TX = [TX; W(:,1); NaN]; TY = [TY; W(:,2); NaN]; TZ = [TZ; W(:,3); NaN]; %#ok<AGROW>
            V = setCached(V, sprintf('plbl%d', i), V.potLbl(i), ...
                {'Position', [pt(i).pos, 0.8], 'Visible', 'on', 'Color', pal.textMuted, ...
                 'String', sprintf('pothole? (%d hits)', pt(i).hits)});
        end
    end
    for i = numel(pt)+1:numel(V.potLbl)
        V = setCached(V, sprintf('plblOff%d', i), V.potLbl(i), {'Visible', 'off'});
    end
    if isempty(RX), RX = NaN; RY = NaN; RZ = NaN; end
    if isempty(TX), TX = NaN; TY = NaN; TZ = NaN; end
    V = setCached(V, 'potRings', V.h.potRings, {'XData', RX, 'YData', RY, 'ZData', RZ});
    V = setCached(V, 'potTent', V.h.potTentative, {'XData', TX, 'YData', TY, 'ZData', TZ});
end

% =====================================================================
% Banner
% =====================================================================
if frame.action.emergencyBrake
    V = setCached(V, 'banner', V.h.banner, ...
        {'String', 'EMERGENCY BRAKING', 'Visible', 'on', 'BackgroundColor', pal.stateStop});
elseif isStop
    V = setCached(V, 'banner', V.h.banner, ...
        {'String', 'SAFE STOP', 'Visible', 'on', 'BackgroundColor', pal.stateAvoid});
elseif frame.collided
    V = setCached(V, 'banner', V.h.banner, ...
        {'String', 'CONTACT RECORDED', 'Visible', 'on', 'BackgroundColor', [0.42 0.16 0.45]});
else
    V = setCached(V, 'banner', V.h.banner, {'Visible', 'off'});
end

% =====================================================================
% Mini-map and time series
% =====================================================================
lg = getf(frame, 'log', []);
k  = frame.k;
if show.minimap && due(V, vc.rate.minimap, first)
    V = setCached(V, 'mapEgo', V.h.mapEgo, {'XData', egoC(1), 'YData', egoC(2)});
    if isstruct(tr) && isfield(tr, 'pos')
        V = setCached(V, 'mapTraj', V.h.mapTraj, {'XData', tr.pos(:,1), 'YData', tr.pos(:,2)});
    end
    if isstruct(lg) && isfield(lg, 'egoPos')
        mt = 1:max(1, round(k/200)):k;      % at most ~200 points on the map
        V = setCached(V, 'mapTrail', V.h.mapTrail, ...
            {'XData', lg.egoPos(mt,1), 'YData', lg.egoPos(mt,2)});
    end
    if ~isempty(truth)
        tp = reshape([truth.pos], 2, []).';
        V = setCached(V, 'mapActors', V.h.mapActors, {'XData', tp(:,1), 'YData', tp(:,2)});
    else
        V = setCached(V, 'mapActors', V.h.mapActors, {'XData', NaN, 'YData', NaN});
    end
    w = V.mapHalfWindow;
    V = setCached(V, 'mapLim', V.axMap, ...
        {'XLim', egoC(1) + [-w/2, w*1.5], 'YLim', egoC(2) + [-w*0.55, w*0.55]});
end

if show.timeseries && due(V, vc.rate.timeseries, first) && isstruct(lg) && isfield(lg, 't')
    t0  = max(0, frame.t - 30);
    idx = find(lg.t(1:k) >= t0);
    tt  = lg.t(idx);
    V = setCached(V, 'tsSpeed', V.h.tsSpeed, ...
        {'XData', tt, 'YData', lg.egoSpeed(idx) / max(V.cfg.ego.maxSpeed, eps)});
    V = setCached(V, 'tsRisk', V.h.tsRisk, {'XData', tt, 'YData', lg.risk(idx)});
    V = setCached(V, 'tsConf', V.h.tsConf, {'XData', tt, 'YData', lg.confidence(idx)});
    stopMask = strcmp(lg.state(idx), 'SAFE_STOP');
    if any(stopMask)
        V = setCached(V, 'tsStop', V.h.tsStop, ...
            {'XData', tt(stopMask), 'YData', 0.02*ones(sum(stopMask),1)});
    else
        V = setCached(V, 'tsStop', V.h.tsStop, {'XData', NaN, 'YData', NaN});
    end
    V = setCached(V, 'tsLim', V.axTs, {'XLim', [t0, max(t0 + 30, frame.t + 0.1)]});
end

% =====================================================================
% Decision log and HUD
% =====================================================================
if show.events
    [newLines, V.evState] = viewerEvents(V.evState, frame, plan, beh);
    for i = 1:numel(newLines)
        V.eventLog{end+1} = newLines{i};
    end
end
if show.hud && due(V, vc.rate.hud, first)
    V = updateHud(V, frame, plan, beh, tcol);
end
V = updatePerf(V, frame);
end

% =========================================================================
function V = updateCamera(V, ctl, egoC, heading, tNow)
%UPDATECAMERA Follow the ego. Smoothing is a time constant in simulated
%seconds, so changing the display rate does not change how the camera moves.
fwd = [cos(heading), sin(heading)];
switch ctl.camera
    case 'overview'
        pos = [egoC - 36*fwd, 46];  tgt = [egoC + 22*fwd, 0];  up = [0 0 1];  va = 45;
    case 'top'
        pos = [egoC + 10*fwd, 90];  tgt = [egoC + 10*fwd, 0];  up = [fwd, 0];  va = 40;
    otherwise
        pos = [egoC - 14*fwd, 7.0]; tgt = [egoC + 17*fwd, 0.6]; up = [0 0 1];  va = 50;
end
if isempty(V.camPos) || ~strcmp(V.camMode, ctl.camera)
    V.camPos = pos;  V.camTgt = tgt;
else
    dt = max(0, min(0.5, tNow - V.camTime));
    alpha = 1 - exp(-dt / max(V.vc.cameraLag, 1e-3));
    V.camPos = V.camPos + alpha * (pos - V.camPos);
    V.camTgt = V.camTgt + alpha * (tgt - V.camTgt);
end
V.camMode = ctl.camera;
V.camTime = tNow;
V = setCached(V, 'cam', V.ax, {'CameraPosition', V.camPos, 'CameraTarget', V.camTgt, ...
                               'CameraUpVector', up, 'CameraViewAngle', va});
V = setCached(V, 'camTxt', V.h.camText, ...
    {'String', sprintf('%s  |  1 chase  2 overview  3 top  |  f p s m modes  |  space pause  q quit', ...
                       upper(ctl.camera))});
end

% =========================================================================
function V = updatePerf(V, frame)
%UPDATEPERF Optional on-screen performance read-out (vc.show.perfHud).
if ~V.vc.show.perfHud || ~isfield(frame, 'perf') || isempty(frame.perf)
    return;
end
p = frame.perf;
s = sprintf(['target %d FPS | actual %4.1f FPS | frame %5.1f ms | sim %5.1f ms | ' ...
             'render %5.1f ms | %d writes'], ...
            round(getf(p, 'target', 0)), getf(p, 'fps', 0), getf(p, 'frameMs', 0), ...
            getf(p, 'simMs', 0), getf(p, 'renderMs', 0), V.stats.writes);
V = setCached(V, 'perf', V.h.perfText, {'String', s});
end

% =========================================================================
function V = updateHud(V, frame, plan, beh, tcol)
%UPDATEHUD One row per thing a judge must read. Values are written only
%when they change, which is most of the saving on a text-heavy panel.
h   = V.hud;
pal = V.pal;
cfg = V.cfg;
act = frame.action;

V = setCached(V, 'hScen', h.scen, ...
    {'String', clip(sprintf('%s  |  seed %d  |  %s', getf(frame, 'plannerName', 'IR-PSC'), ...
                            V.scn.seed, V.scn.sihScenario), 60)});

[badgeCol, word, sub] = stateStyle(pal, act.state, act.emergencyBrake);
V = setCached(V, 'hBox',   h.stateBox, {'FaceColor', badgeCol});
V = setCached(V, 'hState', h.state,    {'String', word});
V = setCached(V, 'hSub',   h.stateSub, {'String', sprintf('%s  |  t = %.1f s', sub, frame.t)});

% -- risk -------------------------------------------------------------
risk = getf(plan, 'risk', 0);
if risk >= cfg.decision.riskAvoid
    rWord = 'HIGH';    rCol = pal.riskHigh;
elseif risk >= cfg.decision.riskHazard
    rWord = 'MEDIUM';  rCol = pal.riskMed;
else
    rWord = 'LOW';     rCol = pal.riskLow;
end
V = setCached(V, 'hRiskV', h.rowRisk.value, {'String', sprintf('%s  (%.2f)', rWord, risk)});
V = setCached(V, 'hRiskB', h.rowRisk.bar, ...
    {'Position', barPos(h.rowRisk.bar, min(risk,1)), 'FaceColor', rCol});

% -- confidence -------------------------------------------------------
conf = getf(plan, 'confidence', 1);
if conf >= cfg.decision.confHigh
    cWord = 'HIGH';    cCol = pal.corridorEdge;
elseif conf >= cfg.decision.confLow
    cWord = 'MEDIUM';  cCol = pal.riskMed;
else
    cWord = 'LOW';     cCol = pal.riskHigh;
end
V = setCached(V, 'hConfV', h.rowConf.value, {'String', sprintf('%s  (%.2f)', cWord, conf)});
V = setCached(V, 'hConfB', h.rowConf.bar, ...
    {'Position', barPos(h.rowConf.bar, min(conf,1)), 'FaceColor', cCol});

% -- numbers ----------------------------------------------------------
ttcP = getf(plan, 'ttcPlanned', Inf);
V = setCached(V, 'hTtc', h.rowTtc.value, {'String', fmtT(ttcP)});
V = setCached(V, 'hSpd', h.rowSpd.value, ...
    {'String', sprintf('%.1f km/h   (%.1f m/s)', 3.6*frame.ego.speed, frame.ego.speed)});
d = frame.detections;
V = setCached(V, 'hDet', h.rowDet.value, ...
    {'String', sprintf('%d tracked  |  %d raw', numel(frame.tracks), ...
                       size(d.camera,1) + size(d.lidar,1) + size(d.radar,1))});
pt = frame.potholeTracks;
nConf = 0;
for i = 1:numel(pt), nConf = nConf + double(pt(i).confirmed); end
V = setCached(V, 'hPot', h.rowPot.value, ...
    {'String', sprintf('%d confirmed  |  %d tentative', nConf, numel(pt) - nConf)});

[pWord, pCol] = plannerWord(pal, plan, beh, act, tcol);
V = setCached(V, 'hPlan', h.rowPlan.value, {'String', pWord, 'Color', pCol});
V = setCached(V, 'hReason', h.reason, {'String', clip(getf(beh, 'reason', ''), 58)});

% -- pothole rows -----------------------------------------------------
pp = getf(plan, 'potholes', []);
lines = {};
for i = 1:numel(pt)
    dist = hypot(pt(i).pos(1) - frame.ego.pos(1), pt(i).pos(2) - frame.ego.pos(2));
    if dist > 60, continue; end
    act2 = '';
    if ~isempty(pp)
        li = find([pp.id] == pt(i).id, 1);
        if ~isempty(li) && ~strcmp(pp(li).action, 'none'), act2 = upper(pp(li).action); end
    end
    lines{end+1} = sprintf('#%d %-8s %4.1f cm  %3.0f m  %s', ...
                           pt(i).id, upper(pt(i).severity), 100*pt(i).depth, dist, act2); %#ok<AGROW>
end
for i = 1:numel(h.pot)
    if i <= numel(lines)
        V = setCached(V, sprintf('hPotL%d', i), h.pot(i), {'String', lines{i}});
    elseif i == 1 && isempty(lines)
        V = setCached(V, 'hPotL1', h.pot(1), {'String', 'none within 60 m'});
    else
        V = setCached(V, sprintf('hPotL%d', i), h.pot(i), {'String', ''});
    end
end

% -- decision log -----------------------------------------------------
n = numel(V.eventLog);
for i = 1:numel(h.ev)
    j = n - numel(h.ev) + i;
    if j >= 1, s = V.eventLog{j}; else, s = ''; end
    V = setCached(V, sprintf('hEv%d', i), h.ev(i), {'String', s, 'Color', eventColor(V.pal, s)});
end
end

% =========================================================================
function [col, word, sub] = stateStyle(pal, state, emergency)
switch state
    case 'NORMAL_DRIVING',       col = pal.stateNormal;  word = 'CRUISE';  sub = 'normal driving';
    case 'HAZARD_ASSESSMENT',    col = pal.stateHazard;  word = 'SLOW';    sub = 'hazard assessment';
    case 'PREDICTIVE_AVOIDANCE', col = pal.stateAvoid;   word = 'AVOID';   sub = 'predictive avoidance';
    case 'CONSERVATIVE_DRIVING', col = pal.stateConserv; word = 'SLOW';    sub = 'conservative (low confidence)';
    case 'SAFE_STOP',            col = pal.stateStop;    word = 'STOP';    sub = 'safe stop';
    case 'RECOVERY',             col = pal.stateRecover; word = 'RESUME';  sub = 'recovering';
    otherwise,                   col = [0.35 0.38 0.42]; word = state;     sub = '';
end
if emergency
    col = pal.stateStop;  word = 'EMERGENCY STOP';  sub = 'emergency braking';
end
end

function [word, col] = plannerWord(pal, plan, beh, act, tcol) %#ok<INUSD>
if strcmp(act.state, 'SAFE_STOP')
    word = 'STOPPED';    col = pal.trajStop;
elseif flag(beh, 'blockedAhead') || flag(beh, 'yielding')
    word = 'WAITING';    col = pal.trajCaution;
elseif flag(beh, 'avoiding') || flag(beh, 'potholeAvoid')
    word = 'AVOIDING';   col = pal.traj;
elseif flag(beh, 'following')
    word = 'FOLLOWING';  col = pal.trajCaution;
elseif flag(beh, 'potholeSlow') || flag(beh, 'narrowPassage') || flag(beh, 'slowing')
    word = 'SLOWING';    col = pal.trajCaution;
elseif ~strcmp(getf(plan, 'status', 'ok'), 'ok')
    word = 'DEGRADED';   col = pal.trajStop;
else
    word = 'SAFE';       col = pal.corridorEdge;
end
end

function c = eventColor(pal, s)
if isempty(s)
    c = pal.text;
elseif ~isempty(strfind(s, '[DECISION]')) %#ok<STREMP>
    c = pal.accent;
elseif ~isempty(strfind(s, '[RISK]')) %#ok<STREMP>
    c = pal.riskHigh;
elseif ~isempty(strfind(s, '[PLAN]')) %#ok<STREMP>
    c = pal.corridorEdge;
else
    c = pal.text;
end
end

% =========================================================================
function V = setCached(V, key, h, args)
%SETCACHED Write graphics properties only when the value actually changed.
%In GNU Octave a property write costs one to several milliseconds depending
%on how much data it carries, while comparing the value costs microseconds,
%so this is the main performance mechanism.
%
%ISEQUALN, not ISEQUAL: cleared layers are written as NaN, and isequal(NaN,
%NaN) is false, which would make every cleared layer rewrite itself on every
%frame -- exactly the cost this cache exists to avoid.
if ~ishandle(h), return; end
if isfield(V.cache, key) && isequaln(V.cache.(key), args)
    return;
end
set(h, args{:});
V.cache.(key) = args;
V.stats.writes = V.stats.writes + 1;
end

function tf = due(V, rate, first)
%DUE Is this layer scheduled to update on this frame?
if first || rate <= 1
    tf = true;
else
    tf = mod(V.renderIndex, rate) == 0;
end
end

function V = setPoints(V, key, h, P, z)
if isempty(P)
    V = setCached(V, key, h, {'XData', NaN, 'YData', NaN, 'ZData', NaN});
else
    V = setCached(V, key, h, {'XData', P(:,1), 'YData', P(:,2), 'ZData', z*ones(size(P,1),1)});
end
end

function pos = barPos(hBar, frac)
pos = get(hBar, 'Position');
pos(3) = max(0.001, 0.535 * frac);
end

% =========================================================================
function [Vv, Ff, Cc] = gridCells(rg, M, thresh, z, pal, stride)
%GRIDCELLS Quads for cells of the station x offset table M above thresh,
%coloured low -> medium -> high on the semantic risk ramp.
%
%   `stride` merges neighbouring lateral offsets into one displayed cell.
%   The DP still uses every offset; this only makes the picture cheaper to
%   draw (and, at 0.15 m spacing, easier to read).
if nargin < 6, stride = 1; end
Vv = zeros(0,3);  Ff = zeros(0,4);  Cc = zeros(0,3);
if stride > 1
    M = blockMaxCols(M, stride);
end
[I, J] = find(isfinite(M) & M > thresh);
if isempty(I), return; end
s = rg.s(:);  off = rg.offsets(:).';
if stride > 1
    off = off(1:stride:end);
end
if numel(s) >= 2, ds = median(diff(s)); else, ds = 1; end
if numel(off) >= 2, dd = off(2) - off(1); else, dd = 0.25 * stride; end
sA = s(I) - ds/2;  sB = s(I) + ds/2;
oA = off(J).' - dd/2;  oB = off(J).' + dd/2;
n = numel(I);
sq = [sA; sB; sB; sA];
oq = [oA; oA; oB; oB];
P = frenetToCartesian(rg.center, max(min(sq, s(end)), s(1)), oq);
Vv = [P, z*ones(4*n,1)];
Ff = [(1:n).', n + (1:n).', 2*n + (1:n).', 3*n + (1:n).'];
val = min(max(M(sub2ind(size(M), I, J)), 0), 1);
Cc = riskRamp(pal, val);
end

function B = blockMaxCols(M, stride)
%BLOCKMAXCOLS Reduce the columns of M by taking the max over blocks of
%`stride` columns, so a merged display cell keeps the worst value in it.
n = size(M, 2);
nb = ceil(n / stride);
B = -inf(size(M,1), nb);
for b = 1:nb
    c0 = (b-1)*stride + 1;
    c1 = min(n, c0 + stride - 1);
    B(:,b) = max(M(:, c0:c1), [], 2);
end
end

function C = riskRamp(pal, val)
%RISKRAMP low (yellow-green) -> medium (orange) -> high (red).
val = val(:);
C = zeros(numel(val), 3);
lo = val < 0.5;
u = val(lo) / 0.5;
C(lo,:) = (1-u) * pal.riskLow + u * pal.riskMed;
hi = ~lo;
u = (val(hi) - 0.5) / 0.5;
C(hi,:) = (1-u) * pal.riskMed + u * pal.riskHigh;
end

function W = place(Vb, pos, yaw, dz)
c = cos(yaw);  s = sin(yaw);
W = [Vb(:,1)*c - Vb(:,2)*s + pos(1), Vb(:,1)*s + Vb(:,2)*c + pos(2), Vb(:,3) + dz];
end

function pool = ensureTextPool(V, pool, n, col, sz)
while numel(pool) < n
    pool(end+1) = text(0, 0, 0, '', 'Parent', V.ax, 'Color', col, 'FontSize', sz, ...
                       'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
                       'Interpreter', 'none', 'Visible', 'off'); %#ok<AGROW>
end
end

function c = sevColor(pal, sev)
switch sev
    case 'severe',   c = pal.potholeSevere;
    case 'moderate', c = pal.potholeMod;
    otherwise,       c = pal.potholeMinor;
end
end

function s = clip(s, n)
if numel(s) > n
    s = [s(1:n-3) '...'];
end
end

function s = fmtT(t)
if isfinite(t), s = sprintf('%.1f s', t); else, s = 'clear'; end
end

function tf = flag(s, name)
tf = isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && logical(s.(name));
end

function v = getf(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
