function [V, perf] = replayDemo(logOrFile, opts)
%REPLAYDEMO Presentation-quality playback of a recorded simulation log.
%
%   COMPONENT STATUS: REAL (visualisation of a recorded run)
%
%   REPLAYDEMO(log) replays a log returned by RUNDEMO or RUNSCENARIO.
%   REPLAYDEMO('results/run.mat') loads a .mat file containing `log`.
%   [V, perf] = REPLAYDEMO(..., opts) returns the viewer and the measured
%   display performance.
%
%   THIS IS THE DEMO MODE. There is no planner in the loop, so the display
%   rate is set by the renderer alone and a smooth 30 FPS playback is
%   achievable. Every frame still shows exactly what the planner,
%   perception and decision logic produced at that step of the recorded
%   run: the log stores the trajectory, corridor, risk grid, predictions,
%   tracks, detections and pothole tracks of every step. Replay is not a
%   re-simulation and computes no new plans.
%
%   Simulation states are recorded every cfg.sim.dt (0.05 s = 20 Hz). When
%   the display runs faster than that, the ego and the other road users are
%   interpolated between two recorded states (INTERPFRAME) so motion is
%   continuous. Interpolation moves the picture only: no logged value, no
%   decision and no trajectory is altered.
%
%   Options:
%       .fps          target display FPS (default vizConfig().displayFps = 30;
%                     15 / 20 / 30 / 45 / 60 are the supported targets)
%       .speed        playback speed relative to simulated time (default 1)
%       .pace         true (default): real-time playback paced to the wall
%                     clock; if drawing cannot keep up, display frames are
%                     DROPPED (and reported) rather than slowing time down.
%                     false: draw every frame as fast as possible.
%                     Recording a video always draws every frame.
%       .camera       'chase' | 'overview' | 'top'
%       .visualMode   'full' | 'planning' | 'sensing' | 'minimal'
%       .viz          a VIZCONFIG struct (overrides the two above)
%       .fromTime     s, start of replay (default 0)
%       .toTime       s, end of replay (default end of log)
%       .videoFile    record the replay at .fps, e.g. 'results/demo.mp4'. MATLAB
%                     uses VideoWriter; GNU Octave writes PNG frames and
%                     assembles them with its bundled ffmpeg (FRAMERECORDER).
%                     Every frame is drawn, so the video is smooth at .fps
%                     however slowly this computer draws.
%       .visible      false for off-screen rendering
%       .snapshotTimes  vector of times at which to save PNG snapshots
%       .snapshotDir    folder for snapshots (default 'results/snapshots')
%       .quiet        true to suppress the performance summary
%
%   Example:
%       [log, M] = runDemo(struct('visible', false));
%       save('results/demo_run.mat', 'log', 'M');
%       replayDemo('results/demo_run.mat')                       % 30 FPS
%       replayDemo('results/demo_run.mat', struct('fps', 60))
%       replayDemo('results/demo_run.mat', struct('camera', 'top'))
%
%   Requires: base MATLAB only.
%
%   See also RUNDEMO, VIZCONFIG, UPDATEDEMOVIEWER, INTERPFRAME.

if nargin < 2, opts = struct(); end
if ischar(logOrFile)
    S = load(logOrFile);
    log = S.log;
else
    log = logOrFile;
end

vc = getOpt(opts, 'viz', vizConfig(getOpt(opts, 'visualMode', 'full')));
if isfield(opts, 'camera') && ~isempty(opts.camera), vc.camera = opts.camera; end
if isfield(opts, 'fps') && ~isempty(opts.fps), vc.displayFps = opts.fps; end
fps       = max(1, vc.displayFps);
speed     = getOpt(opts, 'speed', 1);
pace      = getOpt(opts, 'pace', true);
quiet     = getOpt(opts, 'quiet', false);
snapT     = getOpt(opts, 'snapshotTimes', []);
snapDir   = getOpt(opts, 'snapshotDir', fullfile('results', 'snapshots'));
if ~isempty(snapT) && exist(snapDir, 'dir') ~= 7
    mkdir(snapDir);
end

scn = buildScenario(log.scenario, log.seed);
cfg = irpscConfig(log.profile);
V = createDemoViewer(scn, cfg, struct('viz', vc, ...
        'visible', getOpt(opts, 'visible', true), ...
        'title', sprintf('REPLAY -- %s -- %s', prettyPlanner(log.planner), scn.sihScenario)));

t0 = max(getOpt(opts, 'fromTime', 0), log.t(1));
t1 = min(getOpt(opts, 'toTime', log.t(end)), log.t(end));
videoFile = getOpt(opts, 'videoFile', '');
rec = frameRecorder('start', [], videoFile, fps);
if ~isempty(videoFile) && pace
    % A recording must contain EVERY display frame (one per 1/fps s of
    % simulated time), so it plays back smoothly at fps however long each
    % frame took to draw. Dropping frames to keep up would corrupt its timing.
    pace = false;
    fprintf('Recording: drawing every frame (not paced to the wall clock).\n');
end
if ~isempty(videoFile)
    % The on-screen performance read-out describes THIS computer while
    % exporting (often well under 1 FPS), not how the video plays. Burned
    % into a video it would mislead, so recordings never show it.
    V.vc.show.perfHud = false;
    set(V.h.perfText, 'Visible', 'off');
end
tShown = t0;

dtDisp    = speed / fps;                 % simulated seconds per display frame
nFrames   = max(1, floor((t1 - t0) / dtDisp) + 1);
snapDone  = false(size(snapT));
kLast     = -1;
fA = [];  fB = [];
frameMs = zeros(0,1);  renderMs = zeros(0,1);
nFull = 0;  nTween = 0;  nSkipped = 0;
wall = tic;
frameTimer = tic;

i = 0;
while true
    % -- which display frame is due? ----------------------------------
    % Paced playback keeps REAL TIME: if drawing cannot keep up with the
    % target rate, display frames are dropped (and counted), so simulated
    % time on screen still runs at wall-clock speed instead of slipping into
    % slow motion. Unpaced playback draws every display frame.
    if pace
        due = floor(toc(wall) * fps) + 1;
        if due > i + 1
            nSkipped = nSkipped + (due - i - 1);
        end
        i = max(i + 1, due);
    else
        i = i + 1;
    end
    if i > nFrames, break; end
    if ~ishandle(V.fig), break; end
    ctl = getappdata(V.fig, 'viewerCtl');
    while ctl.paused && ishandle(V.fig) && ~ctl.quit
        pause(0.05);
        ctl = getappdata(V.fig, 'viewerCtl');
    end
    if ~ishandle(V.fig) || ctl.quit, break; end

    tSim = t0 + (i-1) * dtDisp;
    if tSim > t1, break; end

    % -- locate the bracketing recorded states ------------------------
    k = find(log.t <= tSim + 1e-9, 1, 'last');
    if isempty(k), k = 1; end
    u = 0;
    if k < numel(log.t) && log.t(k+1) > log.t(k)
        u = (tSim - log.t(k)) / (log.t(k+1) - log.t(k));
    end
    isNewState = (k ~= kLast);
    if isNewState
        fA = frameFromLog(log, k, scn, cfg);
        if k < numel(log.t)
            fB = frameFromLog(log, k+1, scn, cfg);
        else
            fB = [];
        end
        kLast = k;
    end

    % -- snapshots ----------------------------------------------------
    isSnap = false;
    for q = 1:numel(snapT)
        if ~snapDone(q) && tSim >= snapT(q)
            isSnap = true;  snapDone(q) = true;
        end
    end

    % -- draw ---------------------------------------------------------
    rt = tic;
    frame = interpFrame(fA, fB, u);
    frame.tween = ~isNewState;      % a new state always gets a full redraw
    frame.perf  = struct('target', fps, 'fps', fpsOf(frameMs), ...
                         'frameMs', meanOr(frameMs), 'simMs', 0, ...
                         'renderMs', meanOr(renderMs));
    V = updateDemoViewer(V, frame);
    drawnowFast();
    r = toc(rt) * 1000;
    rec = frameRecorder('write', rec, V.fig);
    if isSnap
        ctlNow = getappdata(V.fig, 'viewerCtl');
        f = fullfile(snapDir, sprintf('%s_t%05.1f_%s.png', log.scenario, tSim, ctlNow.camera));
        print(V.fig, f, '-dpng', '-r90');
        fprintf('snapshot %s\n', f);
    end

    if frame.tween, nTween = nTween + 1; else, nFull = nFull + 1; end
    tShown = tSim;
    renderMs(end+1) = r; %#ok<AGROW>
    frameMs(end+1)  = toc(frameTimer) * 1000; %#ok<AGROW>
    frameTimer = tic;

    % -- pace to the wall clock ---------------------------------------
    if pace
        lag = (i / fps) - toc(wall);
        if lag > 0
            pause(lag);
        end
    end
end
wallTime = toc(wall);
frameRecorder('stop', rec);

perf = struct('targetFps', fps, ...
              'achievedFps', (nFull + nTween) / max(wallTime, eps), ...
              'renderOnlyFps', 1000 / max(mean(renderMs), eps), ...
              'meanFrameMs', meanOr(frameMs), 'maxFrameMs', maxOr(frameMs), ...
              'meanRenderMs', meanOr(renderMs), 'maxRenderMs', maxOr(renderMs), ...
              'framesFull', nFull, 'framesInterpolated', nTween, ...
              'framesDropped', nSkipped, 'wallTime', wallTime, ...
              'simulatedTime', tShown - t0, 'paced', pace);

if ~quiet
    fprintf('\n--- replay display performance (measured, this machine) ---------\n');
    fprintf('  target FPS          : %d\n', perf.targetFps);
    fprintf('  achieved FPS        : %.1f%s\n', perf.achievedFps, pacedNote(pace));
    fprintf('  render-only ceiling : %.1f FPS (mean render %.1f ms)\n', ...
            perf.renderOnlyFps, perf.meanRenderMs);
    fprintf('  mean frame time     : %.1f ms   max %.1f ms\n', perf.meanFrameMs, perf.maxFrameMs);
    fprintf('  frames drawn        : %d full + %d interpolated', nFull, nTween);
    if nSkipped > 0
        fprintf('\n  frames dropped      : %d (drawing could not keep up with %d FPS;', nSkipped, fps);
        fprintf('\n                        dropped to keep playback in real time)');
    end
    fprintf('\n');
    fprintf('  simulated time shown: %.1f s in %.1f s wall clock\n', ...
            perf.simulatedTime, perf.wallTime);
    fprintf('  Replay draws recorded state only; no planning happens here.\n');
    fprintf('-----------------------------------------------------------------\n');
end
end

% =========================================================================
function frame = frameFromLog(log, k, scn, cfg)
%FRAMEFROMLOG Rebuild the viewer frame for one recorded simulation step.
plan = struct();
plan.corridor      = log.corridorCache{k};
plan.preds         = log.preds{k};
plan.riskGrid      = log.riskGrid{k};
plan.preferredPath = log.preferredPath{k};
plan.potholes      = log.plannerPotholes{k};
plan.behaviour     = log.behaviour{k};
plan.risk          = log.risk(k);
plan.confidence    = log.confidence(k);
plan.ttc           = log.ttc(k);
plan.ttcPlanned    = log.ttcPlanned(k);
plan.status        = log.status{k};
plan.minClearance  = log.minClear(k);
plan.rejectedTraj  = getCell(log, 'rejectedTraj', k);

action = struct('state', log.state{k}, 'reason', log.decisionReason{k}, ...
                'emergencyBrake', log.emergencyBrake(k), ...
                'speedLimit', getNum(log, 'speedLimit', k, cfg.ego.maxSpeed), ...
                'stateChanged', getNum(log, 'stateChanged', k, false));

% MAKEEGOSTATE validates its inputs through inputParser, which costs a few
% milliseconds -- a real share of a 33 ms frame budget. Build one canonical
% state and then fill in the recorded numbers, so the struct is identical in
% shape to the simulation's own ego state without re-validating every frame.
persistent egoTemplate
if isempty(egoTemplate)
    egoTemplate = makeEgoState(log.egoPos(k,:), log.egoHeading(k), log.egoSpeed(k), ...
                               'Steer', log.egoSteer(k), 'Accel', log.egoAccel(k), ...
                               'Time', log.t(k));
end
ego = egoTemplate;
ego.pos     = log.egoPos(k,:);
ego.heading = log.egoHeading(k);
ego.speed   = log.egoSpeed(k);
ego.steer   = log.egoSteer(k);
ego.accel   = log.egoAccel(k);
ego.time    = log.t(k);

frame = struct('k', k, 't', log.t(k), 'scn', scn, 'cfg', cfg, 'ego', ego, ...
               'plan', plan, 'action', action, 'traj', log.traj{k}, ...
               'tracks', log.tracks{k}, 'truth', log.obstacles{k}, ...
               'detections', log.detections{k}, 'potholeTracks', log.potholeTracks{k}, ...
               'potholeDets', log.potholeDets{k}, 'steer', log.egoSteer(k), ...
               'accel', log.egoAccel(k), 'collided', log.collided(k), ...
               'replanned', log.replanned(k), 'plannerName', prettyPlanner(log.planner), 'log', log);
if isempty(frame.detections)
    frame.detections = struct('camera', zeros(0,2), 'lidar', zeros(0,2), 'radar', zeros(0,2));
end
end

% =========================================================================
function drawnowFast()
persistent hasLimitRate
if isempty(hasLimitRate)
    hasLimitRate = false;
    try
        drawnow('limitrate');
        hasLimitRate = true;
    catch
        drawnow;
    end
    return;
end
if hasLimitRate
    drawnow('limitrate');
else
    drawnow;
end
end

function s = prettyPlanner(name)
switch name
    case 'irpscPlanner',    s = 'IR-PSC';
    case 'baselinePlanner', s = 'Baseline (fixed candidates)';
    otherwise,              s = name;
end
end

function s = pacedNote(pace)
if pace
    s = '  (paced to the wall clock)';
else
    s = '  (unpaced: as fast as the renderer allows)';
end
end

function m = meanOr(w)
if isempty(w), m = 0; else, m = mean(w); end
end

function m = maxOr(w)
if isempty(w), m = 0; else, m = max(w); end
end

function f = fpsOf(frameMs)
m = meanOr(frameMs);
if m > 0, f = 1000 / m; else, f = 0; end
end

function v = getCell(log, name, k)
if isfield(log, name) && numel(log.(name)) >= k
    v = log.(name){k};
else
    v = [];
end
end

function v = getNum(log, name, k, defaultVal)
if isfield(log, name) && numel(log.(name)) >= k
    v = log.(name)(k);
else
    v = defaultVal;
end
end

function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
