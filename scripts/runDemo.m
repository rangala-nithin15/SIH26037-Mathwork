function [log, M] = runDemo(opts)
%RUNDEMO Live closed-loop IR-PSC simulation drawn in the 3D viewer.
%
%   COMPONENT STATUS: REAL (entry point) over SIMPLIFIED sensor, vehicle and
%   scenario models.
%
%   THE ONE COMMAND TO RUN:
%
%       >> setupPaths
%       >> runDemo
%
%   runDemo runs the full closed loop (RUNSCENARIO) on the primary demo
%   scenario and draws every simulation step in the 3D viewer as it
%   happens. The vehicle's motion comes from the controller and the
%   kinematic bicycle model; the trajectory you see is the IR-PSC planner's
%   output at that step. Nothing is precomputed: the viewer is called from
%   inside the loop through RUNSCENARIO's opts.onStep hook.
%
%   HOW FAST IT RUNS (honest note)
%   ------------------------------
%   In a live run the PLANNER, not the renderer, sets the pace: one
%   simulation step (plan, perception, dynamics) costs far more than one
%   rendered frame in GNU Octave, so the live display advances at whatever
%   rate the simulation produces states -- a few frames per second, in slow
%   motion relative to simulated time. runDemo does not fake a higher rate.
%   What it does do is draw EVERY simulation state (not every second one)
%   and keep the cheap layers moving between full redraws, so motion is
%   continuous rather than jumpy. For a smooth 30 FPS presentation, record
%   a run and play it back with REPLAYDEMO, which has no planner in the
%   loop. The measured numbers for both are in docs/VISUALIZATION.md.
%
%   [log, M] = RUNDEMO(opts) returns the simulation log and metrics.
%
%   Options (all optional):
%       .scenario     'demo' (default) | 'village' | 'urban' | 'highway' |
%                     'market' | 'cattle'
%       .planner      'irpsc' (default) | 'baseline'
%       .seed         random seed (default 1)
%       .camera       'chase' (default) | 'overview' | 'top'
%       .visualMode   'full' (default) | 'planning' | 'sensing' | 'minimal'
%       .viz          a VIZCONFIG struct (overrides the two above)
%       .fullEvery    full redraw every N simulation steps (default 2; the
%                     steps in between move the ego, road users and camera)
%       .realTime     true to slow the display to simulated time (default false)
%       .videoFile    e.g. 'results/demo.mp4' to record (default: no video).
%                     One frame per simulation step, written at 1/dt = 20 FPS
%                     so the video runs in real time. VideoWriter in MATLAB;
%                     in GNU Octave PNG frames assembled with its bundled
%                     ffmpeg (see FRAMERECORDER). For the smoothest video,
%                     record with REPLAYDEMO at 30 FPS instead.
%       .maxTime      stop after this many simulated seconds
%       .usePerfectPerception  bypass simulated sensors (diagnostic)
%       .visible      false to render off-screen (default true)
%       .quiet        true to suppress the performance summary
%
%   Keyboard, once the figure has focus:
%       1 chase   2 overview   3 top-down
%       f full    p planning   s sensing   m minimal
%       space pause/resume     q stop
%
%   Examples:
%       runDemo                                            % the demo
%       runDemo(struct('camera', 'overview'))
%       runDemo(struct('visualMode', 'planning'))
%       runDemo(struct('videoFile', 'results/irpsc_demo.mp4'))
%       runDemo(struct('scenario', 'market', 'planner', 'baseline'))
%
%   Requires: base MATLAB only (VideoWriter is part of base MATLAB).
%
%   See also RUNSCENARIO, REPLAYDEMO, VIZCONFIG, CREATEDEMOVIEWER.

if nargin < 1, opts = struct(); end
scenario  = getOpt(opts, 'scenario', 'demo');
planner   = lower(getOpt(opts, 'planner', 'irpsc'));
seed      = getOpt(opts, 'seed', 1);
realTime  = getOpt(opts, 'realTime', false);
videoFile = getOpt(opts, 'videoFile', '');
quiet     = getOpt(opts, 'quiet', false);

vc = getOpt(opts, 'viz', vizConfig(getOpt(opts, 'visualMode', 'full')));
if isfield(opts, 'camera') && ~isempty(opts.camera), vc.camera = opts.camera; end
fullEvery = max(1, round(getOpt(opts, 'fullEvery', 2)));

switch planner
    case {'irpsc', 'ir-psc'}
        plannerFn = @irpscPlanner;  plannerName = 'IR-PSC';
    case {'baseline', 'fixed'}
        plannerFn = @baselinePlanner;  plannerName = 'Baseline (fixed candidates)';
    otherwise
        error('runDemo:unknownPlanner', 'Unknown planner "%s". Use irpsc or baseline.', planner);
end

scn = buildScenario(scenario, seed);
cfg = irpscConfig(scn.profile);

V = createDemoViewer(scn, cfg, struct('viz', vc, ...
        'visible', getOpt(opts, 'visible', true), ...
        'title', sprintf('%s -- %s', plannerName, scn.sihScenario)));
fig = V.fig;
if ~isempty(videoFile)
    % The performance read-out describes this computer while recording, not
    % how the video plays, so it is not burned into recordings.
    V.vc.show.perfHud = false;
    set(V.h.perfText, 'Visible', 'off');
end

% Viewer, recorder and pacing state live in the figure, so the per-step
% callback is an ordinary sub-function (portable; no nested-function state).
st = struct('V', V, 'rec', frameRecorder('start', [], videoFile, round(1 / cfg.sim.dt)), ...
            'fullEvery', fullEvery, 'realTime', realTime, 'wallStart', tic, ...
            'plannerName', plannerName, 'stepTimer', tic, 'simMs', 0, ...
            'frameMs', [], 'renderMs', [], 'nFull', 0, 'nTween', 0, ...
            'target', vc.displayFps);
setappdata(fig, 'demoState', st);

runOpts = struct('seed', seed, 'onStep', @(frame) demoStep(fig, frame), ...
                 'usePerfectPerception', getOpt(opts, 'usePerfectPerception', false));
if isfield(opts, 'maxTime'), runOpts.maxTime = opts.maxTime; end

fprintf('Running %s on "%s" (seed %d). Close the window or press q to stop.\n', ...
        plannerName, scn.sihScenario, seed);
wall = tic;
[log, M] = runScenario(scenario, plannerFn, cfg, runOpts);
wallTime = toc(wall);

if ishandle(fig)
    st = getappdata(fig, 'demoState');
    frameRecorder('stop', st.rec);
    if ~quiet, printPerf(st, log, wallTime); end
end
printSummary(log, M, plannerName, scn);
end

% =========================================================================
function keepGoing = demoStep(fig, frame)
%DEMOSTEP Called by runScenario after every simulation step.
%
%   Every state is drawn. Full redraws happen every st.fullEvery steps; the
%   steps in between move only the ego, the road users, the trail and the
%   camera ("tween" frames), which is far cheaper and keeps motion smooth.
keepGoing = true;
if ~ishandle(fig)
    keepGoing = false;              % window closed
    return;
end
ctl = getappdata(fig, 'viewerCtl');
while ctl.paused && ishandle(fig) && ~ctl.quit
    pause(0.05);
    ctl = getappdata(fig, 'viewerCtl');
end
if ~ishandle(fig) || ctl.quit
    keepGoing = false;
    return;
end
st = getappdata(fig, 'demoState');

% Time spent in the simulation since the previous frame was finished.
simMs = toc(st.stepTimer) * 1000;

frame.replanned   = frame.log.replanned(frame.k);
frame.plannerName = st.plannerName;
frame.tween       = (mod(frame.k, st.fullEvery) ~= 0) && frame.k ~= 1;
frame.perf        = struct('target', st.target, 'fps', fpsOf(st), ...
                           'frameMs', meanOr(st.frameMs, 0), 'simMs', simMs, ...
                           'renderMs', meanOr(st.renderMs, 0));

rt = tic;
st.V = updateDemoViewer(st.V, frame);
drawnowFast();
renderMs = toc(rt) * 1000;
st.rec = frameRecorder('write', st.rec, fig);

if frame.tween, st.nTween = st.nTween + 1; else, st.nFull = st.nFull + 1; end
st.renderMs = pushWindow(st.renderMs, renderMs, 30);
st.frameMs  = pushWindow(st.frameMs, simMs + renderMs, 30);
st.simMs    = simMs;

if st.realTime
    lag = frame.t - toc(st.wallStart);
    if lag > 0, pause(lag); end
end
st.stepTimer = tic;
setappdata(fig, 'demoState', st);
end

% =========================================================================
function drawnowFast()
%DRAWNOWFAST Use drawnow('limitrate') where it exists (MATLAB), plain
%drawnow otherwise (GNU Octave does not implement limitrate).
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

function w = pushWindow(w, v, n)
w(end+1) = v;
if numel(w) > n, w = w(end-n+1:end); end
end

function m = meanOr(w, defaultVal)
if isempty(w), m = defaultVal; else, m = mean(w); end
end

function f = fpsOf(st)
m = meanOr(st.frameMs, 0);
if m > 0, f = 1000 / m; else, f = 0; end
end

% =========================================================================
function printPerf(st, log, wallTime)
n = st.nFull + st.nTween;
if n == 0, return; end
fprintf('\n--- live display performance (measured, this machine) ------------\n');
fprintf('  frames drawn        : %d full + %d interpolated\n', st.nFull, st.nTween);
fprintf('  mean render time    : %.1f ms per frame\n', meanOr(st.renderMs, 0));
fprintf('  mean frame time     : %.1f ms (simulation + render)\n', meanOr(st.frameMs, 0));
fprintf('  achieved display    : %.1f FPS\n', fpsOf(st));
fprintf('  wall clock          : %.1f s for %.1f s simulated (%.1fx real time)\n', ...
        wallTime, log.t(end), log.t(end) / max(wallTime, eps));
fprintf('  NOTE: the planner, not the renderer, sets this rate. Use\n');
fprintf('        replayDemo for a smooth presentation-quality playback.\n');
end

function printSummary(log, M, plannerName, scn)
fprintf('\n=====================================================================\n');
fprintf('  %s on %s\n', plannerName, scn.sihScenario);
fprintf('=====================================================================\n');
fprintf('  goal reached        : %d\n', log.goalReached);
fprintf('  simulated time      : %.1f s\n', log.t(end));
fprintf('  collisions          : %d (static contacts %d steps, dynamic %d steps)\n', ...
        M.collisionCount, sum(log.collidedStatic), sum(log.collidedDynamic));
fprintf('  worst clearance     : %.2f m\n', M.minClearance);
fprintf('  average speed       : %.2f m/s\n', M.averageSpeed);
fprintf('  safe-stop episodes  : %d\n', M.emergencyStops);
fprintf('  emergency braking   : %d steps\n', sum(log.emergencyBrake));
fprintf('  pothole wheel entries: %d\n', numel(log.potholeEvents));
for e = 1:numel(log.potholeEvents)
    ev = log.potholeEvents(e);
    fprintf('      t=%5.1f s pothole %d (%s) at %.1f m/s\n', ev.t, ev.id, ev.severity, ev.speed);
end
fprintf('---------------------------------------------------------------------\n');
fprintf('  Simulation result under SIMPLIFIED sensor, vehicle and scenario\n');
fprintf('  models. Not real-world safety evidence. Not safety certified.\n');
fprintf('=====================================================================\n\n');
end

function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
