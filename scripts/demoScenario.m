function log = demoScenario(scenarioName, plannerName, opts)
%DEMOSCENARIO Run one scenario with live animation, for demonstrations.
%
%   COMPONENT STATUS: REAL
%
%   log = DEMOSCENARIO(scenarioName, plannerName, opts) runs a scenario and
%   animates it as it goes. This is the function to use in front of judges.
%
%   WHAT THE ANIMATION IS BUILT TO SHOW
%   -----------------------------------
%   The visualisation is arranged so the IR-PSC argument is visible rather
%   than merely asserted:
%
%     - Drivable space is drawn as free area with NO lane markings, because
%       none exist in these scenes. Anyone watching can see that the planner
%       has no lane to follow.
%     - The extracted corridor boundaries appear as dashed lines: road edges
%       inferred purely from free space.
%     - The corridor centreline (dotted) and the final trajectory (solid)
%       are drawn separately. The GAP between them is the deformation --
%       the contribution, made visible.
%     - Predicted occupancy is drawn as ellipses that grow with time and
%       with uncertainty, so "the system is less sure about the motorcycle
%       than about the bus" is something you can see.
%     - The title shows speed, risk and confidence live, so a drop in
%       confidence and the resulting slowdown can be pointed at as they
%       happen.
%
%   Inputs:
%       scenarioName - char: 'village' | 'urban' | 'highway' | 'market' | 'cattle'
%       plannerName  - char: 'irpsc' (default) or 'baseline'
%       opts         - (optional) struct:
%                      .seed      integer seed (default 1)
%                      .pauseTime seconds between frames (default 0.01)
%                      .frameStep draw every Nth step (default 2)
%                      .maxTime   override the scenario duration
%                      .record    logical, capture frames for a movie
%
%   Outputs:
%       log - the simulation log, identical to runScenario()
%
%   Example:
%       setupPaths;
%       demoScenario('village', 'irpsc');
%
%   Side-by-side comparison for a demo:
%       demoScenario('market', 'baseline');   % watch it struggle
%       demoScenario('market', 'irpsc');      % same seed, same world
%
%   Requires: base MATLAB only.
%
%   See also RUNSCENARIO, PLOTSCENE, RUNCOMPARISON.

if nargin < 1 || isempty(scenarioName), scenarioName = 'village'; end
if nargin < 2 || isempty(plannerName),  plannerName  = 'irpsc';   end
if nargin < 3, opts = struct(); end

seed      = getOpt(opts, 'seed', 1);
pauseTime = getOpt(opts, 'pauseTime', 0.01);
frameStep = getOpt(opts, 'frameStep', 2);
record    = getOpt(opts, 'record', false);

switch lower(plannerName)
    case {'irpsc','ir-psc','irpscplanner'}
        plannerFn = @irpscPlanner;
        titleName = 'IR-PSC';
    case {'baseline','baselineplanner','fixed'}
        plannerFn = @baselinePlanner;
        titleName = 'Baseline (fixed candidates)';
    otherwise
        error('demoScenario:unknownPlanner', ...
              'Unknown planner "%s". Use irpsc or baseline.', plannerName);
end

% --- Run the simulation first, then replay it ---------------------------
% Running first and animating afterwards keeps the animation smooth and,
% more importantly, keeps the timing measurements in the log clean. Drawing
% inside the control loop would contaminate the replan-latency metric with
% rendering time.
runOpts = opts;
runOpts.seed = seed;

fprintf('Running %s on scenario "%s" (seed %d)...\n', titleName, scenarioName, seed);
[log, M] = runScenario(scenarioName, plannerFn, [], runOpts);
fprintf('Done: %d steps, %.1f s simulated.\n\n', numel(log.t), log.t(end));

scn = buildScenario(scenarioName, seed);

% --- Replay --------------------------------------------------------------
fig = figure('Name', sprintf('%s -- %s', titleName, scn.sihScenario), ...
             'Color', 'w', 'Position', [80 80 1100 620]);
ax = axes(fig); %#ok<LAXES>

frames = [];

for k = 1:frameStep:numel(log.t)
    if ~ishandle(fig)
        break;                       % user closed the window
    end

    ego = makeEgoState(log.egoPos(k,:), log.egoHeading(k), log.egoSpeed(k));

    plan.traj       = log.traj{k};
    plan.preds      = log.preds{k};
    plan.confidence = log.confidence(k);
    plan.risk       = log.risk(k);
    plan.isSafeStop = strcmp(log.state{k}, 'SAFE_STOP');
    if isfield(log,'corridorCache')
        plan.corridor = log.corridorCache{k};
    else
        plan.corridor = struct('valid', false);
    end

    plotScene(scn, ego, plan, log.obstacles{k}, ...
              struct('axesHandle', ax, 'followEgo', true));

    title(ax, sprintf('%s  |  %s  |  t=%.1fs  v=%.1f m/s  risk=%.2f  conf=%.2f  [%s]', ...
          titleName, scn.sihScenario, log.t(k), log.egoSpeed(k), ...
          log.risk(k), log.confidence(k), log.state{k}), ...
          'FontSize', 9, 'FontWeight', 'normal', 'Interpreter', 'none');

    drawnow;
    if record
        frames = [frames, getframe(fig)]; %#ok<AGROW>
    end
    if pauseTime > 0
        pause(pauseTime);
    end
end

% --- Print the honest summary -------------------------------------------
fprintf('=====================================================================\n');
fprintf('  %s on %s\n', titleName, scn.sihScenario);
fprintf('=====================================================================\n');
fprintf('  goal reached      : %d\n',      log.goalReached);
fprintf('  collisions        : %d\n',      M.collisionCount);
fprintf('  worst clearance   : %.3f m\n',  M.minClearance);
fprintf('  average speed     : %.2f m/s\n',M.averageSpeed);
fprintf('  emergency stops   : %d\n',      M.emergencyStops);
fprintf('  planner failures  : %d\n',      M.plannerFailures);
fprintf('  mean replan time  : %.1f ms\n', M.meanLatency * 1000);
fprintf('---------------------------------------------------------------------\n');
fprintf('  Simulation result under a SIMPLIFIED sensor model and a\n');
fprintf('  SIMPLIFIED kinematic vehicle model. Not real-world safety\n');
fprintf('  evidence. This system is not safety certified.\n');
fprintf('=====================================================================\n\n');

if record && ~isempty(frames)
    log.frames = frames;
    fprintf('Captured %d frames in log.frames (use VideoWriter to save).\n', ...
            numel(frames));
end
end

% =====================================================================
function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
