function [log, M] = runScenario(scenarioName, plannerFn, cfg, opts)
%RUNSCENARIO Closed-loop simulation of one scenario with one planner.
%
%   COMPONENT STATUS: REAL (integration) over SIMPLIFIED sensor and vehicle
%   models. See docs/COMPONENT_REGISTER.md.
%
%   [log, M] = RUNSCENARIO(scenarioName, plannerFn, cfg, opts) runs the full
%   closed loop and returns the log and its metrics.
%
%   ============================================================
%   THE CLOSED LOOP (executed every simulation step)
%   ============================================================
%     scenario actors move                     stepScenario       (SIMPLIFIED)
%         -> camera / LiDAR / radar simulation simulateDetections (SIMPLIFIED)
%         -> multi-sensor fusion               fuseDetections     (SIMPLIFIED)
%         -> multi-object tracking             multiObjectTracker (FALLBACK)
%         -> pothole detection + mapping       detectPotholes,
%                                              potholeTracker     (SIMPLIFIED)
%         -> prediction + planner (IR-PSC or baseline, at cfg.sim.planEvery)
%         -> decision logic                    decisionLogic      (REAL)
%         -> controller                        purePursuitControl,
%                                              longitudinalControl (FALLBACK)
%         -> vehicle dynamics                  bicycleModelStep   (SIMPLIFIED)
%         -> collision referee (ground truth only, never fed to the planner)
%         -> opts.onStep(frame)   e.g. the live 3D viewer
%
%   The loop is genuinely closed: the ego state produced by the dynamics
%   model determines what the sensors see on the next step. Nothing is
%   fed back from ground truth to the planner.
%
%   ============================================================
%   REPRODUCIBILITY
%   ============================================================
%   All randomness comes from one seeded RandStream created here. Two runs
%   with the same seed, scenario and configuration produce identical logs.
%   This is what makes the baseline comparison and the ablation study
%   valid -- both planners face exactly the same noise, the same missed
%   detections and the same false positives. (Actors triggered by ego
%   position, and follower actors, react to where the ego is, so the two
%   planners can meet an event at different times; that is intended.)
%
%   ============================================================
%   COLLISION REFEREE (Phase 2 fix)
%   ============================================================
%   The referee uses exact body rectangles against road users AND the
%   occupancy grid. The original referee ignored the grid entirely, so the
%   vehicle could drive through a parked truck, a stall or a wall without a
%   collision being recorded. Static and dynamic contacts are logged
%   separately and both count as collisions.
%
%   Pothole entries are logged as events (which pothole, severity, speed)
%   whenever a tyre enters a TRUE pothole. They are not collisions.
%
%   ============================================================
%   WHAT THE RESULTS MEAN
%   ============================================================
%   Simulation results only. The sensor model is geometric, not a real
%   detector. The vehicle model is kinematic, with no tyre or suspension
%   dynamics. Results are valid for comparing planning approaches under
%   identical conditions. They are NOT real-world safety evidence, and no
%   collision count from this function may be presented as such.
%
%   Inputs:
%       scenarioName - char, one of the names accepted by buildScenario()
%       plannerFn    - function handle, @irpscPlanner or @baselinePlanner.
%                      Called as plannerFn(grid, ego, obstacles, cfg, state,
%                      potholeTracks).
%       cfg          - (optional) config struct. Default: the profile the
%                      scenario declares.
%       opts         - (optional) struct with fields:
%                      .seed        integer random seed (default 0)
%                      .verbose     logical, print progress (default false)
%                      .usePerfectPerception  logical (default false). When
%                                   true the sensor and tracking chains are
%                                   bypassed and ground truth goes straight
%                                   to the planner.
%                      .maxTime     override the scenario time limit
%                      .onStep      function handle called after every step
%                                   with a frame struct (see below). If it
%                                   returns false the run stops early.
%
%   The frame passed to opts.onStep has fields:
%       .k .t .scn .cfg .ego .plan .action .traj .tracks .truth
%       .detections (struct .camera .lidar .radar, Kx2 positions)
%       .potholeTracks .potholeDets .steer .accel .collided
%       .log (the preallocated log; only entries 1..k are filled in)
%
%   Outputs:
%       log - simulation log struct, as documented in computeMetrics()
%       M   - metric struct from computeMetrics()
%
%   Example:
%       [log, M] = runScenario('demo', @irpscPlanner, [], struct('seed', 1));
%       fprintf('goal %d, collisions %d\n', log.goalReached, M.collisionCount);
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, IRPSCPLANNER, BASELINEPLANNER, COMPUTEMETRICS,
%            RUNDEMO.

if nargin < 3, cfg  = []; end
if nargin < 4, opts = struct(); end

seed       = getOpt(opts, 'seed', 0);
verbose    = getOpt(opts, 'verbose', false);
perfectPer = getOpt(opts, 'usePerfectPerception', false);
onStep     = getOpt(opts, 'onStep', []);

scn = buildScenario(scenarioName, seed);
if isempty(cfg)
    cfg = irpscConfig(scn.profile);
end
maxTime = getOpt(opts, 'maxTime', min(scn.duration, cfg.sim.maxTime));

vp = vehicleParams(cfg);
sc = sensorConfig();

stream = RandStream('mt19937ar', 'Seed', seed);

ego = makeEgoState(scn.egoStart.pos, scn.egoStart.heading, scn.egoStart.speed);

actors        = scn.actors;
plannerState  = [];
trackerState  = [];
potholeState  = [];
decisionState = [];
controlState  = [];

dt     = cfg.sim.dt;
nSteps = max(1, floor(maxTime / dt));

% --- Log -------------------------------------------------------------------
log.t              = zeros(nSteps,1);
log.egoPos         = zeros(nSteps,2);
log.egoSpeed       = zeros(nSteps,1);
log.egoHeading     = zeros(nSteps,1);
log.egoAccel       = zeros(nSteps,1);
log.egoSteer       = zeros(nSteps,1);
log.roadS          = zeros(nSteps,1);
log.minClear       = inf(nSteps,1);
log.clearStatic    = inf(nSteps,1);
log.clearDynamic   = inf(nSteps,1);
log.planTime       = nan(nSteps,1);
log.replanned      = false(nSteps,1);
log.planStamp      = zeros(nSteps,1);
log.risk           = zeros(nSteps,1);
log.confidence     = ones(nSteps,1);
log.ttc            = inf(nSteps,1);
log.ttcPlanned     = inf(nSteps,1);
log.state          = cell(nSteps,1);
log.decisionReason = cell(nSteps,1);
log.behaviour      = cell(nSteps,1);
log.status         = cell(nSteps,1);
log.emergencyBrake = false(nSteps,1);
log.speedLimit     = zeros(nSteps,1);
log.stateChanged   = false(nSteps,1);
log.rejectedTraj   = cell(nSteps,1);
log.collided       = false(nSteps,1);
log.collidedStatic = false(nSteps,1);
log.collidedDynamic= false(nSteps,1);
log.traj           = cell(nSteps,1);
log.obstacles      = cell(nSteps,1);
log.preds          = cell(nSteps,1);
log.tracks         = cell(nSteps,1);
log.nDetections    = zeros(nSteps,1);
log.detections     = cell(nSteps,1);
log.corridorCache  = cell(nSteps,1);
log.preferredPath  = cell(nSteps,1);
log.riskGrid       = cell(nSteps,1);
log.plannerPotholes= cell(nSteps,1);
log.potholeTracks  = cell(nSteps,1);
log.potholeDets    = cell(nSteps,1);
log.potholeEvents  = struct('t', {}, 'id', {}, 'severity', {}, 'speed', {}, 'depth', {});

lastPlan = [];
lastTraj = [];
goalReached = false;
kEnd = nSteps;
insidePothole = false(1, numel(scn.potholes));

for k = 1:nSteps
    t = (k-1) * dt;

    % --- Scenario actors ---------------------------------------------------
    [roadS, ~] = projectPointOnPath(scn.centerline, ego.pos);
    egoInfo = struct('pos', ego.pos, 'heading', ego.heading, 'speed', ego.speed, ...
                     'roadS', roadS, 'length', vp.length, 'width', vp.width, ...
                     'centerOffset', vp.length/2 - vp.rearOverhang);
    [actors, groundTruth] = stepScenario(actors, t, dt, egoInfo);

    % --- Perception: road users -------------------------------------------
    dets = struct('camera', zeros(0,2), 'lidar', zeros(0,2), 'radar', zeros(0,2));
    if perfectPer
        obstacles = groundTruth;
        nDet = numel(groundTruth);
    else
        camDets   = simulateDetections(groundTruth, ego, sc.camera, stream);
        lidarDets = simulateDetections(groundTruth, ego, sc.lidar,  stream);
        radarDets = simulateDetections(groundTruth, ego, sc.radar,  stream);
        nDet = numel(camDets) + numel(lidarDets) + numel(radarDets);
        dets.camera = detPositions(camDets);
        dets.lidar  = detPositions(lidarDets);
        dets.radar  = detPositions(radarDets);

        fused = fuseDetections({camDets, lidarDets, radarDets}, cfg);
        [obstacles, trackerState] = multiObjectTracker(fused, dt, cfg, trackerState);
    end

    % --- Perception: potholes ----------------------------------------------
    if perfectPer
        potDets = struct('pos',{},'length',{},'width',{},'yaw',{},'depth',{}, ...
                         'depthStd',{},'posStd',{},'confidence',{},'sensor',{},'truthId',{});
        potholeTracks = perfectPotholeTracks(scn.potholes, cfg);
    else
        potDets = detectPotholes(scn.potholes, ego, groundTruth, cfg, stream);
        [potholeTracks, potholeState] = potholeTracker(potDets, t, cfg, potholeState);
    end

    % --- Plan ------------------------------------------------------------
    if mod(k-1, cfg.sim.planEvery) == 0 || isempty(lastPlan)
        tPlan = tic;
        plan  = plannerFn(scn.grid, ego, obstacles, cfg, plannerState, potholeTracks);
        log.planTime(k) = toc(tPlan);
        log.replanned(k) = true;
        plannerState = plan.state;
        lastPlan  = plan;
        lastTraj  = plan.traj;
        planStamp = t;
    else
        plan = lastPlan;
    end

    % --- Decide ---------------------------------------------------------
    [action, decisionState] = decisionLogic(plan, cfg, decisionState);

    if action.useSafeStop
        refPath = [];
        if ~isempty(lastTraj) && isfield(lastTraj, 'pos')
            refPath = lastTraj.pos;
        end
        traj = safeStopTrajectory(ego, plan.corridor, cfg, action.emergencyBrake, refPath);
    else
        traj = lastTraj;
    end

    % --- Control ----------------------------------------------------------
    if isempty(traj) || ~isfield(traj,'pos') || size(traj.pos,1) < 2
        steerCmd = 0;
        accelCmd = -cfg.ego.emergencyDecel;
    else
        steerCmd = purePursuitControl(traj, ego, cfg, vp);
        [accelCmd, controlState] = longitudinalControl(traj, ego, ...
                                        action.speedLimit, cfg, controlState, ...
                                        action.emergencyBrake);
    end

    % --- Vehicle dynamics -----------------------------------------------
    ego = bicycleModelStep(ego, steerCmd, accelCmd, cfg);

    % --- Referee: ground truth only --------------------------------------
    [colDyn, colStat, clrDyn, clrStat] = refereeCollision(ego, groundTruth, scn.grid, vp);
    [events, insidePothole] = potholeEntries(ego, scn.potholes, insidePothole, vp, cfg, t);
    if ~isempty(events)
        log.potholeEvents = [log.potholeEvents, events];
    end

    % --- Record ---------------------------------------------------------
    log.t(k)              = t;
    log.egoPos(k,:)       = ego.pos;
    log.egoSpeed(k)       = ego.speed;
    log.egoHeading(k)     = ego.heading;
    log.egoAccel(k)       = ego.accel;
    log.egoSteer(k)       = ego.steer;
    log.roadS(k)          = roadS;
    log.clearDynamic(k)   = clrDyn;
    log.clearStatic(k)    = clrStat;
    log.minClear(k)       = min(clrDyn, clrStat);
    log.planStamp(k)      = planStamp;
    log.risk(k)           = getFieldOr(plan, 'risk', 0);
    log.confidence(k)     = getFieldOr(plan, 'confidence', 1);
    log.ttc(k)            = getFieldOr(plan, 'ttc', Inf);
    log.ttcPlanned(k)     = getFieldOr(plan, 'ttcPlanned', Inf);
    log.state{k}          = action.state;
    log.decisionReason{k} = action.reason;
    log.behaviour{k}      = getFieldOr(plan, 'behaviour', struct());
    log.status{k}         = getFieldOr(plan, 'status', 'ok');
    log.emergencyBrake(k) = action.emergencyBrake;
    log.speedLimit(k)     = action.speedLimit;
    log.stateChanged(k)   = action.stateChanged;
    log.rejectedTraj{k}   = getFieldOr(plan, 'rejectedTraj', []);
    log.collidedDynamic(k)= colDyn;
    log.collidedStatic(k) = colStat;
    log.collided(k)       = colDyn || colStat;
    log.traj{k}           = traj;
    log.obstacles{k}      = groundTruth;
    log.preds{k}          = getFieldOr(plan, 'preds', []);
    log.tracks{k}         = obstacles;
    log.nDetections(k)    = nDet;
    log.detections{k}     = dets;
    log.corridorCache{k}  = getFieldOr(plan, 'corridor', struct('valid', false));
    log.preferredPath{k}  = getFieldOr(plan, 'preferredPath', zeros(0,2));
    log.riskGrid{k}       = getFieldOr(plan, 'riskGrid', []);
    log.plannerPotholes{k}= getFieldOr(plan, 'potholes', []);
    log.potholeTracks{k}  = potholeTracks;
    log.potholeDets{k}    = potholeDetPositions(potDets);

    if verbose && mod(k, 20) == 0
        fprintf('t=%5.2f  v=%5.2f  state=%-20s risk=%.2f conf=%.2f  %s\n', ...
                t, ego.speed, action.state, log.risk(k), log.confidence(k), ...
                getFieldOr(log.behaviour{k}, 'reason', ''));
    end

    if ~isempty(onStep)
        frame = struct('k', k, 't', t, 'scn', scn, 'cfg', cfg, 'ego', ego, ...
                       'plan', plan, 'action', action, 'traj', traj, ...
                       'tracks', obstacles, 'truth', groundTruth, ...
                       'detections', dets, 'potholeTracks', potholeTracks, ...
                       'potholeDets', log.potholeDets{k}, 'steer', steerCmd, ...
                       'accel', accelCmd, 'collided', log.collided(k), ...
                       'replanned', log.replanned(k), ...
                       'log', []);
        frame.log = log;          % full preallocated log; entries 1..k are valid
        keepGoing = onStep(frame);
        if ~isempty(keepGoing) && ~keepGoing
            kEnd = k;
            break;
        end
    end

    if hypot(ego.pos(1)-scn.goal(1), ego.pos(2)-scn.goal(2)) <= scn.goalRadius
        goalReached = true;
        kEnd = k;
        break;
    end
end

log = truncateLog(log, kEnd);

log.goalReached = goalReached;
log.scenario    = scn.name;
log.sihScenario = scn.sihScenario;
log.seed        = seed;
log.planner     = func2str(plannerFn);
log.profile     = cfg.profile;
log.perfectPerception = perfectPer;
log.goal        = scn.goal;
log.goalRadius  = scn.goalRadius;
log.truthPotholes = scn.potholes;

M = computeMetrics(log, cfg);
end

% =========================================================================
function [colDyn, colStat, clrDyn, clrStat] = refereeCollision(ego, truth, grid, vp)
%REFEREECOLLISION Exact body rectangle against road users and the grid.
C = egoFootprint(ego.pos, ego.heading, vp);
clrDyn = Inf;
for i = 1:numel(truth)
    o = truth(i);
    if hypot(ego.pos(1) - o.pos(1), ego.pos(2) - o.pos(2)) > vp.circumRadius + hypot(o.length, o.width) + 5
        continue;
    end
    B = boxCorners(o.pos, o.heading, o.length, o.width);
    clrDyn = min(clrDyn, boxDistance(C, B));
end
clrStat = footprintGridClearance(grid, C, 2.0);
colDyn  = clrDyn <= 0;
colStat = clrStat <= 0;
end

% =========================================================================
function [events, inside] = potholeEntries(ego, potholes, inside, vp, cfg, t)
%POTHOLEENTRIES Log each time a tyre ENTERS a true pothole.
events = struct('t', {}, 'id', {}, 'severity', {}, 'speed', {}, 'depth', {});
if isempty(potholes), return; end
ct = cos(ego.heading);  st = sin(ego.heading);
lat = cfg.pothole.wheelTrack / 2;
wheels = [0, lat; 0, -lat; vp.wheelbase, lat; vp.wheelbase, -lat];
W = [ego.pos(1) + wheels(:,1)*ct - wheels(:,2)*st, ...
     ego.pos(2) + wheels(:,1)*st + wheels(:,2)*ct];
for p = 1:numel(potholes)
    ph = potholes(p);
    dx = W(:,1) - ph.pos(1);  dy = W(:,2) - ph.pos(2);
    c = cos(ph.yaw);  s = sin(ph.yaw);
    u = dx*c + dy*s;  v = -dx*s + dy*c;
    hitNow = any((u / (ph.length/2)).^2 + (v / (ph.width/2 + cfg.pothole.tyreWidth/2)).^2 <= 1);
    if hitNow && ~inside(p)
        events(end+1) = struct('t', t, 'id', ph.id, 'severity', ph.severity, ...
                               'speed', ego.speed, 'depth', ph.depth); %#ok<AGROW>
    end
    inside(p) = hitNow;
end
end

% =========================================================================
function tracks = perfectPotholeTracks(potholes, cfg)
%PERFECTPOTHOLETRACKS Ground truth in pothole-track format (diagnostic mode).
tracks = struct('id',{},'pos',{},'length',{},'width',{},'yaw',{},'depth',{}, ...
                'depthStd',{},'posStd',{},'hits',{},'confirmed',{},'state',{}, ...
                'severity',{},'riskLevel',{},'confidence',{},'firstSeen',{}, ...
                'lastSeen',{},'truthId',{});
for p = 1:numel(potholes)
    ph = potholes(p);
    [sev, rl] = classifyPotholeSeverity(ph.depth, cfg);
    tracks(end+1) = struct('id', ph.id, 'pos', ph.pos, 'length', ph.length, ...
        'width', ph.width, 'yaw', ph.yaw, 'depth', ph.depth, 'depthStd', 0, ...
        'posStd', 0, 'hits', Inf, 'confirmed', true, 'state', 'confirmed', ...
        'severity', sev, 'riskLevel', rl, 'confidence', 1, 'firstSeen', 0, ...
        'lastSeen', 0, 'truthId', ph.id); %#ok<AGROW>
end
end

% =========================================================================
function P = detPositions(dets)
if isempty(dets)
    P = zeros(0,2);
else
    P = reshape([dets.pos], 2, []).';
end
end

function P = potholeDetPositions(dets)
if isempty(dets)
    P = zeros(0,2);
else
    P = reshape([dets.pos], 2, []).';
end
end

% =========================================================================
function log = truncateLog(log, n)
f = fieldnames(log);
for i = 1:numel(f)
    v = log.(f{i});
    if isnumeric(v) || islogical(v)
        if size(v,1) >= n && size(v,1) > 1
            log.(f{i}) = v(1:n, :);
        end
    elseif iscell(v)
        if numel(v) >= n
            log.(f{i}) = v(1:n);
        end
    end
end
end

function v = getOpt(opts, name, defaultVal)
if isstruct(opts) && isfield(opts, name) && ~isempty(opts.(name))
    v = opts.(name);
else
    v = defaultVal;
end
end

function v = getFieldOr(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
