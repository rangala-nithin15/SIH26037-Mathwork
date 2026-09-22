function out = irpscPlanner(grid, ego, obstacles, cfg, state, roadHazards)
%IRPSCPLANNER Indian-Road Predictive Safety Corridor planner.
%
%   COMPONENT STATUS: REAL (base MATLAB; no toolbox required)
%
%   out = IRPSCPLANNER(grid, ego, obstacles, cfg, state) runs one complete
%   planning cycle and returns the trajectory to drive, together with the
%   confidence and risk information the decision logic needs.
%   out = IRPSCPLANNER(..., roadHazards) also plans around CONFIRMED pothole
%   tracks (Phase 2; see perception/tracking/potholeTracker.m).
%
%   ============================================================
%   WHAT IS AND IS NOT CLAIMED AS THE CONTRIBUTION
%   ============================================================
%   The individual techniques used here -- ray casting, Frenet-style
%   road-aligned coordinates, Gaussian uncertainty propagation, dynamic
%   programming, moving-average smoothing -- are all standard and are NOT
%   claimed as novel.
%
%   The contribution is the HIERARCHY: which quantity is treated as primary,
%   and what the system does when it is unsure. Specifically:
%
%     - Drivable space, not lane markings, is the primary planning
%       constraint. The planner never asks where the lane is. On an unmarked
%       village road it behaves no differently in principle than on a marked
%       highway.
%
%     - The preferred path is CONTINUOUS -- the corridor centreline, which
%       moves smoothly as the road bends and narrows. It is not a choice
%       among a handful of fixed lane-offset candidates.
%
%     - Predicted occupancy with explicit uncertainty drives avoidance, and
%       uncertainty changes behaviour rather than merely being reported: a
%       vague prediction inflates the hazard ellipse and the planner gives
%       it more room, with no special-case code.
%
%     - Confidence in the system's own inputs is measured and acted upon,
%       degrading autonomy before a failure rather than after one.
%
%   ============================================================
%   THE 13 IR-PSC STEPS, AND WHERE EACH ONE LIVES
%   ============================================================
%     1. Estimate drivable space        -> seedCenterline
%     2. Identify road boundaries       -> extractCorridor
%     3. Track surrounding road users   -> (input; perception/tracking)
%     4. Predict short-term occupancy   -> predictObstacles
%     5. Represent uncertainty          -> predictionUncertainty
%     6. Estimate collision risk        -> lateralRiskGrid, conflictRisk
%     7. Continuous preferred path      -> extractCenterline
%     8. Deform around hazards          -> deformTrajectory
%     9. Preserve boundary margins      -> corridorBounds, checkClearance
%    10. Smooth spatially/temporally    -> generateTrajectory, smoothPath
%    11. Check vehicle feasibility      -> checkFeasibility
%    12. Confidence-aware behaviour     -> computeConfidence
%    13. Degrade or stop safely         -> safeStopTrajectory
%
%   ============================================================
%   PHASE 2: THE RESPONSE CASCADE
%   ============================================================
%   The original planner had two outcomes: a deformed path, or a safe stop
%   (which the decision logic executed as an emergency stop). Without the
%   graded responses of ordinary driving the vehicle stopped dead in front
%   of every slow truck and every narrow gap. From least to most intrusive:
%
%     a. DEFORM   the continuous path around predicted hazards and potholes
%                 (dynamic programme, unchanged in principle).
%     b. SLOW     through narrow stations (cfg.corridor.narrowSpeed), over
%                 potholes that are not worth avoiding (cfg.pothole.speed),
%                 and where predicted risk is elevated.
%     c. FOLLOW   a road user that stays in the planned path
%                 (leadVehicleCeiling).
%     d. YIELD    if the time-aware clearance check still finds a conflict,
%                 plan a controlled stop before it and re-check.
%     e. STOP     before a blocked station (free width below the body plus
%                 the hard minimum clearance), with service braking.
%     f. SAFE STOP only when none of the above yields a trajectory that is
%                 clear and feasible. EMERGENCY braking is requested only
%                 when the required deceleration exceeds service braking or
%                 the conflict on the planned path is imminent.
%
%   No safety threshold was relaxed to obtain this behaviour: the same
%   clearance, feasibility and jerk limits apply to every trajectory.
%
%   ============================================================
%   SAFETY STATEMENT
%   ============================================================
%   This is simulation research code. It is NOT safety certified, has NOT
%   been validated on a real vehicle, and makes NO claim of guaranteed
%   collision avoidance. The checks below reduce risk within the modelled
%   world; they cannot bound behaviour in the real one.
%
%   Inputs:
%       grid      - occupancy grid struct from makeOccupancyGrid(), giving
%                   drivable space. Non-drivable cells include road edges,
%                   buildings, parked vehicles, stalls and debris. Potholes
%                   are NOT occupied cells: the road is drivable over them.
%       ego       - ego state struct from makeEgoState()
%       obstacles - obstacle struct array from makeObstacle(), may be empty
%       cfg       - config struct from irpscConfig()
%       state     - (optional) planner memory from the previous cycle, for
%                   temporal smoothing. Pass [] on the first cycle.
%       roadHazards - (optional) struct array of pothole tracks (fields .id
%                   .pos .length .width .yaw .depth .severity .confidence
%                   .confirmed). Only confirmed tracks are used.
%
%   Outputs:
%       out - struct with fields:
%           .traj        trajectory struct to drive (see generateTrajectory)
%           .corridor    corridor struct from extractCorridor()
%           .preds       prediction struct array
%           .confidence  scalar in [0,1]
%           .confidenceBreakdown  struct explaining the confidence value
%           .risk        peak predicted risk along the chosen trajectory
%           .ttc         time to conflict if the vehicle keeps its speed and
%                        lateral offset ALONG THE ROAD (nominal, s)
%           .ttcLevel    'none' | 'warning' | 'critical' (nominal)
%           .ttcPlanned  time to conflict along the planned trajectory (s)
%           .emergencyBrake logical, braking beyond service is required
%           .requiredDecel  m/s^2 needed to honour a planned stop
%           .feasible    logical, trajectory passed the feasibility check
%           .clearanceOk logical, trajectory passed the clearance check
%           .minClearance scalar, tightest clearance found (m)
%           .status      char, one of:
%                        'ok' | 'no_corridor' | 'deformation_infeasible' |
%                        'clearance_failed' | 'feasibility_failed'
%           .behaviour   flags .avoiding .slowing .following .yielding
%                        .narrowPassage .blockedAhead .potholeAvoid
%                        .potholeSlow, plus .lead (struct) and .reason (char)
%           .potholes    confirmed potholes with planned .action
%                        ('avoid' | 'slow' | 'straddle') and .plannedSpeed
%           .preferredPath Nx2 undeformed corridor centreline (usable part)
%           .riskGrid    struct .R .offsets .potholeCost .s .center .heading
%           .isSafeStop  logical, a safe-stop trajectory was returned
%           .score       trajectory score from scoreTrajectory()
%           .state       planner memory to pass into the next cycle
%           .timing      struct of per-stage elapsed seconds
%
%   Example:
%       cfg   = irpscConfig('village');
%       state = [];
%       out   = irpscPlanner(grid, ego, obstacles, cfg, state);
%       state = out.state;   % feed forward for temporal smoothing
%       out   = irpscPlanner(grid, ego, obstacles, cfg, state, potholeTracks);
%
%   Requires: base MATLAB only. No toolbox is used anywhere in this call
%   tree, so the planner can be unit-tested on any MATLAB installation.
%
%   See also EXTRACTCORRIDOR, PREDICTOBSTACLES, DEFORMTRAJECTORY,
%            COMPUTECONFIDENCE, SAFESTOPTRAJECTORY, DECISIONLOGIC.

if nargin < 5, state = []; end
if nargin < 6, roadHazards = []; end

vp = vehicleParams(cfg);
timing = struct();
tAll = tic;

% Previous-cycle memory for temporal smoothing.
priorCenter = getFieldOr(state, 'center',   []);
prevTrajPos = getFieldOr(state, 'trajPos',  []);

out = plannerOutputTemplate();

% =====================================================================
% STEPS 1-2: drivable space -> corridor
% =====================================================================
t0 = tic;
corridor = extractCorridor(grid, ego, cfg, priorCenter);
timing.corridor = toc(t0);

% =====================================================================
% STEPS 3-5: predict road users, with uncertainty
% =====================================================================
t0 = tic;
preds = predictObstacles(obstacles, cfg, corridor);
timing.prediction = toc(t0);

% =====================================================================
% STEP 12: confidence in our own inputs
% =====================================================================
t0 = tic;
[conf, confBreak] = computeConfidence(corridor, preds, obstacles, cfg);
timing.confidence = toc(t0);

% Following traffic (behind, same direction) must not make the ego brake:
% see isFollowingRoadUser. It stays in the lateral risk grid only.
behind   = isFollowingRoadUser(ego, obstacles, vp);
obsAhead = obstacles(~behind);
predAhead = preds(~behind);

% Nominal ("do nothing") time to conflict, following the road.
[ttcNom, ~, ttcDetails] = timeToConflict(ego, predAhead, cfg, vp, corridor);

out.corridor             = corridor;
out.preds                = preds;
out.confidence           = conf;
out.confidenceBreakdown  = confBreak;
out.ttc                  = ttcNom;
out.ttcLevel             = ttcDetails.level;
out.potholes             = describePotholes(roadHazards);

% --- No usable corridor at all: stop -----------------------------------
if ~corridor.valid
    out.status = 'no_corridor';
    distAvail  = max(corridor.usableLength - vp.frontOverhang - cfg.corridor.stopStandoff, 0.1);
    needEmerg  = ego.speed^2 / (2 * distAvail) > cfg.ego.maxDecel * cfg.safety.emergencyMargin;
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, predAhead, obsAhead, ...
                             grid, timing, tAll, needEmerg, prevTrajPos);
    return;
end

% Plan on the USABLE part of the corridor only.
if isempty(corridor.blockedIdx)
    lastUse = size(corridor.center,1);
else
    lastUse = max(corridor.blockedIdx - 1, 2);
end
corrUse = truncateCorridor(corridor, lastUse);
out.preferredPath = corrUse.center;     % replaced below by the preferred-offset path
Nu = size(corrUse.center, 1);

% =====================================================================
% STEPS 6-8: risk grid (+ pothole cost), then deform the preferred path
% =====================================================================
t0 = tic;
[dMin, dMax] = corridorBounds(corrUse, vp.halfWidth, cfg.deform.boundaryMargin, ...
                              cfg.deform.minBoundaryMargin);
[R, offsets] = lateralRiskGrid(corrUse, preds, ego, cfg, vp, dMin, dMax);
confirmedHazards = confirmedPotholes(roadHazards);
[Cp, potInfo] = potholeCostGrid(corrUse, offsets, confirmedHazards, cfg, vp);
[Cs, staticClr] = staticClearanceCost(corrUse, offsets, grid, cfg, vp);
% The chosen profile is smoothed afterwards (smoothOffsetProfile), which
% blurs it along the road by about smoothWindow*smoothPasses stations. A
% dodge exactly as long as a pothole would be smoothed back over it, so the
% surface and static costs are dilated along the road by that reach first:
% the dynamic programme then holds the avoiding offset long enough for the
% smoothed path to keep it (Phase 2).
reach = cfg.traj.smoothWindow * cfg.traj.smoothPasses;
extraCost = dilateAlongRoad(cfg.deform.wPothole * Cp + Cs, reach);

% A station no offset can pass (bounds and static clearance) limits the
% usable corridor exactly like a blocked station: plan up to it and stop
% before it, instead of declaring the whole plan infeasible.
rowBlocked = all(~isfinite(R) | ~isfinite(extraCost), 2);
rowBlocked(1) = false;
iBlk = find(rowBlocked, 1, 'first');
sStaticStop = Inf;
if ~isempty(iBlk) && iBlk >= 3
    keepRows = 1:(iBlk - 1);
    sStaticStop = corrUse.s(iBlk) - vp.frontOverhang - cfg.corridor.stopStandoff;
    corrUse = truncateCorridor(corrUse, iBlk - 1);
    R = R(keepRows, :);  Cp = Cp(keepRows, :);  Cs = Cs(keepRows, :);
    staticClr = staticClr(keepRows, :);  extraCost = extraCost(keepRows, :);
    Nu = numel(keepRows);
    out.preferredPath = corrUse.center;
end
timing.riskGrid = toc(t0);

out.riskGrid = struct('R', R, 'offsets', offsets, 'potholeCost', Cp, ...
                      'staticClearance', staticClr, ...
                      's', corrUse.s, 'center', corrUse.center, ...
                      'heading', corrUse.heading);

[~, d0] = projectPointOnPath(corrUse.center, ego.pos);
priorProfile = profileFromPath(corrUse, prevTrajPos);

t0 = tic;
% Lateral changes that are comfortable at walking pace are violent at
% 40 km/h: the smoothness weight grows with the square of speed.
cfgDP = cfg;
cfgDP.deform.wSmooth = cfg.deform.wSmooth * max(1, (ego.speed / cfg.deform.smoothRefSpeed)^2);
[dProfile, defInfo] = deformTrajectory(R, offsets, cfgDP, d0, priorProfile, extraCost);
timing.deformation = toc(t0);
if isfield(defInfo, 'preferred') && numel(defInfo.preferred) == Nu
    out.preferredPath = frenetToCartesian(corrUse.center, corrUse.s, defInfo.preferred);
end

if ~defInfo.feasible
    out.status = 'deformation_infeasible';
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, predAhead, obsAhead, ...
                             grid, timing, tAll, false, prevTrajPos);
    return;
end

% The vehicle IS at d0: station 1 is exactly there, not at the nearest
% lateral grid value. (Quantising it produced a systematic kink ~4 m ahead
% of the vehicle once the path's first point was snapped back to the car.)
dProfile(1) = d0;

% =====================================================================
% STEPS 9-10: speed ceilings, trajectory, smoothing
% =====================================================================
t0 = tic;
stationRisk = zeros(Nu,1);
for i = 1:Nu
    [~, j] = min(abs(offsets - dProfile(i)));
    r = R(i,j);
    if ~isfinite(extraCost(i,j)), r = 1; end
    if ~isfinite(r), r = 1; end
    stationRisk(i) = r;
end

% Same smoothing generateTrajectory applies, so ceilings refer to the path
% that will actually be driven.
dPath = smoothOffsetProfile(dProfile, cfg);

ceilSta = inf(Nu,1);
beh = out.behaviour;

% (b) narrow stations
if any(corrUse.narrow)
    ceilSta(corrUse.narrow) = min(ceilSta(corrUse.narrow), cfg.corridor.narrowSpeed);
    % Only a narrow passage close ahead is worth reporting as a hazard.
    beh.narrowPassage = any(corrUse.narrow & corrUse.s <= max(15, 3 * ego.speed));
end

% (e) blocked station ahead: stop before it with service braking
sBlockStop = Inf;
if ~isempty(corridor.blockedIdx)
    sBlockStop = corridor.usableLength - vp.frontOverhang - cfg.corridor.stopStandoff;
end
sBlockStop = min(sBlockStop, sStaticStop);
if isfinite(sBlockStop)
    ceilSta(corrUse.s >= sBlockStop) = 0;
    beh.blockedAhead = true;
end

% Never outrun the known drivable space: the vehicle must always be able to
% stop before the end of the usable corridor. On a full-length corridor this
% constraint is inactive (the stopping distance is shorter than the
% lookahead); when the corridor ends early -- a wall across the road, a
% dead end -- it is what brings the vehicle to a controlled stop in time.
sEndStop = corrUse.s(end) - vp.frontOverhang - cfg.corridor.stopStandoff;
ceilSta(corrUse.s >= sEndStop) = 0;
if corridor.stalled
    % The seed march itself was stopped by an obstruction: the corridor
    % really does end here.
    beh.blockedAhead = true;
    sBlockStop = min(sBlockStop, sEndStop);
end

% (b) potholes on the chosen path
if isfield(defInfo, 'preferred') && numel(defInfo.preferred) == Nu
    dPref = defInfo.preferred;
else
    dPref = zeros(Nu,1);
end
[ceilSta, out.potholes, beh] = applyPotholeCeilings(ceilSta, corrUse, dPath, dPref, ...
                                   confirmedHazards, out.potholes, beh, cfg, vp);

% (c) follow a road user that stays in the path
[leadCeil, leadInfo] = leadVehicleCeiling(corrUse, dPath, predAhead, obsAhead, ego, cfg, vp);
ceilSta = min(ceilSta, leadCeil);
beh.following = leadInfo.active && any(isfinite(leadCeil) & leadCeil < cfg.ego.maxSpeed);
beh.lead      = leadInfo;

traj = generateTrajectory(corrUse, dProfile, ego, cfg, stationRisk, ceilSta);
timing.trajectory = toc(t0);

% =====================================================================
% STEPS 9 and 11: time-aware clearance (with yielding) and feasibility
% =====================================================================
t0 = tic;
[clearOk, minClear, cdet] = checkClearance(traj, grid, obsAhead, cfg, vp, predAhead);
sYieldStop = Inf;
for attempt = 1:3
    if clearOk || isempty(cdet.firstViolationIdx) || cdet.firstViolationIdx <= 1
        break;
    end
    % (d) yield: plan a stop before the first conflicting sample, re-check.
    sViol = traj.s(cdet.firstViolationIdx) / max(traj.s(end), eps) * corrUse.s(end);
    sYieldStop = min(sYieldStop, sViol - cfg.safety.yieldStandoff);
    ceilSta(corrUse.s >= sYieldStop) = 0;
    traj = generateTrajectory(corrUse, dProfile, ego, cfg, stationRisk, ceilSta);
    [clearOk, minClear, cdet] = checkClearance(traj, grid, obsAhead, cfg, vp, predAhead);
    beh.yielding = true;
end
[feasOk, feasDet] = checkFeasibility(traj, cfg, vp);
if ~feasOk && clearOk
    % (b') SLOW for feasibility: a path that is too dynamic at the current
    % speed is retried with a lower speed cap before giving up. Curvature
    % and steering violations cannot be fixed this way and remain failures.
    ceilRetry = min(ceilSta, cfg.traj.feasibilityRetrySpeed * cfg.ego.maxSpeed);
    trajRetry = generateTrajectory(corrUse, dProfile, ego, cfg, stationRisk, ceilRetry);
    [okC, mcR, cdR] = checkClearance(trajRetry, grid, obsAhead, cfg, vp, predAhead);
    [okF, fdR] = checkFeasibility(trajRetry, cfg, vp);
    if okC && okF
        traj = trajRetry;  ceilSta = ceilRetry;
        clearOk = okC;  minClear = mcR;  cdet = cdR;
        feasOk = okF;   feasDet = fdR;
        beh.slowing = true;
    end
end
timing.checks = toc(t0);

out.clearanceOk  = clearOk;
out.minClearance = minClear;
out.feasible     = feasOk;
out.feasibilityDetails = feasDet;
out.clearanceDetails   = cdet;

% Braking demand of any planned stop (rear-axle distance to the stop point).
sStopWanted = min([sBlockStop, sYieldStop]);
if isfinite(sStopWanted)
    out.requiredDecel = ego.speed^2 / (2 * max(sStopWanted, 0.1));
end

out.rejectedTraj = [];
if ~clearOk || ~feasOk
    out.rejectedTraj = traj;          % shown in the viewer as the rejected candidate
end

if ~clearOk
    out.status = 'clearance_failed';
    imminent = cdet.firstViolationTime <= cfg.risk.ttcCritical || ...
               out.requiredDecel > cfg.ego.maxDecel * cfg.safety.emergencyMargin;
    out.behaviour = beh;
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, predAhead, obsAhead, ...
                             grid, timing, tAll, imminent, prevTrajPos);
    return;
end

if ~feasOk
    out.status = 'feasibility_failed';
    out.behaviour = beh;
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, predAhead, obsAhead, ...
                             grid, timing, tAll, false, prevTrajPos);
    return;
end

% =====================================================================
% STEP 11 (scoring) and hand-off
% =====================================================================
out.traj  = traj;
out.risk  = conflictRisk(traj, predAhead, cfg, vp);
out.score = scoreTrajectory(traj, corrUse, predAhead, cfg, vp);
out.ttcPlanned = timeToConflict(ego, predAhead, cfg, vp, traj);

if isfield(defInfo, 'preferred') && numel(defInfo.preferred) == Nu
    beh.avoiding = max(abs(dPath - defInfo.preferred)) > 0.5;   % bends away from the preferred path
else
    beh.avoiding = max(abs(dPath)) > 0.5;
end
reachSpeeds  = traj.speed(traj.reachable);
beh.slowing  = min(reachSpeeds) < 0.9 * cfg.ego.maxSpeed && ...
               (any(isfinite(ceilSta)) || max(stationRisk) >= cfg.decision.riskHazard);
out.emergencyBrake = out.requiredDecel > cfg.ego.maxDecel * cfg.safety.emergencyMargin || ...
                     out.ttcPlanned <= cfg.risk.ttcCritical;
beh.reason = describeBehaviour(beh, out);
out.behaviour = beh;

out.state = struct('center', corridor.center, 'profile', dProfile, ...
                   'trajPos', traj.pos(traj.reachable, :));

timing.total = toc(tAll);
out.timing   = timing;
end

% =====================================================================
function out = finishWithSafeStop(out, ego, corridor, cfg, vp, preds, ...
                                  obstacles, grid, timing, tAll, emergency, refPath)
%FINISHWITHSAFESTOP Populate the output with a safe-stop trajectory.
%   emergency true uses emergency braking, false service braking. The stop
%   follows the path the vehicle was already on (refPath) where possible.
traj = safeStopTrajectory(ego, corridor, cfg, emergency, refPath);

out.traj       = traj;
out.isSafeStop = true;
out.emergencyBrake = logical(emergency);
out.risk       = conflictRisk(traj, preds, cfg, vp);
if isstruct(corridor) && isfield(corridor, 'center')
    out.score  = scoreTrajectory(traj, corridor, preds, cfg, vp);
end
out.ttcPlanned = timeToConflict(ego, preds, cfg, vp, traj);

[clearOk, minClear] = checkClearance(traj, grid, obstacles, cfg, vp, preds);
if ~strcmp(out.status, 'clearance_failed')
    out.clearanceOk = clearOk;
end
out.minClearance = min(out.minClearance, minClear);
if ~strcmp(out.status, 'feasibility_failed')
    out.feasible = checkFeasibility(traj, cfg, vp);
end

beh = out.behaviour;
beh.reason = sprintf('safe stop (%s)', strrep(out.status, '_', ' '));
out.behaviour = beh;

% Do not carry a deformation profile forward out of a failed cycle; the
% stop path itself becomes the reference for the next cycle.
out.state = struct('center', corridor.center, 'profile', [], 'trajPos', traj.pos);

timing.total = toc(tAll);
out.timing   = timing;
end

% =====================================================================
function D = dilateAlongRoad(C, w)
%DILATEALONGROAD Running maximum of each column over +/- w stations.
D = C;
N = size(C,1);
if w <= 0 || N < 2, return; end
for i = 1:N
    D(i,:) = max(C(max(1,i-w):min(N,i+w), :), [], 1);
end
% The first stations stay as they were: the vehicle is already there.
D(1,:) = C(1,:);
end

% =====================================================================
function prof = profileFromPath(corr, pathPos)
%PROFILEFROMPATH Lateral offset of the previous plan at each CURRENT station.
%   NaN where the previous plan does not cover the station, so temporal
%   smoothing never compares different stretches of road.
N = size(corr.center, 1);
prof = nan(N,1);
if isempty(pathPos) || size(pathPos,1) < 2
    return;
end
sP = pathArcLength(pathPos);
if sP(end) < 1.0
    return;
end
for i = 1:N
    [sp, ~, ~, foot] = projectPointOnPath(pathPos, corr.center(i,:));
    if sp > 1e-3 && sp < sP(end) - 1e-3
        nL = [-sin(corr.heading(i)), cos(corr.heading(i))];
        prof(i) = (foot - corr.center(i,:)) * nL.';
    end
end
end

% =====================================================================
function list = describePotholes(hz)
list = struct('id', {}, 'severity', {}, 'action', {}, 'distance', {}, ...
              'plannedSpeed', {});
for i = 1:numel(hz)
    if ~(isfield(hz(i), 'confirmed') && hz(i).confirmed), continue; end
    list(end+1) = struct('id', hz(i).id, 'severity', hz(i).severity, ...
                         'action', 'none', 'distance', Inf, ...
                         'plannedSpeed', NaN); %#ok<AGROW>
end
end

% =====================================================================
function [ceilSta, list, beh] = applyPotholeCeilings(ceilSta, corr, dPath, dPref, ...
                                   hz, list, beh, cfg, vp)
%APPLYPOTHOLECEILINGS Speed caps for potholes the chosen path drives over,
%   and classification of every confirmed pothole ahead:
%     'slow'     - a tyre goes through it; speed capped to cfg.pothole.speed
%     'avoid'    - the undeformed preferred path would hit it, the chosen does not
%     'straddle' - it lies between the wheel tracks of the chosen path
%     'clear'    - it is ahead but away from both paths
if isempty(hz)
    return;
end
hitPath = potholeWheelOverlap(corr, dPath, hz, cfg, vp);   % Nx1xP
hitPref = potholeWheelOverlap(corr, dPref, hz, cfg, vp);   % Nx1xP
s = corr.s(:);
for p = 1:numel(hz)
    li = find([list.id] == hz(p).id, 1);
    if isempty(li), continue; end
    [sp, ~] = projectPointOnPath(corr.center, hz(p).pos);
    list(li).distance = sp;
    rows = find(hitPath(:,1,p));
    if ~isempty(rows)
        vCap = severityValue(cfg.pothole.speed, hz(p).severity);
        zone = s >= s(rows(1)) - cfg.pothole.slowdownLead & s <= s(rows(end));
        ceilSta(zone) = min(ceilSta(zone), vCap);
        list(li).action       = 'slow';
        list(li).plannedSpeed = vCap;
        beh.potholeSlow = true;
    elseif any(hitPref(:,1,p))
        list(li).action = 'avoid';
        beh.potholeAvoid = true;
    elseif sp > 0 && sp < s(end)
        [~, dp] = projectPointOnPath(corr.center, hz(p).pos);
        dAt = interp1(s, dPath(:), min(max(sp, s(1)), s(end)), 'linear');
        halfAcross = max(hz(p).length, hz(p).width) / 2;
        if abs(dp - dAt) + halfAcross < cfg.pothole.wheelTrack/2 - cfg.pothole.tyreWidth/2
            list(li).action = 'straddle';
        else
            list(li).action = 'clear';
        end
    end
end
end

function v = severityValue(table, sev)
if isfield(table, sev)
    v = table.(sev);
else
    v = table.moderate;
end
end

% =====================================================================
function r = describeBehaviour(beh, out)
parts = {};
if out.emergencyBrake,   parts{end+1} = 'EMERGENCY BRAKE'; end
if beh.blockedAhead,     parts{end+1} = 'stopping before blocked passage'; end
if beh.yielding,         parts{end+1} = 'yielding to predicted conflict'; end
if beh.following
    parts{end+1} = sprintf('following %s', beh.lead.class);
end
if beh.potholeAvoid,     parts{end+1} = 'steering around pothole'; end
if beh.potholeSlow,      parts{end+1} = 'slowing for pothole'; end
if beh.avoiding && ~beh.potholeAvoid, parts{end+1} = 'deforming around hazard'; end
if beh.narrowPassage,    parts{end+1} = 'narrow passage'; end
if isempty(parts)
    r = 'cruise along drivable corridor';
else
    r = strjoin(parts, ' | ');
end
end

% =====================================================================
function v = getFieldOr(s, name, defaultVal)
%GETFIELDOR Read a struct field, or return a default if it is absent.
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
