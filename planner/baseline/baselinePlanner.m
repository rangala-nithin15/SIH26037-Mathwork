function out = baselinePlanner(grid, ego, obstacles, cfg, state, roadHazards)
%BASELINEPLANNER Fixed-candidate reactive planner, for comparison only.
%
%   COMPONENT STATUS: REAL (a faithful implementation of the approach it
%   represents -- deliberately NOT an improved version of it)
%
%   out = BASELINEPLANNER(grid, ego, obstacles, cfg, state) plans using the
%   conventional structured-road recipe: a small fixed set of lateral lane
%   offsets, each evaluated against CURRENT obstacle positions, pick the
%   best. This is the approach IR-PSC is claimed to improve upon, so it
%   exists here to make that claim measurable rather than rhetorical.
%   out = BASELINEPLANNER(..., roadHazards) also receives confirmed pothole
%   tracks, with the same interface as IRPSCPLANNER.
%
%   WHY THIS FILE MATTERS
%   ---------------------
%   A contribution that is never compared against the obvious alternative is
%   an assertion, not a result. This baseline is what makes the IR-PSC
%   comparison honest. It is implemented properly -- same corridor input,
%   same vehicle limits, same scoring function, same safe-stop fallback --
%   so any difference in the results comes from the planning approach and
%   not from one planner being handicapped.
%
%   THE THREE DIFFERENCES, WHICH ARE THE WHOLE POINT
%   ------------------------------------------------
%     1. FIXED CANDIDATES. Five discrete lateral offsets
%        (cfg.baseline.candidateOffsets) instead of a continuously
%        deformable path. On a road whose usable width varies, the right
%        offset is usually not one of the five.
%
%     2. NO PREDICTION. Obstacles are treated as static at their current
%        positions. A pedestrian who will be in the road in 1.5 seconds is
%        invisible to this planner until they are already there.
%
%     3. NO CONFIDENCE AWARENESS. It plans at full commitment regardless of
%        how poor its inputs are. There is no mechanism by which uncertainty
%        can change its behaviour.
%
%   It DOES still use the drivable-space corridor for its reference path and
%   its lateral limits. That is a deliberate choice: giving the baseline the
%   corridor isolates the effect of prediction, continuous deformation and
%   confidence, rather than conflating them with the separate benefit of not
%   needing lane markings. The comparison is therefore conservative -- it
%   understates the full IR-PSC advantage rather than inflating it.
%
%   Phase 2: the baseline received the same infrastructure fixes as IR-PSC,
%   so the comparison stays fair: the jerk-limited speed profile, narrow and
%   blocked passage handling, following a road user in its lane (against
%   FROZEN obstacles, consistent with "no prediction"), slowing for potholes
%   its chosen candidate drives over, and a pothole term in its candidate
%   score. It still has no pothole avoidance beyond picking among its five
%   fixed offsets.
%
%   Inputs and Outputs match IRPSCPLANNER exactly, so the two are drop-in
%   interchangeable in the simulation loop and in the metrics code.
%   Additional output field:
%       .chosenOffset - the lateral offset selected, in metres
%
%   Example:
%       out = baselinePlanner(grid, ego, obstacles, cfg, []);
%
%   Requires: base MATLAB only.
%
%   See also IRPSCPLANNER, SCORETRAJECTORY, RUNCOMPARISON.

if nargin < 5, state = []; end %#ok<NASGU>
if nargin < 6, roadHazards = []; end

vp     = vehicleParams(cfg);
timing = struct();
tAll   = tic;

out = plannerOutputTemplate();
out.chosenOffset = 0;

% --- Corridor (shared with IR-PSC, see note above) ---------------------
t0 = tic;
corridor = extractCorridor(grid, ego, cfg, []);   % no temporal smoothing
timing.corridor = toc(t0);

% --- No prediction: obstacles frozen where they are --------------------
t0 = tic;
cfgNoPred = cfg;
cfgNoPred.ablation.usePrediction  = false;
cfgNoPred.ablation.useUncertainty = false;
preds = predictObstacles(obstacles, cfgNoPred, corridor);
timing.prediction = toc(t0);

% --- No confidence model -------------------------------------------------
out.corridor            = corridor;
out.preds               = preds;
out.confidence          = 1.0;
out.confidenceBreakdown = struct('note', 'baseline planner does not model confidence');
[out.ttc, ~, ttcDetails] = timeToConflict(ego, preds, cfg, vp, corridor);
out.ttcLevel = ttcDetails.level;

hz = confirmedPotholes(roadHazards);

if ~corridor.valid
    out.status = 'no_corridor';
    out = baselineSafeStop(out, ego, corridor, cfg, vp, preds, obstacles, ...
                           grid, timing, tAll);
    return;
end

if isempty(corridor.blockedIdx)
    lastUse = size(corridor.center,1);
else
    lastUse = max(corridor.blockedIdx - 1, 2);
end
corrUse = truncateCorridor(corridor, lastUse);
out.preferredPath = corrUse.center;
N = size(corrUse.center, 1);

% --- Shared speed ceilings (narrow / blocked) -----------------------------
ceilBase = inf(N,1);
ceilBase(corrUse.narrow) = cfg.corridor.narrowSpeed;
if ~isempty(corridor.blockedIdx)
    sStop = corridor.usableLength - vp.frontOverhang - cfg.corridor.stopStandoff;
    ceilBase(corrUse.s >= sStop) = 0;
end
% Same rule as IR-PSC: be able to stop within the known drivable space.
ceilBase(corrUse.s >= corrUse.s(end) - vp.frontOverhang - cfg.corridor.stopStandoff) = 0;

% --- Fixed candidate set --------------------------------------------------
candidateOffsets = cfg.baseline.candidateOffsets;
[dMin, dMax] = corridorBounds(corrUse, vp.halfWidth, cfg.deform.boundaryMargin, ...
                              cfg.deform.minBoundaryMargin);

t0 = tic;
bestScore  = Inf;
bestTraj   = [];
bestOffset = 0;
bestBeh    = out.behaviour;
for c = 1:numel(candidateOffsets)
    d = candidateOffsets(c);
    if any(d < dMin - 1e-9) || any(d > dMax + 1e-9)
        continue;                     % would leave the corridor somewhere
    end
    dProfile = repmat(d, N, 1);

    ceilSta = ceilBase;
    beh = out.behaviour;
    potholeCost = 0;
    if ~isempty(hz)
        hit = potholeWheelOverlap(corrUse, dProfile, hz, cfg, vp);
        for p = 1:numel(hz)
            rows = find(hit(:,1,p));
            if isempty(rows), continue; end
            zone = corrUse.s >= corrUse.s(rows(1)) - cfg.pothole.slowdownLead & ...
                   corrUse.s <= corrUse.s(rows(end));
            ceilSta(zone) = min(ceilSta(zone), cfg.pothole.speed.(hz(p).severity));
            potholeCost = potholeCost + cfg.pothole.cost.(hz(p).severity);
            beh.potholeSlow = true;
        end
    end
    [leadCeil, leadInfo] = leadVehicleCeiling(corrUse, dProfile, preds, obstacles, ego, cfg, vp);
    ceilSta = min(ceilSta, leadCeil);
    beh.following = leadInfo.active;
    beh.lead = leadInfo;

    traj = generateTrajectory(corrUse, dProfile, ego, cfg, zeros(N,1), ceilSta);
    if ~traj.valid
        continue;
    end
    % Frozen predictions make the time-aware check a current-position check,
    % which is exactly what "no prediction" means.
    clearOk = checkClearance(traj, grid, obstacles, cfg, vp, preds);
    feasOk  = checkFeasibility(traj, cfg, vp);
    if ~clearOk || ~feasOk
        continue;
    end
    s = scoreTrajectory(traj, corrUse, preds, cfg, vp) + cfg.score.wPothole * potholeCost;
    if s < bestScore
        bestScore  = s;
        bestTraj   = traj;
        bestOffset = d;
        bestBeh    = beh;
    end
end
timing.candidates = toc(t0);

if isempty(bestTraj)
    out.status = 'no_feasible_candidate';
    out = baselineSafeStop(out, ego, corridor, cfg, vp, preds, obstacles, ...
                           grid, timing, tAll);
    return;
end

[clearOk, minClear] = checkClearance(bestTraj, grid, obstacles, cfg, vp, preds);
out.traj         = bestTraj;
out.chosenOffset = bestOffset;
out.risk         = conflictRisk(bestTraj, preds, cfg, vp);
out.score        = bestScore;
out.clearanceOk  = clearOk;
out.minClearance = minClear;
out.feasible     = checkFeasibility(bestTraj, cfg, vp);
out.ttcPlanned   = timeToConflict(ego, preds, cfg, vp, bestTraj);
out.emergencyBrake = out.ttcPlanned <= cfg.risk.ttcCritical;
bestBeh.avoiding = abs(bestOffset) > 0.5;
bestBeh.blockedAhead = ~isempty(corridor.blockedIdx);
bestBeh.reason = sprintf('fixed candidate offset %+.1f m', bestOffset);
out.behaviour    = bestBeh;
out.state        = struct('center', corridor.center, 'profile', [], ...
                          'trajPos', bestTraj.pos(bestTraj.reachable,:));

timing.total = toc(tAll);
out.timing   = timing;
end

% =====================================================================
function out = baselineSafeStop(out, ego, corridor, cfg, vp, preds, ...
                                obstacles, grid, timing, tAll)
% The baseline has no graded response: any planning failure is a full stop.
traj = safeStopTrajectory(ego, corridor, cfg, true);

out.traj         = traj;
out.isSafeStop   = true;
out.emergencyBrake = true;
out.chosenOffset = 0;
out.risk         = conflictRisk(traj, preds, cfg, vp);
out.score        = scoreTrajectory(traj, corridor, preds, cfg, vp);
[clearOk, minClear] = checkClearance(traj, grid, obstacles, cfg, vp, preds);
out.clearanceOk  = clearOk;
out.minClearance = minClear;
out.feasible     = checkFeasibility(traj, cfg, vp);
out.ttcPlanned   = timeToConflict(ego, preds, cfg, vp, traj);
beh = out.behaviour;
beh.reason = sprintf('safe stop (%s)', strrep(out.status, '_', ' '));
out.behaviour    = beh;
out.state        = struct('center', corridor.center, 'profile', [], 'trajPos', traj.pos);

timing.total = toc(tAll);
out.timing   = timing;
end
