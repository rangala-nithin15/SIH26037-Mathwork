function [ok, minClear, details] = checkClearance(traj, grid, obstacles, cfg, vp, preds)
%CHECKCLEARANCE Verify a trajectory keeps clear of static space and road users.
%
%   COMPONENT STATUS: REAL
%
%   [ok, minClear, details] = CHECKCLEARANCE(traj, grid, obstacles, cfg, vp)
%   [ok, minClear, details] = CHECKCLEARANCE(traj, grid, obstacles, cfg, vp, preds)
%   measures the smallest clearance the ego vehicle BODY would have along the
%   trajectory, against the occupancy grid (road edges, buildings, parked
%   vehicles, stalls, debris) and against road users.
%
%   IR-PSC step 8: preserve road-boundary and obstacle safety margins.
%
%   This geometric check is deliberately separate from the probabilistic
%   risk assessment: it uses exact footprints and no uncertainty weighting,
%   so a mis-tuned risk model cannot silently disable it.
%
%   Phase 2 changes, and why
%   ------------------------
%   1. TIME-AWARE road users. The earlier version compared the CURRENT
%      position of every road user with EVERY point of the future path, up
%      to 70 m ahead. A slow truck 30 m ahead, or a pedestrian who will have
%      finished crossing long before the car arrives, therefore "violated
%      clearance" and caused an emergency stop. When `preds` is supplied,
%      each trajectory sample at time t is now compared with each road
%      user's PREDICTED pose at the same time t (mean prediction, within the
%      prediction horizon). The first samples (t ~ 0) are still compared
%      with where road users are now. Samples beyond the horizon are not
%      checked against road users; they are re-checked on later cycles.
%      Without `preds` the legacy current-position check is used.
%   2. EXACT RECTANGLES instead of enclosing circles and discs (see
%      BOXDISTANCE, FOOTPRINTGRIDCLEARANCE). A bus was previously a circle
%      of radius 5.6 m.
%   3. Only REACHABLE samples are checked. Path geometry beyond a planned
%      stop point is never driven.
%   4. If the plan comes to rest, the resting pose is also checked against
%      road users for the remainder of the horizon, and any conflict there
%      is reported separately as a stationary conflict (something moving
%      INTO the stopped vehicle).
%
%   Inputs:
%       traj      - trajectory struct from generateTrajectory() or
%                   safeStopTrajectory()
%       grid      - occupancy grid struct, or [] to skip the static check
%       obstacles - obstacle struct array (current state), may be empty
%       cfg       - config struct from irpscConfig()
%       vp        - vehicle params struct from vehicleParams()
%       preds     - (optional) prediction struct array from
%                   predictObstacles(obstacles, ...), same order
%
%   Outputs:
%       ok       - logical, true if every reachable sample keeps at least
%                  cfg.safety.minLateralClearance from everything
%       minClear - scalar smallest clearance found (m). Negative = overlap.
%       details  - struct with fields:
%                  .perSample          Mx1 clearance per sample (Inf if unchecked)
%                  .worstIndex         index of the tightest sample
%                  .staticMin          tightest clearance against the grid
%                  .dynamicMin         tightest clearance against road users
%                  .firstViolationIdx  first sample below the threshold ([] if none)
%                  .firstViolationTime its time from now (s), Inf if none
%                  .violationIsStatic  logical, the first violation is static
%                  .stationaryConflict logical, a road user is predicted to
%                                      reach the vehicle after it has stopped
%                  .timeAware          logical, predictions were used
%
%   Example:
%       [ok, mc, d] = checkClearance(traj, grid, obstacles, cfg, vp, preds);
%
%   Requires: base MATLAB only.
%
%   See also CHECKFEASIBILITY, BOXDISTANCE, FOOTPRINTGRIDCLEARANCE.

if nargin < 6, preds = []; end

M = size(traj.pos,1);
if isfield(traj, 'reachable') && numel(traj.reachable) == M
    reach = logical(traj.reachable(:));
else
    reach = true(M,1);
end
if isfield(traj, 'times') && numel(traj.times) == M
    tS = traj.times(:);
else
    tS = zeros(M,1);
end
th = traj.heading(:);

thr        = cfg.safety.minLateralClearance;
perSample  = inf(M,1);
staticPer  = inf(M,1);
staticMin  = Inf;
dynamicMin = Inf;

timeAware = ~isempty(preds) && numel(preds) == numel(obstacles);
% The HARD geometric check against predicted road users covers the near
% term (cfg.safety.clearanceHorizon), where a mean prediction is reliable.
% Beyond it the probabilistic risk model -- which carries the prediction's
% uncertainty -- governs. Checking a noisy 3-4 s mean prediction as if it
% were certain made the vehicle stop for oncoming traffic in its own lane
% whenever a tracked heading wobbled by a few degrees (Phase 2).
horizon   = min(cfg.prediction.horizon, cfg.safety.clearanceHorizon);
searchR   = cfg.safety.clearanceSearch;

for i = 1:M
    if ~reach(i), continue; end
    C = egoFootprint(traj.pos(i,:), th(i), vp);

    % --- Static: occupancy grid -------------------------------------------
    if ~isempty(grid)
        clr = footprintGridClearance(grid, C, searchR);
        staticPer(i) = clr;
        staticMin    = min(staticMin, clr);
        perSample(i) = min(perSample(i), clr);
    end

    % --- Dynamic: road users --------------------------------------------
    for k = 1:numel(obstacles)
        if timeAware
            if tS(i) > horizon, continue; end
            [op, oh] = predictedPose(preds(k), tS(i), obstacles(k));
        else
            op = obstacles(k).pos;  oh = obstacles(k).heading;
        end
        % Cheap reject before the exact rectangle test.
        cEgo = mean(C, 1);
        reachR = vp.circumRadius + hypot(obstacles(k).length, obstacles(k).width)/2 + searchR;
        if hypot(cEgo(1) - op(1), cEgo(2) - op(2)) > reachR
            continue;
        end
        B = boxCorners(op, oh, obstacles(k).length, obstacles(k).width);
        d = boxDistance(C, B);
        dynamicMin   = min(dynamicMin, d);
        perSample(i) = min(perSample(i), d);
    end
end

% --- Resting pose: is anything predicted to move into the stopped car? ----
stationaryConflict = false;
lastReach = find(reach, 1, 'last');
if timeAware && ~isempty(lastReach) && traj.speed(lastReach) <= 0.05 && ...
        tS(lastReach) < horizon
    C = egoFootprint(traj.pos(lastReach,:), th(lastReach), vp);
    for k = 1:numel(obstacles)
        tp = preds(k).times(:);
        tq = tp(tp >= tS(lastReach));
        for q = 1:numel(tq)
            [op, oh] = predictedPose(preds(k), tq(q), obstacles(k));
            B = boxCorners(op, oh, obstacles(k).length, obstacles(k).width);
            if boxDistance(C, B) < thr
                stationaryConflict = true;
                break;
            end
        end
        if stationaryConflict, break; end
    end
end

[minClear, worstIndex] = min(perSample);
if ~isfinite(minClear)
    minClear = Inf;
end
ok = minClear >= thr;

firstIdx = find(perSample < thr, 1, 'first');
details.perSample          = perSample;
details.worstIndex         = worstIndex;
details.staticMin          = staticMin;
details.dynamicMin         = dynamicMin;
details.firstViolationIdx  = firstIdx;
if isempty(firstIdx)
    details.firstViolationTime = Inf;
    details.violationIsStatic  = false;
else
    details.firstViolationTime = tS(firstIdx);
    details.violationIsStatic  = staticPer(firstIdx) < thr;
end
details.stationaryConflict = stationaryConflict;
details.timeAware          = timeAware;
end

% -------------------------------------------------------------------------
function [pos, heading] = predictedPose(p, t, obs)
tp = p.times(:);
if numel(tp) < 2
    pos = obs.pos;  heading = obs.heading;
    return;
end
t = min(max(t, tp(1)), tp(end));
j = find(tp <= t, 1, 'last');
if isempty(j), j = 1; end
j = min(j, numel(tp) - 1);
a = (t - tp(j)) / max(tp(j+1) - tp(j), eps);
pos = (1 - a) * p.pos(j,:) + a * p.pos(j+1,:);
hx = (1 - a) * cos(p.heading(j)) + a * cos(p.heading(j+1));
hy = (1 - a) * sin(p.heading(j)) + a * sin(p.heading(j+1));
heading = atan2(hy, hx);
end
