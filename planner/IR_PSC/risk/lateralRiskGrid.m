function [R, offsets, stationTimes] = lateralRiskGrid(corridor, preds, ego, cfg, vp, dMin, dMax)
%LATERALRISKGRID Risk of each lateral offset at each corridor station.
%
%   COMPONENT STATUS: REAL
%
%   [R, offsets, stationTimes] = LATERALRISKGRID(corridor, preds, ego, cfg,
%   vp, dMin, dMax) builds the station-by-offset risk table that the
%   trajectory deformation stage searches.
%
%   This is the bridge between "where are the hazards predicted to be" and
%   "which way should the trajectory bend". Every cell R(i,j) answers a
%   single concrete question:
%
%       If the vehicle passes station i at lateral offset offsets(j),
%       at the time it is predicted to arrive there, how dangerous is that?
%
%   Time matters and is handled explicitly. Station i is reached at
%   approximately s(i)/v seconds, so a road user predicted to cross the road
%   in two seconds only makes the stations the ego reaches at around two
%   seconds dangerous. A purely spatial obstacle map cannot express that,
%   and would brake for a hazard that will have cleared the road long before
%   the vehicle arrives.
%
%   Inputs:
%       corridor - corridor struct from extractCorridor()
%       preds    - prediction struct array from predictObstacles()
%       ego      - ego state struct from makeEgoState()
%       cfg      - config struct from irpscConfig()
%       vp       - vehicle params struct from vehicleParams()
%       dMin     - Nx1 most negative allowed offset per station
%       dMax     - Nx1 most positive allowed offset per station
%
%   Outputs:
%       R            - NxK risk matrix in [0,1]. Cells outside the allowed
%                      lateral band are set to Inf so the search can never
%                      choose them.
%       offsets      - 1xK lateral offsets sampled (m, positive = left)
%       stationTimes - Nx1 predicted arrival time at each station (s)
%
%   Example:
%       [R, off] = lateralRiskGrid(corr, preds, ego, cfg, vp, dMin, dMax);
%
%   Requires: base MATLAB only.
%
%   See also DEFORMTRAJECTORY, PREDICTEDOCCUPANCYRISK, CORRIDORBOUNDS.

N = size(corridor.center, 1);

% --- Lateral offset samples ------------------------------------------
maxShift = cfg.deform.maxLateralShift;
step     = cfg.risk.gridOffsetStep;              % m, offset resolution
offsets  = -maxShift:step:maxShift;
if isempty(offsets)
    offsets = 0;
end
K = numel(offsets);

% --- Arrival time at each station ------------------------------------
% Planning speed is the current speed, floored so a stopped vehicle still
% produces finite arrival times instead of Inf everywhere.
vPlan        = max(ego.speed, 1.0);
stationTimes = corridor.s(:) / vPlan;

tHorizon = cfg.prediction.horizon;
beyond   = stationTimes > tHorizon;
stationTimes = min(stationTimes, tHorizon);   % clamp, never extrapolate

% --- Evaluate ---------------------------------------------------------
R = zeros(N, K);

if isempty(preds)
    R = applyBounds(R, offsets, dMin, dMax);
    return;
end

% All (station, offset, disc) query points are evaluated in ONE call to
% predictedOccupancyRisk. This is numerically identical to looping over
% offsets and discs (Phase 2 change, for speed only).
sAll = repmat(corridor.s(:), K, 1);                    % (N*K)x1, offset blocks
dAll = reshape(repmat(offsets, N, 1), [], 1);          % column-major: offset j block
P    = frenetToCartesian(corridor.center, sAll, dAll); % (N*K)x2
thAll = repmat(corridor.heading(:), K, 1);
tAll  = repmat(stationTimes, K, 1);

nD = numel(vp.discOffsets);
Q  = zeros(N*K*nD, 2);
TQ = zeros(N*K*nD, 1);
for dIdx = 1:nD
    off = vp.discOffsets(dIdx);
    rows = (dIdx-1)*N*K + (1:N*K);
    Q(rows,:) = [P(:,1) + off .* cos(thAll), P(:,2) + off .* sin(thAll)];
    TQ(rows)  = tAll;
end
r = predictedOccupancyRisk(Q, TQ, preds, vp.discRadius, cfg);
r = reshape(r, N*K, nD);
R = reshape(max(r, [], 2), N, K);

% Stations reached after the prediction horizon are evaluated at the
% horizon but down-weighted: the planner may start bending early, but a
% guess about where a road user will be must not dominate (Phase 2).
R(beyond, :) = cfg.risk.beyondHorizonWeight * R(beyond, :);

R = applyBounds(R, offsets, dMin, dMax);
end

% =====================================================================
function R = applyBounds(R, offsets, dMin, dMax)
%APPLYBOUNDS Mark offsets outside the drivable corridor as forbidden.
%   Inf rather than a large finite penalty, so no weighting of the other
%   cost terms can ever buy a path off the road. If a narrow band contains
%   no sampled offset at all, the single sampled offset nearest to the
%   band centre stays allowed; the planner's clearance check still decides
%   whether the body genuinely fits there.
N = size(R,1);
for i = 1:N
    bad = offsets < dMin(i) - 1e-9 | offsets > dMax(i) + 1e-9;
    if all(bad)
        [~, j] = min(abs(offsets - 0.5 * (dMin(i) + dMax(i))));
        bad(j) = false;
    end
    R(i, bad) = Inf;
end
end
