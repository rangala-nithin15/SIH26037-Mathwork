function [dProfile, info] = deformTrajectory(R, offsets, cfg, d0, priorProfile, extraCost)
%DEFORMTRAJECTORY Locally deform the preferred path around predicted hazards.
%
%   COMPONENT STATUS: REAL
%
%   [dProfile, info] = DEFORMTRAJECTORY(R, offsets, cfg, d0, priorProfile)
%   chooses a lateral offset at every corridor station so the resulting path
%   avoids predicted hazards while staying inside the corridor, close to its
%   centre, and smooth enough to drive.
%
%   IR-PSC steps 7, 8 and 9: deform locally around hazards, preserve
%   boundary margins, smooth the result.
%
%   Method: dynamic programming over a lateral grid
%   -----------------------------------------------
%   The cost of a whole offset profile d(1..N) is
%
%       J = sum_i [ wRisk * R(i, d_i) + wDev * d_i^2 + C(i, d_i) ]
%         + sum_i wSmooth * (d_i - d_{i-1})^2
%         + wAnchor * (d_1 - d0)^2
%
%   Because the smoothness term couples only neighbouring stations, the
%   global minimum over the discretised grid can be found exactly by
%   dynamic programming in O(N*K^2). No local minima, no gradient tuning,
%   no iteration count to guess.
%
%   Each term earns its place:
%     wRisk   - avoid predicted hazards. The point of the exercise.
%     wDev    - prefer the preferred lateral position: the corridor centre,
%               shifted by cfg.deform.preferredOffset where a traffic-side
%               convention applies (Phase 2). Without it, the path drifts to
%               whichever side is momentarily emptier and wanders.
%     wSmooth - punish sharp lateral changes, which are both uncomfortable
%               and often infeasible for the steering system.
%     wAnchor - start where the vehicle actually is. Without it the profile
%               can begin with a step the controller cannot execute.
%     C       - optional additional per-cell cost (Phase 2: pothole
%               traversal cost from potholeCostGrid). Kept SEPARATE from the
%               collision risk R so that reported collision risk is never
%               inflated by road-surface hazards.
%
%   All weights live in cfg.deform (Phase 2; previously hard-coded here).
%
%   Forbidden cells (outside the corridor) arrive as Inf in R and are simply
%   never selected. If an entire station is forbidden the problem is
%   infeasible, which is reported rather than papered over -- that is the
%   condition that makes the decision logic call a safe stop.
%
%   Inputs:
%       R            - NxK risk matrix from lateralRiskGrid(); Inf = forbidden
%       offsets      - 1xK lateral offsets corresponding to the columns (m)
%       cfg          - config struct from irpscConfig()
%       d0           - scalar, ego's current lateral offset from the
%                      corridor centre (m)
%       priorProfile - (optional) Nx1 offset of the previous plan at each
%                      CURRENT station, for temporal smoothing. NaN where
%                      the previous plan does not cover the station. Pass []
%                      on the first cycle.
%       extraCost    - (optional) NxK non-negative additional cost, already
%                      weighted by the caller (pothole cost, static clearance
%                      cost). Inf marks a forbidden cell.
%
%   Outputs:
%       dProfile - Nx1 chosen lateral offset at each station (m)
%       info     - struct with fields:
%                  .feasible    logical, false if no admissible profile exists
%                  .cost        scalar total cost of the chosen profile
%                  .maxRisk     peak risk along the chosen profile
%                  .meanRisk    mean risk along the chosen profile
%                  .maxShift    largest absolute offset used (m)
%                  .blockedRows indices of stations with no admissible offset
%
%   If cfg.ablation.useDeformation is false the centreline is returned
%   unchanged. That is the "no deformation" ablation.
%
%   Example:
%       [d, info] = deformTrajectory(R, offsets, cfg, 0.2, prevD);
%
%   Requires: base MATLAB only.
%
%   See also LATERALRISKGRID, CORRIDORBOUNDS, GENERATETRAJECTORY.

if nargin < 5, priorProfile = []; end
if nargin < 6, extraCost    = []; end

[N, K] = size(R);
info = struct('feasible', true, 'cost', 0, 'maxRisk', 0, 'meanRisk', 0, ...
              'maxShift', 0, 'blockedRows', []);

% --- Infeasibility check ---------------------------------------------
if ~isempty(extraCost) && isequal(size(extraCost), [N K])
    blocked = find(all(~isfinite(R) | ~isfinite(extraCost), 2));
else
    blocked = find(all(~isfinite(R), 2));
end
if ~isempty(blocked)
    info.feasible    = false;
    info.blockedRows = blocked(:).';
    dProfile         = zeros(N,1);
    info.cost        = Inf;
    info.maxRisk     = 1;
    return;
end

% --- Ablation: skip deformation entirely -----------------------------
if isfield(cfg,'ablation') && ~cfg.ablation.useDeformation
    dProfile = zeros(N,1);
    % Report the risk actually incurred by NOT deforming, so the ablation
    % comparison is meaningful.
    [~, jZero] = min(abs(offsets));
    r = R(:, jZero);
    r(~isfinite(r)) = 1;
    info.maxRisk  = max(r);
    info.meanRisk = mean(r);
    info.cost     = sum(r);
    info.preferred = zeros(N,1);
    return;
end

% --- Cost weights ------------------------------------------------------
wRisk   = cfg.deform.wRisk * cfg.deform.gain;
wDev    = cfg.deform.wDev;
wSmooth = cfg.deform.wSmooth;
wAnchor = cfg.deform.wAnchor;

% --- Stage costs -------------------------------------------------------
% Preferred offset per station: cfg.deform.preferredOffset clipped to the
% admissible (finite) columns of that station.
pref = zeros(N,1);
if isfield(cfg.deform, 'preferredOffset') && cfg.deform.preferredOffset ~= 0
    for i = 1:N
        ok = isfinite(R(i,:));
        if any(ok)
            pref(i) = min(max(cfg.deform.preferredOffset, min(offsets(ok))), max(offsets(ok)));
        end
    end
end
stage = wRisk * R + wDev * (repmat(offsets(:).', N, 1) - repmat(pref, 1, K)).^2;
if ~isempty(extraCost) && isequal(size(extraCost), [N K])
    stage = stage + extraCost;      % already weighted by the caller; Inf = forbidden
end

% Anchor the first station to where the vehicle actually is.
stage(1,:) = stage(1,:) + wAnchor * (offsets - d0).^2;

% Temporal smoothing: bias toward the previous cycle's answer so the path
% does not flip between two equally good sides on successive frames.
if ~isempty(priorProfile) && numel(priorProfile) == N && ...
        (~isfield(cfg,'ablation') || cfg.ablation.useTemporalSmoothing)
    wTemporal = cfg.deform.wTemporal * (1 - cfg.deform.temporalAlpha);
    for i = 1:N
        if isfinite(priorProfile(i))
            stage(i,:) = stage(i,:) + wTemporal * (offsets - priorProfile(i)).^2;
        end
    end
end

% --- Transition cost ---------------------------------------------------
% trans(a,b) = wSmooth * (offsets(b) - offsets(a))^2
dOff  = offsets(:) - offsets(:).';          % KxK, dOff(a,b) = off(a) - off(b)
trans = wSmooth * (dOff.').^2;              % trans(a,b) as described above

% --- Forward DP --------------------------------------------------------
J    = inf(N, K);
back = zeros(N, K);
J(1,:) = stage(1,:);

for i = 2:N
    % candidate(a,b) = J(i-1,a) + trans(a,b)
    candidate = J(i-1,:).' + trans;
    [bestPrev, argPrev] = min(candidate, [], 1);
    J(i,:)    = stage(i,:) + bestPrev;
    back(i,:) = argPrev;
end

% --- Backtrack ---------------------------------------------------------
[totalCost, jEnd] = min(J(N,:));
if ~isfinite(totalCost)
    info.feasible = false;
    dProfile      = zeros(N,1);
    info.cost     = Inf;
    info.maxRisk  = 1;
    return;
end

idx    = zeros(N,1);
idx(N) = jEnd;
for i = N:-1:2
    idx(i-1) = back(i, idx(i));
end

dProfile = offsets(idx).';
dProfile = dProfile(:);

% --- Report ------------------------------------------------------------
chosenRisk = zeros(N,1);
for i = 1:N
    chosenRisk(i) = R(i, idx(i));
end

info.cost     = totalCost;
info.maxRisk  = max(chosenRisk);
info.meanRisk = mean(chosenRisk);
info.maxShift = max(abs(dProfile));
info.preferred = pref;
end
