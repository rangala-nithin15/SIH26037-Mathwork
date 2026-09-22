function [totalRisk, perSample, worstId] = conflictRisk(traj, preds, cfg, vp)
%CONFLICTRISK Aggregate collision risk along a candidate ego trajectory.
%
%   COMPONENT STATUS: REAL
%
%   [totalRisk, perSample, worstId] = CONFLICTRISK(traj, preds, cfg, vp)
%   scores a proposed ego trajectory against the predicted occupancy of
%   every tracked road user.
%
%   IR-PSC step 6, evaluated on the trajectory the planner actually intends
%   to drive (as opposed to TIMETOCONFLICT, which evaluates the do-nothing
%   trajectory).
%
%   The ego footprint is represented by the three covering discs from
%   VEHICLEPARAMS rather than a single point, so a long vehicle body
%   swinging past an obstacle is scored correctly rather than only its rear
%   axle being checked.
%
%   The trajectory total is the MAXIMUM per-sample risk, not the mean.
%   Averaging would let a long safe trajectory hide one lethal instant,
%   which is exactly the failure mode a safety score must not have.
%
%   Inputs:
%       traj  - trajectory struct with fields:
%               .pos     Nx2 positions
%               .times   Nx1 times from the planning instant (s)
%               .heading Nx1 headings (rad)
%       preds - prediction struct array from predictObstacles()
%       cfg   - config struct from irpscConfig()
%       vp    - vehicle params struct from vehicleParams()
%
%   Outputs:
%       totalRisk - scalar in [0,1], the peak risk over the trajectory
%       perSample - Nx1 risk at each trajectory sample
%       worstId   - track id of the largest contributor at the peak, or NaN
%
%   Example:
%       [R, rs] = conflictRisk(traj, preds, cfg, vp);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTEDOCCUPANCYRISK, SCORETRAJECTORY, TIMETOCONFLICT.

N = size(traj.pos, 1);
perSample = zeros(N,1);
worstId   = NaN;
totalRisk = 0;

if isempty(preds) || N == 0
    return;
end

% Phase 2: only samples the vehicle actually reaches, and only within the
% prediction horizon. Evaluating a sample reached at t = 6 s against where
% a road user is predicted to be at the 3.5 s horizon is not a prediction,
% it is a guess that made every slow lead vehicle look like a certain
% collision. Beyond-horizon samples are re-evaluated on later cycles.
M = N;
if isfield(traj, 'reachable') && numel(traj.reachable) == M
    reach = logical(traj.reachable(:));
else
    reach = true(M,1);
end
th = traj.heading(:);
t  = traj.times(:);
use = reach & t <= cfg.prediction.horizon;

pos = traj.pos(use,:);
hdg = th(use);
tq  = t(use);

% A vehicle that comes to rest stays where it stopped: evaluate the resting
% pose for the remainder of the horizon as well, so something moving INTO
% the stopped vehicle is still scored.
lastR = find(reach, 1, 'last');
nRest = 0;
if ~isempty(lastR) && traj.speed(lastR) <= 0.05 && t(lastR) < cfg.prediction.horizon
    tRest = (t(lastR):cfg.prediction.dt:cfg.prediction.horizon).';
    nRest = numel(tRest);
    pos = [pos; repmat(traj.pos(lastR,:), nRest, 1)];
    hdg = [hdg; repmat(th(lastR), nRest, 1)];
    tq  = [tq; tRest];
end

if isempty(tq)
    return;
end

Q = numel(tq);
perQ   = zeros(Q,1);
worstQ = nan(Q,1);
for dIdx = 1:numel(vp.discOffsets)
    off = vp.discOffsets(dIdx);
    discPos = [pos(:,1) + off .* cos(hdg), pos(:,2) + off .* sin(hdg)];
    [r, wid] = predictedOccupancyRisk(discPos, tq, preds, vp.discRadius, cfg);
    better = r > perQ;
    perQ(better)   = r(better);
    worstQ(better) = wid(better);
end

idxUse = find(use);
perSample(idxUse) = perQ(1:numel(idxUse));
sampleWorst = nan(N,1);
sampleWorst(idxUse) = worstQ(1:numel(idxUse));
if nRest > 0
    restRisk = max(perQ(numel(idxUse)+1:end));
    if restRisk > perSample(lastR)
        perSample(lastR) = restRisk;
        [~, jr] = max(perQ(numel(idxUse)+1:end));
        sampleWorst(lastR) = worstQ(numel(idxUse) + jr);
    end
end

[totalRisk, iPeak] = max(perSample);
if isfinite(iPeak) && iPeak >= 1
    worstId = sampleWorst(iPeak);
end
end
