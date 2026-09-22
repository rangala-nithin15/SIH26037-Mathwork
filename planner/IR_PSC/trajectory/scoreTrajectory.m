function [score, terms] = scoreTrajectory(traj, corridor, preds, cfg, vp)
%SCORETRAJECTORY Single comparable quality score for a candidate trajectory.
%
%   COMPONENT STATUS: REAL
%
%   [score, terms] = SCORETRAJECTORY(traj, corridor, preds, cfg, vp) reduces
%   a trajectory to one number so that alternatives -- IR-PSC output, safe
%   stop, and every baseline candidate -- can be compared on identical
%   terms.
%
%   IR-PSC step 10 (score) feeding step 11 (return the safest feasible
%   trajectory).
%
%   LOWER IS BETTER. The score is a weighted sum of penalties:
%
%       risk       (w 10.0) - peak predicted conflict risk along the path
%       deviation  (w  1.0) - mean squared lateral offset from the corridor
%                             centre; keeps the vehicle road-following
%       curvature  (w  2.0) - mean squared curvature; comfort and feasibility
%       jerkiness  (w  1.5) - variation of curvature along the path; a path
%                             that wobbles scores worse than one that bends
%                             once, even at equal peak curvature
%       progress   (w  3.0) - penalty for a short or slow path; without this
%                             term, stopping dead always wins, since a
%                             stationary vehicle has zero risk
%
%   The progress term is what stops the planner being trivially degenerate.
%   It is the counterweight that forces the risk term to be earned rather
%   than avoided.
%
%   Inputs:
%       traj     - trajectory struct from generateTrajectory()
%       corridor - corridor struct from extractCorridor()
%       preds    - prediction struct array from predictObstacles()
%       cfg      - config struct from irpscConfig()
%       vp       - vehicle params struct from vehicleParams()
%
%   Outputs:
%       score - scalar, lower is better. Inf for an invalid trajectory.
%       terms - struct with each weighted contribution and its raw value,
%               so a demo or report can show why one path beat another
%
%   Example:
%       [s, t] = scoreTrajectory(traj, corr, preds, cfg, vp);
%
%   Requires: base MATLAB only.
%
%   See also CONFLICTRISK, IRPSCPLANNER, BASELINEPLANNER.

w = struct('risk', cfg.score.wRisk, 'deviation', cfg.score.wDeviation, ...
           'curvature', cfg.score.wCurvature, 'jerkiness', cfg.score.wJerkiness, ...
           'progress', cfg.score.wProgress);   % weights: cfg.score (Phase 2)

if ~isfield(traj,'valid') || ~traj.valid || size(traj.pos,1) < 2
    score = Inf;
    terms = struct('risk',Inf,'deviation',Inf,'curvature',Inf, ...
                   'jerkiness',Inf,'progress',Inf,'weights',w,'raw',struct());
    return;
end

% --- Risk ---------------------------------------------------------------
rawRisk = conflictRisk(traj, preds, cfg, vp);

% --- Deviation from the corridor centre ---------------------------------
if ~isempty(corridor) && isfield(corridor,'center') && size(corridor.center,1) >= 2
    N = size(traj.pos,1);
    dLat = zeros(N,1);
    for i = 1:N
        [~, dLat(i)] = projectPointOnPath(corridor.center, traj.pos(i,:));
    end
    rawDev = mean(dLat.^2);
else
    rawDev = 0;
end

% --- Curvature and its variation ----------------------------------------
k = traj.curvature(:);
rawCurv = mean(k.^2);
if numel(k) >= 2
    rawJerk = mean(diff(k).^2);
else
    rawJerk = 0;
end

% --- Progress -----------------------------------------------------------
% Normalised shortfall against what the vehicle could ideally achieve over
% the planning horizon at its configured maximum speed.
idealDist = cfg.ego.maxSpeed * cfg.prediction.horizon;
actual    = traj.s(end);
if isfield(traj, 'stopS') && isfinite(traj.stopS)
    actual = min(actual, traj.stopS);     % distance actually covered before stopping
end
rawProg   = max(0, 1 - actual / max(idealDist, eps));

meanSpeed  = mean(traj.speed);
speedShort = max(0, 1 - meanSpeed / max(cfg.ego.maxSpeed, eps));
rawProg    = 0.5 * rawProg + 0.5 * speedShort;

% --- Combine ------------------------------------------------------------
terms.risk      = w.risk      * rawRisk;
terms.deviation = w.deviation * rawDev;
terms.curvature = w.curvature * rawCurv;
terms.jerkiness = w.jerkiness * rawJerk;
terms.progress  = w.progress  * rawProg;
terms.weights   = w;
terms.raw       = struct('risk', rawRisk, 'deviation', rawDev, ...
                         'curvature', rawCurv, 'jerkiness', rawJerk, ...
                         'progress', rawProg);

score = terms.risk + terms.deviation + terms.curvature + ...
        terms.jerkiness + terms.progress;
end
