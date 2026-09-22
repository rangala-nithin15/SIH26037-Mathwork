function [rmsErr, details] = metricLateralTrackingError(log)
%METRICLATERALTRACKINGERROR How well the controller followed the plan.
%
%   COMPONENT STATUS: REAL
%
%   [rmsErr, details] = METRICLATERALTRACKINGERROR(log) measures the
%   perpendicular distance between where the vehicle actually was and the
%   trajectory that had been planned for it.
%
%   Why this metric is not optional
%   -------------------------------
%   Every safety check in this project -- clearance, feasibility, risk -- is
%   performed on the PLANNED trajectory. If the vehicle does not actually
%   follow that trajectory, those checks were performed on a path that was
%   never driven, and their guarantees do not transfer to reality. A large
%   tracking error therefore invalidates the planner's safety reasoning, no
%   matter how good the planner is.
%
%   This is the number that tells you whether the rest of the evaluation can
%   be believed.
%
%   Inputs:
%       log - simulation log struct with fields .egoPos (Nx2) and .traj
%             (1xN cell of the trajectory planned at each step)
%
%   Outputs:
%       rmsErr  - RMS lateral tracking error (m), NaN if not computable
%       details - struct with .maxError, .meanAbsError, .nSamples
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, PUREPURSUITCONTROL, PROJECTPOINTONPATH.

details = struct('maxError', NaN, 'meanAbsError', NaN, 'nSamples', 0);
rmsErr  = NaN;

if ~isfield(log,'egoPos') || ~isfield(log,'traj') || isempty(log.traj)
    return;
end

N = min(size(log.egoPos,1), numel(log.traj));
errs = [];

for i = 1:N
    tr = log.traj{i};
    if isempty(tr) || ~isstruct(tr) || ~isfield(tr,'pos') || size(tr.pos,1) < 2
        continue;
    end
    [~, d] = projectPointOnPath(tr.pos, log.egoPos(i,:));
    errs(end+1) = d; %#ok<AGROW>
end

if isempty(errs)
    return;
end

details.maxError     = max(abs(errs));
details.meanAbsError = mean(abs(errs));
details.nSamples     = numel(errs);
rmsErr = sqrt(mean(errs.^2));
end
