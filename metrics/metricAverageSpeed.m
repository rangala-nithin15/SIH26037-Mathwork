function [avgSpeed, details] = metricAverageSpeed(log)
%METRICAVERAGESPEED Average speed achieved during a run.
%
%   COMPONENT STATUS: REAL
%
%   [avgSpeed, details] = METRICAVERAGESPEED(log) reports how fast the
%   vehicle actually travelled.
%
%   This metric is the necessary counterweight to every safety metric. A
%   planner can trivially achieve perfect clearance and zero collisions by
%   never moving. Average speed and scenario completion are what make such
%   a strategy visibly useless, and they must always be reported alongside
%   the safety numbers rather than separately.
%
%   Two variants are returned:
%     avgSpeed          - mean over all logged steps, including stopped time
%     details.movingAvg - mean over steps where the vehicle was moving
%   The gap between them shows how much of the run was spent halted.
%
%   Inputs:
%       log - simulation log struct with fields .egoSpeed (Nx1) and .t (Nx1)
%
%   Outputs:
%       avgSpeed - mean speed over the whole run (m/s)
%       details  - struct with .movingAvg, .maxSpeed, .stoppedFraction,
%                  .distanceCovered
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, METRICCOMPLETION.

details = struct('movingAvg', 0, 'maxSpeed', 0, 'stoppedFraction', 1, ...
                 'distanceCovered', 0);
avgSpeed = 0;

if ~isfield(log,'egoSpeed') || isempty(log.egoSpeed)
    return;
end

v = log.egoSpeed(:);
v = v(isfinite(v));
if isempty(v)
    return;
end

avgSpeed = mean(v);
details.maxSpeed = max(v);

moving = v > 0.1;
if any(moving)
    details.movingAvg = mean(v(moving));
end
details.stoppedFraction = 1 - (sum(moving) / numel(v));

if isfield(log,'egoPos') && size(log.egoPos,1) >= 2
    details.distanceCovered = sum(sqrt(sum(diff(log.egoPos,1,1).^2, 2)));
end
end
