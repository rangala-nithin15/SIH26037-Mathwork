function [minClear, meanClear] = metricClearance(log)
%METRICCLEARANCE Minimum and mean obstacle clearance over a run.
%
%   COMPONENT STATUS: REAL
%
%   [minClear, meanClear] = METRICCLEARANCE(log) reports the tightest and
%   the average clearance the ego vehicle body had during the run.
%
%   Both are reported because they answer different questions. The minimum
%   is the safety-relevant one: it is the single worst moment, and an
%   otherwise excellent run with one near-miss is not a good run. The mean
%   describes comfort and confidence -- how much room the vehicle generally
%   left itself.
%
%   Negative values indicate the vehicle body overlapped an obstacle.
%
%   Inputs:
%       log - simulation log struct with field .minClear (Nx1, metres)
%
%   Outputs:
%       minClear  - smallest clearance recorded (m), Inf if unavailable
%       meanClear - mean clearance across the run (m), Inf if unavailable
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, CHECKCLEARANCE.

if ~isfield(log, 'minClear') || isempty(log.minClear)
    minClear  = Inf;
    meanClear = Inf;
    return;
end

c = log.minClear(:);
c = c(isfinite(c));

if isempty(c)
    minClear  = Inf;
    meanClear = Inf;
else
    minClear  = min(c);
    meanClear = mean(c);
end
end
