function [longest, details] = metricLongestStop(log, cfg)
%METRICLONGESTSTOP Longest continuous standstill of the ego vehicle (stall detector).
%
%   COMPONENT STATUS: REAL
%
%   [longest, details] = METRICLONGESTSTOP(log, cfg) returns the longest
%   continuous period (s) with ego speed below 0.1 m/s, excluding the final
%   stop at the goal. A "permanent stall" -- the failure the original
%   planner showed on every scenario -- appears as a stop lasting until the
%   end of the run without reaching the goal (details.endsStopped).
%
%   Outputs:
%       longest - s
%       details - struct: .endsStopped (logical), .fractionStopped
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS.

details = struct('endsStopped', false, 'fractionStopped', 0);
longest = 0;
if ~isfield(log, 'egoSpeed') || isempty(log.egoSpeed)
    return;
end
stopped = log.egoSpeed(:) < 0.1;
dt = cfg.sim.dt;
run = 0;
for i = 1:numel(stopped)
    if stopped(i)
        run = run + 1;
        longest = max(longest, run * dt);
    else
        run = 0;
    end
end
details.fractionStopped = mean(stopped);
goal = isfield(log, 'goalReached') && logical(log.goalReached);
details.endsStopped = ~goal && numel(stopped) > 1 && stopped(end);
end
