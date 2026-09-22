function [n, details] = metricPlannerFailures(log)
%METRICPLANNERFAILURES Count of planning cycles that failed, by cause.
%
%   COMPONENT STATUS: REAL
%
%   [n, details] = METRICPLANNERFAILURES(log) counts planning cycles that
%   did not return a normal trajectory, broken down by the reason.
%
%   The breakdown matters more than the total, because the causes call for
%   completely different fixes:
%
%     no_corridor             Perception found no drivable space. A
%                             perception or occupancy problem, not a
%                             planning one.
%     deformation_infeasible  A corridor existed but every lateral offset
%                             was blocked. A genuine planning limitation.
%     clearance_failed        A path was produced but the body would not
%                             fit. Points at margins or vehicle geometry.
%     feasibility_failed      A path was produced but the vehicle could not
%                             drive it. Points at the trajectory generator.
%     no_feasible_candidate   Baseline planner only: none of its five fixed
%                             offsets worked. This is the characteristic
%                             fixed-candidate failure and is the single most
%                             informative number in the baseline comparison.
%
%   A total alone would say "the planner failed 14 times" without
%   distinguishing a perception blackout from a planner limitation, which
%   would make the result useless for deciding what to improve.
%
%   Inputs:
%       log - simulation log struct with field .status (Nx1 cellstr)
%
%   Outputs:
%       n       - total number of failed planning cycles
%       details - struct with one counter field per cause, plus .causes
%                 (cellstr of causes seen) and .failureRate
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, IRPSCPLANNER, BASELINEPLANNER.

details = struct('no_corridor', 0, 'deformation_infeasible', 0, ...
                 'clearance_failed', 0, 'feasibility_failed', 0, ...
                 'no_feasible_candidate', 0, 'other', 0, ...
                 'causes', {{}}, 'failureRate', 0);
n = 0;

if ~isfield(log,'status') || isempty(log.status)
    return;
end

known = {'no_corridor','deformation_infeasible','clearance_failed', ...
         'feasibility_failed','no_feasible_candidate'};
seen  = {};

for i = 1:numel(log.status)
    s = log.status{i};
    if isempty(s) || strcmp(s, 'ok')
        continue;
    end
    n = n + 1;
    if ismember(s, known)
        details.(s) = details.(s) + 1;
    else
        details.other = details.other + 1;
    end
    if ~ismember(s, seen)
        seen{end+1} = s; %#ok<AGROW>
    end
end

details.causes      = seen;
details.failureRate = n / max(numel(log.status), 1);
end
