function completed = metricCompletion(log)
%METRICCOMPLETION Did the run reach its goal without a terminal failure?
%
%   COMPONENT STATUS: REAL
%
%   completed = METRICCOMPLETION(log) returns true only if the ego vehicle
%   reached the scenario goal AND did not collide.
%
%   Scenario completion is deliberately strict, because the alternative is
%   worse. A planner that avoids every hazard by stopping permanently is
%   perfectly safe and completely useless; requiring the goal to be reached
%   is what stops that degenerate strategy from scoring well. Requiring no
%   collision stops the opposite failure, where a planner reaches the goal
%   by driving through something.
%
%   Across a set of runs, the mean of this value is the scenario completion
%   RATE.
%
%   Inputs:
%       log - simulation log struct with fields .goalReached and .collided
%
%   Outputs:
%       completed - logical scalar
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, AGGREGATEMETRICS.

goalReached = isfield(log, 'goalReached') && ~isempty(log.goalReached) && ...
              logical(log.goalReached);

collided = isfield(log, 'collided') && any(logical(log.collided));

completed = goalReached && ~collided;
end
