function M = computeMetrics(log, cfg)
%COMPUTEMETRICS Compute every evaluation metric from one simulation log.
%
%   COMPONENT STATUS: REAL
%
%   M = COMPUTEMETRICS(log, cfg) computes the full metric set for a single
%   scenario run.
%
%   ============================================================
%   NO VALUE IN THIS FILE IS INVENTED
%   ============================================================
%   Every number returned is computed from the log passed in. If a run has
%   not been executed there is no log, and therefore no metrics. This
%   function cannot produce a result without one, by construction. Nothing
%   in this project ships with pre-filled metric values.
%
%   ============================================================
%   HOW TO READ THESE NUMBERS
%   ============================================================
%   They describe behaviour IN SIMULATION, against a simplified sensor
%   model and a kinematic vehicle model. They are valid for comparing
%   planners against each other under identical conditions, which is what
%   the baseline and ablation studies do. They are NOT statements about
%   real-world safety, and a collision count of zero in simulation must
%   never be reported as evidence of real-world collision avoidance.
%
%   Inputs:
%       log - simulation log struct from runScenario(), with fields:
%             .t          Nx1 time stamps (s)
%             .egoPos     Nx2 ego positions
%             .egoSpeed   Nx1 ego speed
%             .minClear   Nx1 minimum clearance at each step
%             .planTime   Nx1 planner elapsed time per replan (s), NaN when
%                         no replan occurred on that step
%             .state      Nx1 cellstr of decision states
%             .status     Nx1 cellstr of planner statuses
%             .collided   Nx1 logical
%             .traj       1xN cell of the trajectory planned at each step
%             .goalReached logical scalar
%             .obstacles  1xN cell of obstacle arrays (truth, per step)
%             .preds      1xN cell of prediction arrays
%       cfg - config struct from irpscConfig()
%
%   Outputs:
%       M - struct with fields:
%           .collisionCount, .collisionRate
%           .minClearance, .meanClearance
%           .completed
%           .meanLatency, .maxLatency, .p95Latency
%           .smoothness
%           .predictionError
%           .lateralTrackingError
%           .emergencyStops
%           .averageSpeed
%           .plannerFailures
%           .duration, .distance
%           .provenance - struct recording how these numbers were produced
%
%   Example:
%       M = computeMetrics(log, cfg);
%
%   Requires: base MATLAB only.
%
%   See also RUNSCENARIO, AGGREGATEMETRICS.

if ~isstruct(log) || ~isfield(log, 't') || isempty(log.t)
    error('computeMetrics:emptyLog', ...
          ['No simulation log supplied. Metrics can only be computed from ' ...
           'an executed run -- this project never fabricates metric values.']);
end

M.collisionCount = metricCollisionCount(log);
% Phase 2: static contacts (grid) and dynamic contacts (road users) separately.
if isfield(log, 'collidedStatic')
    M.staticContactSteps  = sum(log.collidedStatic);
    M.dynamicContactSteps = sum(log.collidedDynamic);
else
    M.staticContactSteps  = NaN;
    M.dynamicContactSteps = NaN;
end
[M.longestStop, M.stopDetails] = metricLongestStop(log, cfg);
M.pothole = metricPotholes(log);
if isfield(log, 'emergencyBrake')
    M.emergencyBrakeSteps = sum(log.emergencyBrake);
else
    M.emergencyBrakeSteps = NaN;
end
M.collisionRate  = M.collisionCount / max(numel(log.t), 1);

[M.minClearance, M.meanClearance] = metricClearance(log);

M.completed = metricCompletion(log);

[M.meanLatency, M.maxLatency, M.p95Latency] = metricReplanLatency(log);

M.smoothness = metricSmoothness(log);

M.predictionError = metricPredictionError(log, cfg);

M.lateralTrackingError = metricLateralTrackingError(log);

M.emergencyStops = metricEmergencyStops(log);

M.averageSpeed = metricAverageSpeed(log);

M.plannerFailures = metricPlannerFailures(log);

% --- Run summary --------------------------------------------------------
M.duration = log.t(end) - log.t(1);
if size(log.egoPos,1) >= 2
    M.distance = sum(sqrt(sum(diff(log.egoPos,1,1).^2, 2)));
else
    M.distance = 0;
end

% --- Provenance ---------------------------------------------------------
% Recorded with every metric set so a result can never be separated from
% the conditions that produced it.
M.provenance.source        = 'simulation';
M.provenance.sensorModel   = 'SIMPLIFIED geometric (simulateDetections)';
M.provenance.vehicleModel  = 'SIMPLIFIED kinematic bicycle (bicycleModelStep)';
M.provenance.profile       = cfg.profile;
M.provenance.dt            = cfg.sim.dt;
M.provenance.nSteps        = numel(log.t);
M.provenance.realWorldClaim = false;
M.provenance.note = ['Simulation result under a simplified sensor and ' ...
                     'vehicle model. Not a real-world safety measurement.'];
end
