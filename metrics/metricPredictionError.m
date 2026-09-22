function [rmse, details] = metricPredictionError(log, cfg)
%METRICPREDICTIONERROR Accuracy of short-term road-user prediction.
%
%   COMPONENT STATUS: REAL
%
%   [rmse, details] = METRICPREDICTIONERROR(log, cfg) compares what the
%   predictor said each road user would do against what the scenario ground
%   truth shows they actually did.
%
%   Method
%   ------
%   For each logged step, each prediction is looked up at a fixed horizon
%   ahead, and compared with the true position of that same road user at
%   that later time. Errors are accumulated across every road user and every
%   step, and reported as RMS displacement error in metres.
%
%   Errors are reported per horizon (1 s, 2 s and the full horizon) because
%   a single averaged number hides the shape of the failure. Prediction
%   error always grows with horizon; what matters is HOW FAST. A predictor
%   that is excellent at 1 s and hopeless at 3 s is a different engineering
%   problem from one that is mediocre throughout.
%
%   Matching (Phase 2 fix): each prediction carries .truthId, the id of the
%   ground-truth road user its track was associated with. The earlier code
%   compared TRACK ids (1, 2, 3, ... assigned by the tracker) with ACTOR ids
%   (assigned by the scenario), which only coincide by accident, so with
%   simulated sensors the metric compared unrelated road users. Predictions
%   with no truth association (false-positive tracks) are skipped, as are
%   road users that disappear mid-horizon.
%
%   Timing (Phase 2 fix): the planner runs every cfg.sim.planEvery steps
%   and its predictions are reused in between, so only steps where a new
%   plan was made (log.replanned) are evaluated, against the plan's own
%   time stamp (log.planStamp).
%
%   Inputs:
%       log - simulation log struct with fields .t, .preds, .obstacles
%             (.obstacles holds GROUND TRUTH, .preds holds predictions)
%       cfg - config struct from irpscConfig()
%
%   Outputs:
%       rmse    - RMS displacement error over all horizons (m), NaN if no
%                 comparable pair was found
%       details - struct with .rmseAt1s, .rmseAt2s, .rmseAtHorizon,
%                 .nSamples
%
%   Requires: base MATLAB only.
%
%   See also COMPUTEMETRICS, PREDICTOBSTACLES.

details = struct('rmseAt1s', NaN, 'rmseAt2s', NaN, 'rmseAtHorizon', NaN, ...
                 'nSamples', 0);
rmse = NaN;

if ~isfield(log,'preds') || ~isfield(log,'obstacles') || ~isfield(log,'t')
    return;
end

t  = log.t(:);
N  = numel(t);
H  = cfg.prediction.horizon;
horizons = [1.0, 2.0, H];

if isfield(log, 'replanned') && numel(log.replanned) == N
    evalSteps = find(log.replanned(:)).';
else
    evalSteps = 1:N;
end
if isfield(log, 'planStamp') && numel(log.planStamp) == N
    stamp = log.planStamp(:);
else
    stamp = t;
end

sqErr  = cell(1, numel(horizons));
allErr = [];
for h = 1:numel(horizons)
    sqErr{h} = [];
end

for i = evalSteps
    preds = log.preds{i};
    if isempty(preds)
        continue;
    end
    for h = 1:numel(horizons)
        hz      = horizons(h);
        tFuture = stamp(i) + hz;
        [dtBest, iFuture] = min(abs(t - tFuture));
        if dtBest > cfg.sim.dt * 2
            continue;                      % no logged truth close enough
        end
        truth = log.obstacles{iFuture};
        if isempty(truth)
            continue;
        end
        truthIds = [truth.id];
        for p = 1:numel(preds)
            pr = preds(p);
            if isfield(pr, 'truthId') && ~isempty(pr.truthId)
                tid = pr.truthId;
            else
                tid = pr.id;               % legacy logs: ground truth fed directly
            end
            if isnan(tid)
                continue;                  % false-positive track: nothing to compare
            end
            k = find(truthIds == tid, 1);
            if isempty(k)
                continue;                  % road user gone: not measurable
            end
            tp = pr.times(:);
            if hz < tp(1) || hz > tp(end)
                continue;
            end
            px = interp1(tp, pr.pos(:,1), hz, 'linear');
            py = interp1(tp, pr.pos(:,2), hz, 'linear');
            e2 = (px - truth(k).pos(1))^2 + (py - truth(k).pos(2))^2;
            sqErr{h}(end+1) = e2; %#ok<AGROW>
            allErr(end+1)   = e2; %#ok<AGROW>
        end
    end
end

if ~isempty(sqErr{1}), details.rmseAt1s      = sqrt(mean(sqErr{1})); end
if ~isempty(sqErr{2}), details.rmseAt2s      = sqrt(mean(sqErr{2})); end
if ~isempty(sqErr{3}), details.rmseAtHorizon = sqrt(mean(sqErr{3})); end

details.nSamples = numel(allErr);
if ~isempty(allErr)
    rmse = sqrt(mean(allErr));
end
end
