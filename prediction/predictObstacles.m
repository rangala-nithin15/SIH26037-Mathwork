function preds = predictObstacles(obstacles, cfg, corridor)
%PREDICTOBSTACLES Short-term prediction of every tracked road user.
%
%   COMPONENT STATUS: REAL (orchestration) over SIMPLIFIED sub-models
%
%   preds = PREDICTOBSTACLES(obstacles, cfg, corridor) predicts where every
%   tracked road user will be over the configured horizon, together with how
%   uncertain each prediction is.
%
%   IR-PSC steps 4 and 5: predict short-term future occupancy, and represent
%   uncertainty.
%
%   For each obstacle this combines three pieces:
%     1. PREDICTCONSTANTVELOCITY  - the interpretable mean motion
%     2. PREDICTIONUNCERTAINTY    - how vague that mean becomes over time
%     3. IRREGULARMOTIONMODEL     - alternative manoeuvres a lane-following
%                                   model would miss entirely
%
%   The irregular hypotheses are folded back into the uncertainty as extra
%   lateral spread, using the spread of the hypothesis endpoints. That keeps
%   the downstream risk computation working with a single Gaussian-shaped
%   occupancy per time step, which is far cheaper than carrying every
%   hypothesis through the planner, while still widening the danger zone
%   when a road user might realistically swerve.
%
%   Inputs:
%       obstacles - 1xM struct array from makeObstacle(); may be empty
%       cfg       - config struct from irpscConfig()
%       corridor  - corridor struct from extractCorridor(), or [] if none
%
%   Outputs:
%       preds - 1xM struct array with fields:
%           .id          track id, copied from the obstacle
%           .class       class name
%           .times       1xT prediction times (s)
%           .pos         Tx2 predicted mean positions
%           .vel         Tx2 predicted mean velocities
%           .sigmaLong   Tx1 along-track std dev (m)
%           .sigmaLat    Tx1 cross-track std dev (m)
%           .heading     Tx1 predicted heading (rad)
%           .radius      scalar, obstacle footprint radius (m)
%           .weight      scalar risk weight for this class
%           .confidence  copied track confidence
%           .hypotheses  struct array from irregularMotionModel()
%
%   If cfg.ablation.usePrediction is false, every obstacle is predicted as
%   frozen at its current position. That is the "no prediction" ablation.
%
%   Example:
%       preds = predictObstacles(obs, cfg, corridor);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTCONSTANTVELOCITY, PREDICTIONUNCERTAINTY,
%            IRREGULARMOTIONMODEL, CONFLICTRISK.

if nargin < 3, corridor = []; end

if isempty(obstacles)
    preds = struct('id',{},'class',{},'times',{},'pos',{},'vel',{}, ...
                   'sigmaLong',{},'sigmaLat',{},'heading',{},'radius',{}, ...
                   'weight',{},'confidence',{},'hypotheses',{}, ...
                   'halfLength',{},'halfWidth',{},'truthId',{});
    return;
end

pc    = cfg.prediction;
times = 0:pc.dt:pc.horizon;
T     = numel(times);
M     = numel(obstacles);

preds = repmat(struct('id',[],'class','','times',[],'pos',[],'vel',[], ...
                      'sigmaLong',[],'sigmaLat',[],'heading',[],'radius',[], ...
                      'weight',[],'confidence',[],'hypotheses',[], ...
                      'halfLength',[],'halfWidth',[],'truthId',[]), 1, M);

usePred = ~isfield(cfg,'ablation') || cfg.ablation.usePrediction;

for m = 1:M
    o = obstacles(m);

    if usePred
        [meanPos, meanVel] = predictConstantVelocity(o, times);
        [sL, sT]           = predictionUncertainty(o, times, cfg);
        hyp                = irregularMotionModel(o, times, cfg, corridor);

        % Fold hypothesis spread into lateral uncertainty. Spread is measured
        % as the weighted RMS deviation of the hypothesis endpoints from the
        % straight-line mean at each time step.
        if numel(hyp) > 1
            spread = zeros(T,1);
            for h = 1:numel(hyp)
                dev    = hyp(h).pos - meanPos;
                dist   = sqrt(sum(dev.^2, 2));
                spread = spread + hyp(h).weight * dist.^2;
            end
            sT = sqrt(sT.^2 + spread);
            sT = min(sT, pc.maxSigma);
        end
    else
        % ABLATION: no prediction. The obstacle is assumed frozen.
        meanPos = repmat(o.pos(:).', T, 1);
        meanVel = zeros(T,2);
        sL      = repmat(pc.posNoiseStd, T, 1);
        sT      = repmat(pc.posNoiseStd, T, 1);
        hyp     = struct('name',{'frozen'},'weight',{1},'pos',{meanPos});
    end

    heading = repmat(o.heading, T, 1);
    moving  = sqrt(sum(meanVel.^2, 2)) > 0.1;
    if any(moving)
        heading(moving) = atan2(meanVel(moving,2), meanVel(moving,1));
    end

    preds(m).id         = o.id;
    preds(m).class      = o.class;
    preds(m).times      = times;
    preds(m).pos        = meanPos;
    preds(m).vel        = meanVel;
    preds(m).sigmaLong  = sL;
    preds(m).sigmaLat   = sT;
    preds(m).heading    = heading;
    preds(m).radius     = hypot(o.length, o.width) / 2;
    preds(m).weight     = classRiskWeight(o.class, cfg);
    preds(m).confidence = o.confidence;
    preds(m).hypotheses = hyp;
    preds(m).halfLength = o.length / 2;
    preds(m).halfWidth  = o.width / 2;
    if isfield(o, 'truthId')
        preds(m).truthId = o.truthId;   % evaluation only
    else
        preds(m).truthId = NaN;
    end
end
end

% =====================================================================
function w = classRiskWeight(className, cfg)
%CLASSRISKWEIGHT Look up the per-class risk weight, defaulting to unknown.
if isfield(cfg.risk.classWeight, className)
    w = cfg.risk.classWeight.(className);
else
    w = cfg.risk.classWeight.unknown;
end
end
