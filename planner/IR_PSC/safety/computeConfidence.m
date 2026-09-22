function [conf, breakdown] = computeConfidence(corridor, preds, obstacles, cfg)
%COMPUTECONFIDENCE How much the planner should trust its own inputs.
%
%   COMPONENT STATUS: REAL
%
%   [conf, breakdown] = COMPUTECONFIDENCE(corridor, preds, obstacles, cfg)
%   returns a single scalar in [0,1] summarising how well-founded the
%   current planning problem is.
%
%   IR-PSC steps 11 and 12: confidence-aware planning, and reduced autonomy
%   when confidence is insufficient.
%
%   This is the piece that turns "we know we might be wrong" into behaviour.
%   Rather than planning at full speed on bad data and hoping, the system
%   measures the quality of its own inputs and hands that number to the
%   decision logic, which slows down or stops.
%
%   Four contributions, each genuinely measurable from data the system
%   already has:
%
%     corridorScore   (weight 0.35) - corridor.quality: how much of the
%                     requested lookahead was achieved, how much of it is
%                     wide enough, and how many boundary rays actually found
%                     an edge rather than running to maximum range.
%
%     perceptionScore (weight 0.25) - mean detection/track confidence of the
%                     road users currently being tracked.
%
%     trackScore      (weight 0.20) - track maturity. A scene full of tracks
%                     one frame old is a scene we do not yet understand.
%
%     predictScore    (weight 0.20) - inverse of mean predicted uncertainty
%                     at the end of the horizon, normalised by
%                     cfg.prediction.maxSigma. Vague predictions lower
%                     confidence.
%
%   HONESTY NOTE: this is a designed heuristic, not a calibrated probability.
%   It has not been validated against outcomes, and a confidence of 0.8 does
%   NOT mean "80% chance of being right". It is an internal, monotonic
%   quality signal: higher means better-founded. Calibrating it against
%   recorded outcomes is future work.
%
%   Inputs:
%       corridor  - corridor struct from extractCorridor()
%       preds     - prediction struct array from predictObstacles()
%       obstacles - obstacle struct array (current tracks)
%       cfg       - config struct from irpscConfig()
%
%   Outputs:
%       conf      - scalar in [0,1]
%       breakdown - struct with the four component scores and their weights,
%                   so a demo can show WHY confidence dropped
%
%   If cfg.ablation.useConfidenceAware is false, conf is forced to 1.0.
%   That is the "no confidence-aware behaviour" ablation.
%
%   Example:
%       [c, b] = computeConfidence(corr, preds, obs, cfg);
%
%   Requires: base MATLAB only.
%
%   See also EXTRACTCORRIDOR, DECISIONLOGIC, IRPSCPLANNER.

% --- Corridor quality ---------------------------------------------------
if isempty(corridor) || ~isfield(corridor,'quality')
    corridorScore = 0;
else
    corridorScore = max(0, min(1, corridor.quality));
end

% --- Perception confidence ---------------------------------------------
if isempty(obstacles)
    % No road users detected. That is not evidence of good perception, but
    % it is also not evidence of bad perception. A neutral value is used
    % rather than 1.0, so an empty scene does not masquerade as certainty.
    perceptionScore = cfg.confidence.emptySceneScore;
    trackScore      = cfg.confidence.emptySceneScore;
else
    confs = zeros(1, numel(obstacles));
    ages  = zeros(1, numel(obstacles));
    for i = 1:numel(obstacles)
        confs(i) = max(0, min(1, obstacles(i).confidence));
        ages(i)  = max(0, obstacles(i).age);
    end
    perceptionScore = mean(confs);
    % Track maturity saturates at about 10 frames.
    trackScore = mean(min(ages / cfg.confidence.matureAge, 1));
end

% --- Prediction sharpness ----------------------------------------------
if isempty(preds)
    predictScore = cfg.confidence.emptySceneScore;
else
    endSigma = zeros(1, numel(preds));
    for i = 1:numel(preds)
        sL = preds(i).sigmaLong;
        sT = preds(i).sigmaLat;
        if isempty(sL)
            endSigma(i) = cfg.prediction.maxSigma;
        else
            endSigma(i) = 0.5 * (sL(end) + sT(end));
        end
    end
    predictScore = 1 - min(mean(endSigma) / max(cfg.prediction.maxSigma, eps), 1);
end

% --- Combine ------------------------------------------------------------
w = struct('corridor', cfg.confidence.wCorridor, 'perception', cfg.confidence.wPerception, ...
           'track', cfg.confidence.wTrack, 'predict', cfg.confidence.wPredict);  % cfg.confidence (Phase 2)

conf = w.corridor   * corridorScore + ...
       w.perception * perceptionScore + ...
       w.track      * trackScore + ...
       w.predict    * predictScore;

conf = max(0, min(1, conf));

breakdown.corridorScore   = corridorScore;
breakdown.perceptionScore = perceptionScore;
breakdown.trackScore      = trackScore;
breakdown.predictScore    = predictScore;
breakdown.weights         = w;
breakdown.ablated         = false;

if isfield(cfg,'ablation') && ~cfg.ablation.useConfidenceAware
    breakdown.rawConfidence = conf;
    breakdown.ablated       = true;
    conf = 1.0;
end
end
