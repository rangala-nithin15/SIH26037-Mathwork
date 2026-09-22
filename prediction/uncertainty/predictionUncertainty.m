function [sigmaLong, sigmaLat] = predictionUncertainty(obs, times, cfg)
%PREDICTIONUNCERTAINTY Growth of prediction uncertainty over the horizon.
%
%   COMPONENT STATUS: REAL (analytical model, NOT fitted to real data)
%
%   [sigmaLong, sigmaLat] = PREDICTIONUNCERTAINTY(obs, times, cfg) returns
%   the standard deviation of the predicted position, decomposed along and
%   across the road user's direction of travel.
%
%   IR-PSC step 5: represent uncertainty.
%
%   Model
%   -----
%   Treating unknown acceleration as white noise of standard deviation
%   sigma_a, the position variance after time t grows as
%
%       var(t) = var_pos0 + var_vel0 * t^2 + (sigma_a^2 * t^4) / 4
%
%   The t^4 term is what makes far-future predictions appropriately vague.
%   Longitudinal and lateral directions use the same structure but different
%   noise levels, because a vehicle's speed is far more predictable than its
%   sideways motion.
%
%   Two further terms are added laterally:
%     - a class-specific irregular-motion allowance (see cfg.prediction.
%       lateralSpread), which is the term that encodes "this road user is
%       not going to stay in a lane",
%     - inflation for low track confidence and for very young tracks, since
%       a track seen for two frames is a much weaker basis for extrapolation
%       than one seen for fifty.
%
%   HONESTY NOTE: the numbers in cfg.prediction are engineering estimates
%   chosen to be plausible. They have NOT been fitted to Indian traffic data,
%   and no claim is made that they match real-world statistics. Fitting them
%   against recorded trajectories is future work.
%
%   Inputs:
%       obs   - obstacle struct from makeObstacle()
%       times - 1xT or Tx1 prediction times in seconds
%       cfg   - config struct from irpscConfig()
%
%   Outputs:
%       sigmaLong - Tx1 std dev along the direction of travel (m)
%       sigmaLat  - Tx1 std dev across the direction of travel (m)
%
%   Example:
%       [sL, sT] = predictionUncertainty(o, 0:0.2:3, cfg);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTOBSTACLES, IRREGULARMOTIONMODEL.

t  = times(:);
pc = cfg.prediction;

% If uncertainty is ablated away, return a tiny fixed value rather than
% zero, so downstream divisions stay finite.
if isfield(cfg, 'ablation') && ~cfg.ablation.useUncertainty
    sigmaLong = repmat(0.05, numel(t), 1);
    sigmaLat  = repmat(0.05, numel(t), 1);
    return;
end

varPos0 = pc.posNoiseStd^2;
varVel0 = pc.velNoiseStd^2;
sigA    = pc.accelStd;

base = varPos0 + varVel0 * t.^2 + (sigA^2 * t.^4) / 4;

sigmaLong = sqrt(base);

% Lateral: same base, plus the class irregular-motion allowance.
lateralRate = classLateralSpread(obs.class, pc);
sigmaLat    = sqrt(base + (lateralRate * t).^2);

% Heading uncertainty smears position sideways in proportion to distance
% travelled: an error of dtheta at distance L displaces by roughly L*dtheta.
travelled = max(obs.speed, 0) * t;
hStd = pc.headingStd;
if isfield(pc, 'headingStdClass') && isfield(pc.headingStdClass, obs.class)
    hStd = pc.headingStdClass.(obs.class);
end
sigmaLat  = sqrt(sigmaLat.^2 + (travelled * hStd).^2);

% Weak evidence inflates uncertainty.
confFactor = 1 + 1.5 * (1 - max(min(obs.confidence, 1), 0));
ageFactor  = 1 + 0.5 * exp(-max(obs.age, 0) / 5);   % new tracks are shaky
infl       = confFactor * ageFactor;

sigmaLong = min(sigmaLong * infl, pc.maxSigma);
sigmaLat  = min(sigmaLat  * infl, pc.maxSigma);
end

% =====================================================================
function rate = classLateralSpread(className, pc)
%CLASSLATERALSPREAD Lateral spread rate (m of std dev per second) by class.
if isfield(pc.lateralSpread, className)
    rate = pc.lateralSpread.(className);
else
    rate = pc.lateralSpread.unknown;
end
end
