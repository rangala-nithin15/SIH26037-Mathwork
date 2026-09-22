function [meanPos, meanVel] = predictConstantVelocity(obs, times)
%PREDICTCONSTANTVELOCITY Interpretable constant-acceleration motion model.
%
%   COMPONENT STATUS: REAL
%
%   [meanPos, meanVel] = PREDICTCONSTANTVELOCITY(obs, times) propagates a
%   road user forward using its current velocity and (if known) acceleration.
%
%   Why start here
%   --------------
%   This is deliberately the simplest defensible predictor. It is fully
%   interpretable, has no training data requirement, and gives an honest
%   baseline that a learned predictor must beat to justify its complexity.
%   Its known weakness -- it cannot anticipate a turn that has not started --
%   is exactly what the uncertainty model and the irregular-motion model
%   exist to cover.
%
%   The acceleration term is damped over the horizon rather than integrated
%   indefinitely. A measured acceleration is informative for the next
%   fraction of a second but extrapolating it for three seconds produces
%   absurd speeds, so its contribution decays.
%
%   Inputs:
%       obs   - obstacle struct from makeObstacle()
%       times - 1xT or Tx1 vector of prediction times in seconds (>= 0)
%
%   Outputs:
%       meanPos - Tx2 predicted [x y] positions
%       meanVel - Tx2 predicted [vx vy] velocities
%
%   Example:
%       [p, v] = predictConstantVelocity(o, 0:0.2:3.0);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTOBSTACLES, PREDICTIONUNCERTAINTY.

validateattributes(times, {'numeric'}, {'vector','nonnegative','finite','real'}, ...
                   mfilename, 'times');
t = times(:);
T = numel(t);

p0 = obs.pos(:).';
v0 = obs.vel(:).';
a0 = obs.accel(:).';

% Acceleration credibility decays with a 1 second time constant.
tauA  = 1.0;
decay = tauA * (1 - exp(-t / tauA));          % integral of exp(-t/tau)
decay2 = tauA * (t - decay);                  % double integral

meanPos = p0 + t * v0 + decay2 * a0;
meanVel = v0 + decay * a0;

% Never predict a road user reversing or exceeding a plausible class speed.
[names, info] = objectClasses();
ci = find(strcmp(names, obs.class), 1);
if ~isempty(ci)
    vmax  = info(ci).maxSpeed;
    spd   = sqrt(sum(meanVel.^2, 2));
    over  = spd > vmax;
    if any(over)
        scale = ones(T,1);
        scale(over) = vmax ./ spd(over);
        meanVel = meanVel .* scale;
    end
end
end
