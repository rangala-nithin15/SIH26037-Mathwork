function [steer, info] = purePursuitControl(traj, ego, cfg, vp)
%PUREPURSUITCONTROL Geometric path-following steering command.
%
%   COMPONENT STATUS: DOCUMENTED FALLBACK
%   (Model Predictive Control Toolbox is not available in this package.
%   Pure pursuit is the standard, well-understood substitute. It is a real,
%   working controller -- not a stub -- but it does not handle constraints
%   the way MPC would. See docs/COMPONENT_REGISTER.md, fallback F7.)
%
%   [steer, info] = PUREPURSUITCONTROL(traj, ego, cfg, vp) computes the road
%   wheel angle that steers the vehicle onto the planned trajectory.
%
%   Method
%   ------
%   Pure pursuit picks a target point one lookahead distance L ahead on the
%   path and steers along the circular arc from the rear axle to that point:
%
%       steer = atan2(2 * wheelbase * sin(alpha), L)
%
%   where alpha is the angle to the target point in the vehicle frame. The
%   lookahead is speed-dependent, L = lookaheadMin + gain * speed, because a
%   fixed lookahead either oscillates at speed or cuts corners when slow.
%
%   Strengths and limits, stated plainly: pure pursuit is simple, stable and
%   easy to explain, which suits a demo that must be defended to judges. It
%   does not anticipate curvature ahead, and it cannot enforce constraints,
%   so it will cut a corner slightly on tight bends. Upgrading to MPC is
%   listed as future work.
%
%   Inputs:
%       traj - trajectory struct from generateTrajectory()
%       ego  - ego state struct from makeEgoState()
%       cfg  - config struct from irpscConfig()
%       vp   - vehicle params struct from vehicleParams()
%
%   Outputs:
%       steer - road wheel angle (rad), saturated to +/- cfg.ego.maxSteer
%       info  - struct with fields:
%               .lookahead      m, lookahead distance used
%               .targetIndex    index of the target point on the trajectory
%               .targetPoint    1x2 world coordinates of the target
%               .crossTrackError m, signed lateral error (positive = the
%                                path is to the LEFT of the vehicle)
%               .headingError   rad, signed heading error
%               .saturated      logical, steering command hit its limit
%
%   Example:
%       [steer, info] = purePursuitControl(traj, ego, cfg, vp);
%
%   Requires: base MATLAB only.
%
%   See also LONGITUDINALCONTROL, BICYCLEMODELSTEP, GENERATETRAJECTORY.

P = traj.pos;
N = size(P,1);

info = struct('lookahead',0,'targetIndex',1,'targetPoint',P(1,:), ...
              'crossTrackError',0,'headingError',0,'saturated',false);

if N < 2
    steer = 0;
    return;
end

% --- Lookahead distance -------------------------------------------------
L = cfg.control.lookaheadMin + cfg.control.lookaheadGain * max(ego.speed, 0);
L = max(L, 0.5);
info.lookahead = L;

% --- Errors relative to the path ---------------------------------------
[sEgo, dEgo, idxEgo] = projectPointOnPath(P, ego.pos);
info.crossTrackError = dEgo;
info.headingError = wrapToPiLocal(traj.heading(min(idxEgo, numel(traj.heading))) ...
                                  - ego.heading);

% --- Target point one lookahead ahead in arc length --------------------
sTarget = sEgo + L;
sPath   = traj.s;
if sTarget >= sPath(end)
    idxT = N;
else
    idxT = find(sPath >= sTarget, 1, 'first');
    if isempty(idxT), idxT = N; end
end
target = P(idxT,:);
info.targetIndex = idxT;
info.targetPoint = target;

% --- Pure pursuit geometry ---------------------------------------------
dx = target(1) - ego.pos(1);
dy = target(2) - ego.pos(2);

% Rotate into the vehicle frame.
ct = cos(ego.heading);  st = sin(ego.heading);
xv =  dx*ct + dy*st;
yv = -dx*st + dy*ct;

ld = hypot(xv, yv);
if ld < 1e-3
    steer = 0;
    return;
end

% Curvature of the arc from the rear axle to the target point.
kappa = 2 * yv / (ld^2);
steer = atan(vp.wheelbase * kappa);

% --- Saturate -----------------------------------------------------------
maxS = cfg.ego.maxSteer;
if abs(steer) > maxS
    steer = sign(steer) * maxS;
    info.saturated = true;
end
end
