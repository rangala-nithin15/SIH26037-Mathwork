function [accelCmd, cstate] = longitudinalControl(traj, ego, speedLimit, cfg, cstate, emergencyBrake)
%LONGITUDINALCONTROL PI speed controller with anti-windup.
%
%   COMPONENT STATUS: DOCUMENTED FALLBACK
%   (Substitute for MPC longitudinal control; see fallback F7 in
%   docs/COMPONENT_REGISTER.md. This is a real working controller.)
%
%   [accelCmd, cstate] = LONGITUDINALCONTROL(traj, ego, speedLimit, cfg,
%   cstate) computes the longitudinal acceleration command that tracks the
%   planned speed, capped by the decision logic's speed limit.
%
%   The target speed is read from the trajectory at the vehicle's current
%   arc-length position, so the controller follows the planned speed PROFILE
%   rather than a single set point. That matters because the profile already
%   encodes slowing for curvature and for risk.
%
%   Phase 2 additions
%   -----------------
%     - Feed-forward. The planned profile is jerk-limited, so its slope is a
%       meaningful acceleration request. The command is now
%       a = v*dv/ds (from the profile, at a short preview) + PI correction,
%       which tracks planned stops without the lag of a pure PI loop.
%     - Emergency braking. The original clamp to -cfg.ego.maxDecel meant the
%       configured emergency deceleration (cfg.ego.emergencyDecel) was never
%       used by anything. With emergencyBrake = true the lower limit becomes
%       -cfg.ego.emergencyDecel. The decision logic sets it only for genuine
%       emergencies.
%
%   Anti-windup
%   -----------
%   The integrator is clamped to +/- cfg.control.iMax, and integration is
%   suspended whenever the output is saturated and the error would push it
%   further into saturation. Without this, a long period of commanded
%   braking against an acceleration limit winds up a large integral term
%   that then causes a surge the moment the road clears -- exactly the wrong
%   behaviour after a hazard has passed.
%
%   Inputs:
%       traj       - trajectory struct from generateTrajectory()
%       ego        - ego state struct from makeEgoState()
%       speedLimit - m/s cap from decisionLogic()
%       cfg        - config struct from irpscConfig()
%       cstate     - controller state from the previous call, or [] to init
%       emergencyBrake - (optional) logical, allow emergency deceleration
%
%   Outputs:
%       accelCmd - commanded longitudinal acceleration (m/s^2), saturated to
%                  [-cfg.ego.maxDecel, +cfg.ego.maxAccel], or to
%                  -cfg.ego.emergencyDecel when emergencyBrake is true
%       cstate   - updated controller state, pass into the next call
%
%   Example:
%       cstate = [];
%       [a, cstate] = longitudinalControl(traj, ego, 8.0, cfg, cstate);
%
%   Requires: base MATLAB only.
%
%   See also PUREPURSUITCONTROL, DECISIONLOGIC, SPEEDPROFILE.

if nargin < 6 || isempty(emergencyBrake), emergencyBrake = false; end

if isempty(cstate) || ~isstruct(cstate)
    cstate.integral = 0;
    cstate.lastErr  = 0;
end

dt = cfg.sim.dt;

% --- Target speed (and profile slope) at a short preview ahead ----------
aFF = 0;
if size(traj.pos,1) >= 2
    sEgo = projectPointOnPath(traj.pos, ego.pos);
    sP   = traj.s(:);
    vP   = traj.speed(:);
    keep = [true; diff(sP) > 1e-9];
    if sum(keep) >= 2
        sK = sP(keep);  vK = vP(keep);
        % Preview at least cfg.control.minPreviewDist ahead. At standstill a
        % zero preview reads the profile exactly where it starts -- at the
        % current speed, 0 -- and the vehicle would never pull away.
        prev = max(max(ego.speed, 0) * cfg.control.speedPreview, cfg.control.minPreviewDist);
        sQ = min(max(sEgo + prev, sK(1)), sK(end));
        vTarget = interp1(sK, vK, sQ, 'linear');
        j  = min(max(find(sK <= sQ, 1, 'last'), 1), numel(sK) - 1);
        dvds = (vK(j+1) - vK(j)) / (sK(j+1) - sK(j));
        aFF  = vTarget * dvds;
    else
        vTarget = traj.speed(1);
    end
else
    vTarget = 0;
end

if vTarget > speedLimit
    aFF = min(aFF, 0);                 % the cap, not the profile, is binding
end
vTarget = min(vTarget, speedLimit);
vTarget = max(vTarget, 0);

% --- PI ------------------------------------------------------------------
err = vTarget - ego.speed;

aMax = cfg.ego.maxAccel;
if emergencyBrake
    aMin = -cfg.ego.emergencyDecel;
else
    aMin = -cfg.ego.maxDecel;
end

unsaturated = aFF + cfg.control.kpSpeed * err + cfg.control.kiSpeed * cstate.integral;

% Integrate only when doing so will not deepen an existing saturation.
willSaturateHigh = unsaturated >= aMax && err > 0;
willSaturateLow  = unsaturated <= aMin && err < 0;
if ~willSaturateHigh && ~willSaturateLow
    cstate.integral = cstate.integral + err * dt;
    cstate.integral = max(min(cstate.integral, cfg.control.iMax), -cfg.control.iMax);
end

accelCmd = aFF + cfg.control.kpSpeed * err + cfg.control.kiSpeed * cstate.integral;
if emergencyBrake && vTarget <= 0.05
    accelCmd = aMin;                   % full braking until stationary
end
accelCmd = max(min(accelCmd, aMax), aMin);

cstate.lastErr = err;
end
