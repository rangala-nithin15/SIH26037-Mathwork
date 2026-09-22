function egoNext = bicycleModelStep(ego, steerCmd, accelCmd, cfg)
%BICYCLEMODELSTEP Advance the ego vehicle one step (kinematic bicycle model).
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%   (Vehicle Dynamics Blockset is not available in this package. The
%   kinematic bicycle model is the standard academic substitute -- fallback
%   F8 in docs/COMPONENT_REGISTER.md. It is a real, correct implementation
%   of that model, not a stub.)
%
%   egoNext = BICYCLEMODELSTEP(ego, steerCmd, accelCmd, cfg) integrates the
%   vehicle state forward by cfg.sim.dt.
%
%   Model
%   -----
%   State is [x, y, heading, speed] at the REAR AXLE:
%
%       x'       = v * cos(heading)
%       y'       = v * sin(heading)
%       heading' = v * tan(steer) / wheelbase
%       v'       = a
%
%   Integration is midpoint (RK2) rather than Euler. At a 20 Hz step and
%   realistic speeds, plain Euler accumulates a visible heading drift on
%   sustained turns; midpoint removes most of it for one extra evaluation.
%
%   WHAT THIS MODEL DOES NOT REPRESENT -- stated explicitly so no result
%   produced with it is over-interpreted:
%     - tyre slip, and therefore any loss of grip
%     - load transfer, roll, pitch or suspension
%     - powertrain and brake dynamics (acceleration is applied instantly)
%     - road surface effects, including the potholes the scenarios model as
%       occupancy rather than as disturbance
%     - actuator lag beyond the steering rate limit applied here
%
%   It is therefore valid for evaluating PLANNING and DECISION behaviour,
%   which is what this project studies. It is NOT valid for evaluating
%   handling limits, stability, or ride, and no such claim is made.
%
%   Inputs:
%       ego      - ego state struct from makeEgoState()
%       steerCmd - commanded road wheel angle (rad)
%       accelCmd - commanded longitudinal acceleration (m/s^2)
%       cfg      - config struct from irpscConfig()
%
%   Outputs:
%       egoNext - updated ego state struct, with .time advanced by dt
%
%   Example:
%       ego = bicycleModelStep(ego, steer, accel, cfg);
%
%   Requires: base MATLAB only.
%
%   See also MAKEEGOSTATE, PUREPURSUITCONTROL, LONGITUDINALCONTROL.

dt = cfg.sim.dt;
Lw = cfg.ego.wheelbase;

% --- Actuator limits ----------------------------------------------------
% Steering rate limit: the wheel cannot jump instantly to a new angle.
maxDelta = cfg.ego.maxSteerRate * dt;
steer    = ego.steer + max(min(steerCmd - ego.steer, maxDelta), -maxDelta);
steer    = max(min(steer, cfg.ego.maxSteer), -cfg.ego.maxSteer);

accel = max(min(accelCmd, cfg.ego.maxAccel), -cfg.ego.emergencyDecel);

% --- Midpoint integration ----------------------------------------------
x0 = ego.pos(1);
y0 = ego.pos(2);
h0 = ego.heading;
v0 = max(ego.speed, 0);

% Half step
vHalf = max(v0 + 0.5 * accel * dt, 0);
hHalf = h0 + 0.5 * dt * v0 * tan(steer) / Lw;
xHalf = x0 + 0.5 * dt * v0 * cos(h0);
yHalf = y0 + 0.5 * dt * v0 * sin(h0);   %#ok<NASGU> % kept for clarity

% Full step using midpoint derivatives
x1 = x0 + dt * vHalf * cos(hHalf);
y1 = y0 + dt * vHalf * sin(hHalf);
h1 = h0 + dt * vHalf * tan(steer) / Lw;
v1 = v0 + dt * accel;

% A vehicle cannot be pushed backwards by braking.
v1 = max(v1, 0);

egoNext         = ego;
egoNext.pos     = [x1, y1];
egoNext.heading = wrapToPiLocal(h1);
egoNext.speed   = v1;
egoNext.yawRate = vHalf * tan(steer) / Lw;
egoNext.accel   = accel;
egoNext.steer   = steer;
egoNext.time    = ego.time + dt;
end
