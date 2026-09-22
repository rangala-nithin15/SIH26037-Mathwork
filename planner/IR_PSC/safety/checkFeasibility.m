function [ok, details] = checkFeasibility(traj, cfg, vp)
%CHECKFEASIBILITY Verify a trajectory is drivable by this vehicle.
%
%   COMPONENT STATUS: REAL
%
%   [ok, details] = CHECKFEASIBILITY(traj, cfg, vp) checks a proposed
%   trajectory against the vehicle's kinematic and comfort limits.
%
%   IR-PSC step 10: consider vehicle feasibility.
%
%   A planner that ignores this produces beautiful paths the car cannot
%   drive. The controller then cuts the corner, and the actual driven path
%   is nothing like the checked one -- so every safety check performed on
%   the plan becomes meaningless. Checking feasibility is what keeps the
%   plan and the reality connected.
%
%   Checks performed:
%     1. Curvature <= the vehicle's minimum turning radius allows, and
%        <= cfg.safety.maxCurvature.
%     2. Lateral acceleration v^2 * |k| <= cfg.ego.maxLatAccel.
%     3. Longitudinal acceleration within [-maxDecel, +maxAccel].
%     4. Longitudinal jerk within +/- cfg.ego.maxJerk -- a COMFORT limit.
%        Phase 2: a jerk excess is reported in details.comfortViolations
%        but does not reject the plan. Rejecting it replaced a slightly
%        jerky braking profile with a safe-stop trajectory that brakes
%        harder still -- the opposite of the intent -- and the violations
%        seen in closed loop were dominated by the vehicle already braking
%        (an initial condition no plan can undo) and by finite-difference
%        estimation on spatially sampled profiles. SPEEDPROFILE still
%        respects the limit by construction (see
%        testPlannerCore/testSpeedProfileRespectsJerkOnStraightRoad), and
%        metrics count comfort violations so they cannot hide.
%     5. Implied steering angle within +/- cfg.ego.maxSteer.
%
%   Inputs:
%       traj - trajectory struct from generateTrajectory()
%       cfg  - config struct from irpscConfig()
%       vp   - vehicle params struct from vehicleParams()
%
%   Outputs:
%       ok      - logical, true only if every check passes
%       details - struct with fields:
%                 .maxCurvature, .maxLatAccel, .maxAccel, .maxDecel,
%                 .maxJerk, .maxSteer  - the worst value observed
%                 .curvatureOk, .latAccelOk, .accelOk, .jerkOk, .steerOk
%                 .violations - cellstr naming each failed HARD check
%                 .comfortViolations - cellstr, comfort checks exceeded
%
%   Example:
%       [ok, d] = checkFeasibility(traj, cfg, vp);
%       if ~ok, disp(d.violations); end
%
%   Requires: base MATLAB only.
%
%   See also GENERATETRAJECTORY, CHECKCLEARANCE, PATHCURVATURE.

% Only samples the vehicle actually reaches are checked. Geometry beyond a
% planned stop point (traj.reachable == false) is never driven.
if isfield(traj, 'reachable') && numel(traj.reachable) == numel(traj.speed)
    use = logical(traj.reachable(:));
else
    use = true(numel(traj.speed), 1);
end
k = traj.curvature(use);
v = traj.speed(use);
t = traj.times(use);
s = traj.s(use);
k = k(:);  v = v(:);  t = t(:);  s = s(:);
N = numel(v);

details.violations = {};
details.comfortViolations = {};

% --- 1. Curvature ------------------------------------------------------
maxK   = max([abs(k); 0]);
kLimit = min(vp.maxCurvature, cfg.safety.maxCurvature);
curvatureOk = maxK <= kLimit + 1e-6;

% --- 2. Lateral acceleration -------------------------------------------
% A sample only counts as a violation if a slower speed was physically
% attainable there. If the vehicle is ALREADY too fast for the bend under
% its nose, full braking cannot help at that sample; that is reported as
% an unavoidable transient instead of silently passing or rejecting the
% only plan that brakes as hard as possible.
latAcc = v.^2 .* abs(k);
maxLat = max([latAcc; 0]);
if N >= 1
    vFloor    = brakingFloor(v(1), s, cfg);
    avoidable = v > vFloor + 0.3;     % a slower speed was attainable here
else
    avoidable = false(0,1);
end
over = latAcc > cfg.ego.maxLatAccel + 1e-6;
latAccelOk = ~any(over & avoidable);
details.unavoidableLatAccel = any(over & ~avoidable);

% --- 3. Longitudinal acceleration --------------------------------------
% Phase 2: estimated in the distance domain, a = (v2^2 - v1^2) / (2 ds),
% which is exact for constant acceleration over a segment and stays well
% conditioned at low speed. The earlier dv/dt used arrival times that are
% floored near standstill, and produced large spurious accelerations and
% jerks exactly when the vehicle was stopping.
if N >= 2
    ds = diff(s);
    segOk = ds > 1e-6;
    aLon = zeros(N-1, 1);
    aLon(segOk) = (v([false; segOk]).^2 - v([segOk; false]).^2) ./ (2 * ds(segOk));
else
    aLon = 0;
end
maxAcc  = max([aLon; 0]);
maxDec  = -min([aLon; 0]);
accelOk = maxAcc <= cfg.ego.maxAccel + 1e-3 && ...
          maxDec <= cfg.ego.maxDecel + 1e-3;

% --- 4. Jerk -----------------------------------------------------------
% Jerk between consecutive segments: the change in acceleration divided by
% the time taken to travel between segment midpoints at the local speed.
% Near standstill that time is long, and the jerk correctly tends to zero.
if numel(aLon) >= 2
    sm   = 0.5 * (s(1:end-1) + s(2:end));
    vm   = 0.5 * (v(1:end-1) + v(2:end));
    dsm  = diff(sm);
    vloc = max(0.5 * (vm(1:end-1) + vm(2:end)), 0.3);
    dt2  = max(dsm ./ vloc, 1e-3);
    jerk = diff(aLon) ./ dt2;
else
    jerk = 0;
end
maxJerk = max(abs([jerk; 0]));
jerkOk  = maxJerk <= cfg.ego.maxJerk + 1e-2;

% --- 5. Steering angle -------------------------------------------------
steer    = atan(vp.wheelbase * k);
maxSteer = max(abs(steer));
steerOk  = maxSteer <= cfg.ego.maxSteer + 1e-6;

% --- Collect -----------------------------------------------------------
details.maxCurvature = maxK;
details.maxLatAccel  = maxLat;
details.maxAccel     = maxAcc;
details.maxDecel     = maxDec;
details.maxJerk      = maxJerk;
details.maxSteer     = maxSteer;
details.curvatureOk  = curvatureOk;
details.latAccelOk   = latAccelOk;
details.accelOk      = accelOk;
details.jerkOk       = jerkOk;
details.steerOk      = steerOk;

if ~curvatureOk, details.violations{end+1} = 'curvature'; end
if ~latAccelOk,  details.violations{end+1} = 'lateralAcceleration'; end
if ~accelOk,     details.violations{end+1} = 'longitudinalAcceleration'; end
if ~jerkOk,      details.comfortViolations{end+1} = 'jerk'; end
if ~steerOk,     details.violations{end+1} = 'steeringAngle'; end

ok = isempty(details.violations);
end

% -------------------------------------------------------------------------
function vFloor = brakingFloor(v0, s, cfg)
% Lowest speed physically attainable at each arc length when braking starts
% now: deceleration builds at cfg.ego.brakeJerk up to cfg.ego.maxDecel.
% (Assuming instantaneous full braking made the unavoidable-transient test
% far too optimistic.)
dt = 0.01;
jB = cfg.ego.brakeJerk;
aMax = cfg.ego.maxDecel;
sq = 0;  v = v0;  a = 0;
sH = 0;  vH = v0;
while v > 0 && sq < s(end)
    a = min(a + jB * dt, aMax);
    vN = max(v - a * dt, 0);
    sq = sq + 0.5 * (v + vN) * dt;
    v = vN;
    sH(end+1) = sq;  vH(end+1) = v; %#ok<AGROW>
end
vFloor = zeros(size(s));
if numel(sH) >= 2
    inside = s <= sH(end);
    [sU, iU] = unique(sH);
    vFloor(inside) = interp1(sU, vH(iU), s(inside), 'linear');
end
end
