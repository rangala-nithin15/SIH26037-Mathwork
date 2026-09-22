function traj = safeStopTrajectory(ego, corridor, cfg, emergency, refPath)
%SAFESTOPTRAJECTORY Controlled stop along the best available path.
%
%   COMPONENT STATUS: REAL
%
%   traj = SAFESTOPTRAJECTORY(ego, corridor, cfg, emergency) produces a
%   trajectory that brings the vehicle to a standstill.
%
%   IR-PSC step 13: stop safely when no feasible trajectory exists.
%
%   This is the fallback the whole confidence-aware design exists to reach
%   gracefully. It is invoked when the corridor is invalid, when deformation
%   finds no admissible path, when clearance or feasibility checks fail, or
%   when the decision logic escalates to SAFE_STOP.
%
%   The stop follows, in order of preference:
%     1. refPath, normally the trajectory the vehicle was already following
%        (Phase 2). Braking along the current plan keeps an avoidance
%        manoeuvre in place instead of steering back into the hazard it was
%        avoiding, which is what stopping along the centreline did.
%     2. The corridor centreline at the vehicle's CURRENT lateral offset
%        (previously offset 0, which put a kink into the stop path whenever
%        the vehicle was not exactly centred).
%     3. The current heading, where no corridor exists.
%
%   Deceleration is constant, giving the classic v^2 = v0^2 - 2*a*s profile.
%   Constant deceleration is used rather than a smoother jerk-limited
%   profile because in this situation stopping distance matters more than
%   passenger comfort.
%
%   HONESTY NOTE: "safe stop" names the INTENT and the manoeuvre. It does
%   not assert that stopping is always safe, that the vehicle will always
%   stop in time, or that a following vehicle will not strike it. Those are
%   outcomes this project has not measured and does not claim.
%
%   Inputs:
%       ego       - ego state struct from makeEgoState()
%       corridor  - corridor struct from extractCorridor(), or [] if none
%       cfg       - config struct from irpscConfig()
%       emergency - logical; true uses cfg.ego.emergencyDecel, false uses
%                   cfg.ego.maxDecel. Default false.
%       refPath   - (optional) Kx2 polyline to stop along, e.g. the previous
%                   trajectory. Ignored if shorter than the stop distance
%                   needs or if the vehicle is far from it.
%
%   Outputs:
%       traj - trajectory struct in the same format as generateTrajectory(),
%              with an extra field .isSafeStop = true
%
%   Example:
%       traj = safeStopTrajectory(ego, corridor, cfg, true);
%
%   Requires: base MATLAB only.
%
%   See also GENERATETRAJECTORY, DECISIONLOGIC.

if nargin < 4 || isempty(emergency), emergency = false; end
if nargin < 5, refPath = []; end

if emergency
    decel = cfg.ego.emergencyDecel;
else
    decel = cfg.ego.maxDecel;
end

v0       = max(ego.speed, 0);
stopDist = v0^2 / (2 * max(decel, 0.1));
stopDist = max(stopDist, 0.5);     % always emit a path of non-zero length

M = cfg.traj.numPoints;

% --- Geometry -----------------------------------------------------------
useRef = false;
if ~isempty(refPath) && size(refPath,1) >= 2
    [sE, dE] = projectPointOnPath(refPath, ego.pos);
    sRef = pathArcLength(refPath);
    useRef = abs(dE) < 1.0 && (sRef(end) - sE) >= min(stopDist, 2.0);
end
useCorridor = ~useRef && ~isempty(corridor) && isfield(corridor,'center') && ...
              size(corridor.center,1) >= 2 && corridor.length > 0.5;

if useRef
    sAvail = sRef(end) - sE;
    sStop  = sE + linspace(0, min(stopDist, sAvail), M).';
    P      = frenetToCartesian(refPath, sStop, 0);
    P(1,:) = ego.pos(:).';
    if sAvail < stopDist
        % Extend straight along the end of the reference path.
        extra = stopDist - sAvail;
        hEnd  = pathHeading(refPath);
        P = [P; P(end,:) + extra * [cos(hEnd(end)), sin(hEnd(end))]];
    end
elseif useCorridor
    [sE, dE] = projectPointOnPath(corridor.center, ego.pos);
    sAvail = max(corridor.length - sE, 0);
    sStop  = sE + linspace(0, min(stopDist, sAvail), M).';
    P      = frenetToCartesian(corridor.center, sStop, dE);
    P(1,:) = ego.pos(:).';
    if sAvail < stopDist
        extra = stopDist - sAvail;
        P = [P; P(end,:) + extra * [cos(corridor.heading(end)), sin(corridor.heading(end))]];
    end
else
    fwd = [cos(ego.heading), sin(ego.heading)];
    sStop = linspace(0, stopDist, M).';
    P = ego.pos(:).' + sStop .* fwd;
end

P = resamplePath(P, M);

sArc = pathArcLength(P);
th   = pathHeading(P);
k    = pathCurvature(P);

% --- Speed: constant deceleration to zero -------------------------------
L = max(sArc(end), 1e-3);
v = sqrt(max(v0^2 - 2 * decel * sArc, 0));
v(sArc >= L) = 0;
v(end)       = 0;

% --- Timing -------------------------------------------------------------
times = zeros(M,1);
for i = 2:M
    ds   = sArc(i) - sArc(i-1);
    vAvg = 0.5 * (v(i) + v(i-1));
    if vAvg < 0.05
        % Vehicle has stopped: remaining samples share the final instant.
        times(i) = times(i-1);
    else
        times(i) = times(i-1) + ds / vAvg;
    end
end

traj.pos        = P;
traj.heading    = th;
traj.curvature  = k;
traj.speed      = v;
traj.s          = sArc;
traj.times      = times;
traj.reachable  = true(M,1);
traj.stopS      = sArc(end);
traj.ceiling    = zeros(M,1);
traj.offsets    = zeros(M,1);
traj.valid      = true;
traj.isSafeStop = true;
traj.emergency  = emergency;
end
