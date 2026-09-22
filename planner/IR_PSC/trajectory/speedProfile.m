function [v, info] = speedProfile(sArc, curvature, vTarget, vCurrent, cfg, riskProfile, extraCeiling, aCurrent)
%SPEEDPROFILE Feasible, comfortable, JERK-LIMITED speed along a planned path.
%
%   COMPONENT STATUS: REAL
%
%   v = SPEEDPROFILE(sArc, curvature, vTarget, vCurrent, cfg)
%   v = SPEEDPROFILE(..., riskProfile, extraCeiling, aCurrent)
%   [v, info] = SPEEDPROFILE(...)
%
%   Assigns a speed to every point of a planned path so that the vehicle
%   respects curvature, predicted risk, any extra speed ceilings (narrow
%   passages, potholes, a slower lead vehicle, a stop point) AND the
%   vehicle's acceleration, braking and jerk limits.
%
%   Why this was rewritten (Phase 2)
%   --------------------------------
%   The previous version applied acceleration limits with a forward and a
%   backward pass. That produces a piecewise-constant acceleration: the
%   acceleration jumps from maxAccel to 0 the instant the speed cap is
%   reached. checkFeasibility then (correctly) measured jerks of 6-33 m/s^3
%   against a 3 m/s^3 limit, rejected the plan, and the vehicle performed
%   an emergency stop -- on a perfectly straight, empty road. From
%   standstill every new plan was rejected the same way, so the vehicle
%   could never move again. The limit was right; the profile was wrong.
%
%   Method
%   ------
%     1. Speed ceiling per point: min(vTarget, sqrt(maxLatAccel/|k|),
%        risk-scaled speed, extraCeiling). The ceiling is eroded (running
%        minimum) and then averaged over cfg.traj.ceilingWindow stations.
%        Erosion followed by averaging can only LOWER the ceiling, never
%        raise it, so no limit is violated by the smoothing.
%     2. A braking envelope is built backwards from the ceiling with the
%        planned deceleration cfg.traj.profileDecel, so every slow point is
%        anticipated early enough.
%     3. A jerk-limited longitudinal model (state: position, speed,
%        acceleration) is integrated forward in time from the vehicle's
%        CURRENT speed and acceleration, tracking the envelope with
%        feed-forward plus proportional control. Acceleration is clamped
%        to [-maxDecel, maxAccel] and its rate of change to profileJerk.
%        Near standstill the deceleration is limited to sqrt(2*j*v) so the
%        vehicle can come to rest without a jerk spike.
%     4. The integrated speed is sampled at the path's arc lengths. If the
%        model comes to rest before the end of the path, every later point
%        gets speed 0 and info.stopS records where.
%
%   Inputs:
%       sArc         - Nx1 arc length along the path (m), non-decreasing
%       curvature    - Nx1 signed curvature (1/m)
%       vTarget      - scalar desired cruising speed (m/s)
%       vCurrent     - scalar current ego speed (m/s)
%       cfg          - config struct from irpscConfig()
%       riskProfile  - (optional) Nx1 risk in [0,1]. Default 0.
%       extraCeiling - (optional) Nx1 additional speed ceiling (m/s), Inf
%                      where there is none. Default none.
%       aCurrent     - (optional) scalar current acceleration. Default 0.
%
%   Outputs:
%       v    - Nx1 speed profile (m/s), non-negative
%       info - struct: .ceiling (Nx1 final ceiling), .stopS (arc length at
%              which the profile reaches rest, Inf if it does not),
%              .maxCeilingExcess (largest v - ceiling, m/s)
%
%   Example:
%       v = speedProfile((0:40)', zeros(41,1), 11.1, 6.0, irpscConfig('village'));
%
%   Requires: base MATLAB only.
%
%   See also GENERATETRAJECTORY, CHECKFEASIBILITY.

N = numel(sArc);
if nargin < 6 || isempty(riskProfile),  riskProfile  = zeros(N,1); end
if nargin < 7 || isempty(extraCeiling), extraCeiling = inf(N,1);   end
if nargin < 8 || isempty(aCurrent),     aCurrent     = 0;          end

sArc         = sArc(:);
curvature    = curvature(:);
riskProfile  = riskProfile(:);
extraCeiling = extraCeiling(:);
if isscalar(extraCeiling), extraCeiling = repmat(extraCeiling, N, 1); end

e    = cfg.ego;
tr   = cfg.traj;
jMax = min(tr.profileJerk, e.maxJerk);

% ---- 1. ceiling ---------------------------------------------------------
kAbs  = max(abs(curvature), 1e-6);
% The profile is planned against a fraction of the lateral-acceleration
% limit: the jerk-limited generator tracks its ceiling with a small lag, and
% the margin keeps that lag inside the limit checkFeasibility enforces.
vCurv = sqrt(tr.latAccelMargin * e.maxLatAccel ./ kAbs);
vRisk = vTarget * (1 - cfg.risk.speedReduction * min(max(riskProfile, 0), 1));
ceilRaw = min([repmat(vTarget, N, 1), vCurv, vRisk, extraCeiling], [], 2);
ceilRaw = max(ceilRaw, cfg.traj.minSpeed);
ceilRaw(~isfinite(ceilRaw)) = vTarget;

w = max(0, round(tr.ceilingWindow));
vCeil = runningMean(runningMin(ceilRaw, w), w);
vCeil = min(vCeil, ceilRaw);          % numerical guard: never above the raw limit

info.ceiling = vCeil;
info.stopS   = Inf;
info.maxCeilingExcess = 0;

if N < 2
    v = max(min(vCurrent, vCeil), 0);
    return;
end

% ---- 2. braking envelope -------------------------------------------------
dPlan = min(tr.profileDecel, e.maxDecel);
vRef = vCeil;
for i = N-1:-1:1
    ds = max(sArc(i+1) - sArc(i), 0);
    vRef(i) = min(vRef(i), sqrt(vRef(i+1)^2 + 2 * dPlan * ds));
end

% ---- 3. jerk-limited forward integration ---------------------------------
dt     = tr.profileDt;
% Integrate against limits slightly inside the vehicle limits. The profile
% is later SAMPLED at the path's arc lengths and checked with finite
% differences; without this margin, sampling error alone (~0.2 %) can push
% a profile that is exactly at the limit over it.
aAccLim = e.maxAccel * tr.profileLimitMargin;
aDecLim = e.maxDecel * tr.profileLimitMargin;
kMax   = max(1, ceil(tr.profileMaxTime / dt));
L      = sArc(end);
gain   = tr.profileGain;
preview = 0.3;                        % s, compensates the jerk-limited lag

sH = zeros(kMax+1,1);  vH = zeros(kMax+1,1);
s  = 0;
vv = max(vCurrent, 0);
a  = min(max(aCurrent, -e.maxDecel), e.maxAccel);
sH(1) = s;  vH(1) = vv;
n = 1;
idx = 1;                              % bracket pointer into sArc (s is monotone)
stopped = false;

for k = 1:kMax
    sq = min(s + vv * preview, L);
    [vr, slope, idx] = sampleEnvelope(sArc, vRef, sq, idx);
    aFF  = vr * slope;                % d(v)/dt along the envelope = v dv/ds
    aDes = aFF + gain * (vr - vv);
    aDes = min(max(aDes, -aDecLim), aAccLim);
    aDes = max(aDes, -sqrt(2 * jMax * max(vv, 0)));   % smooth arrival at rest

    da = min(max(aDes - a, -jMax * dt), jMax * dt);
    a  = a + da;
    vNew = vv + a * dt;
    if vNew <= 0
        vNew = 0;
        a    = 0;
    end
    s  = s + 0.5 * (vv + vNew) * dt;
    vv = vNew;
    n  = n + 1;
    sH(n) = s;  vH(n) = vv;

    if s >= L
        break;
    end
    if vv < 0.02 && vr < 0.05
        stopped = true;
        break;
    end
end

sH = sH(1:n);  vH = vH(1:n);
keep = [true; diff(sH) > 1e-6];
sK = sH(keep);  vK = vH(keep);
if ~keep(end)
    vK(end) = vH(end);                % final state is the rest state
end

v = zeros(N,1);
if numel(sK) < 2
    v(:) = 0;
    v(1) = max(vCurrent, 0);
    if stopped || vH(end) <= 0.02
        info.stopS = sArc(1);
        v(2:end) = 0;
    end
else
    inside = sArc <= sK(end);
    v(inside) = interp1(sK, vK, sArc(inside), 'linear');
    if stopped
        v(~inside) = 0;
        info.stopS = sK(end);
    else
        v(~inside) = vK(end);         % time cap reached while still moving
    end
end
v = max(v, 0);
info.maxCeilingExcess = max(v - vCeil);
end

% ---------------------------------------------------------------------------
function [vr, slope, idx] = sampleEnvelope(sArc, vRef, sq, idx)
% Linear interpolation of the envelope and its slope dv/ds at sq, walking a
% monotone bracket pointer (sq never decreases between calls).
N = numel(sArc);
while idx < N-1 && sArc(idx+1) <= sq
    idx = idx + 1;
end
ds = sArc(idx+1) - sArc(idx);
if ds <= 1e-9
    vr = vRef(idx+1);
    slope = 0;
    return;
end
t = min(max((sq - sArc(idx)) / ds, 0), 1);
vr = vRef(idx) + t * (vRef(idx+1) - vRef(idx));
slope = (vRef(idx+1) - vRef(idx)) / ds;
end

function y = runningMin(x, w)
N = numel(x);
y = x;
if w == 0, return; end
for i = 1:N
    y(i) = min(x(max(1, i-w):min(N, i+w)));
end
end

function y = runningMean(x, w)
N = numel(x);
y = x;
if w == 0, return; end
for i = 1:N
    y(i) = mean(x(max(1, i-w):min(N, i+w)));
end
end
