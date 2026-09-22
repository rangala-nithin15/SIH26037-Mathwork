function traj = generateTrajectory(corridor, dProfile, ego, cfg, riskProfile, stationCeiling)
%GENERATETRAJECTORY Build the final continuous trajectory from an offset profile.
%
%   COMPONENT STATUS: REAL
%
%   traj = GENERATETRAJECTORY(corridor, dProfile, ego, cfg)
%   traj = GENERATETRAJECTORY(corridor, dProfile, ego, cfg, riskProfile, stationCeiling)
%   converts the chosen lateral-offset profile into a world-frame,
%   time-parameterised trajectory the controller can follow.
%
%   IR-PSC steps 7 and 9: continuous road-following trajectory, spatially
%   and temporally smoothed.
%
%   Steps:
%     1. Smooth the offset profile. The dynamic-programming solution is
%        optimal on a discrete grid, so it can contain 0.25 m steps between
%        adjacent stations. Smoothing removes the quantisation without
%        materially changing the route chosen.
%     2. Map (station, offset) back to world coordinates.
%     3. Anchor the first point to the ego's actual position, so the
%        controller is never handed a path that starts somewhere the vehicle
%        is not.
%     4. Resample to a fixed number of points and smooth the geometry.
%     5. Compute heading and curvature.
%     6. Assign a JERK-LIMITED speed profile (speedProfile) that honours
%        curvature, risk and any per-station speed ceiling (narrow passage,
%        pothole, lead vehicle, stop point), then integrate arrival times.
%     7. Mark which samples are actually REACHED. If the profile comes to
%        rest before the end of the path, later samples are geometry only:
%        they get time Inf and reachable = false, and no safety check or
%        risk evaluation treats them as places the vehicle will be.
%
%   Inputs:
%       corridor       - corridor struct from extractCorridor()
%       dProfile       - Nx1 lateral offsets from deformTrajectory()
%       ego            - ego state struct from makeEgoState()
%       cfg            - config struct from irpscConfig()
%       riskProfile    - (optional) Nx1 risk per station, used for speed
%       stationCeiling - (optional) Nx1 extra speed ceiling per station, m/s
%
%   Outputs:
%       traj - struct with fields:
%           .pos       Mx2 world positions
%           .heading   Mx1 heading (rad)
%           .curvature Mx1 signed curvature (1/m)
%           .speed     Mx1 speed (m/s)
%           .s         Mx1 arc length (m)
%           .times     Mx1 arrival time from now (s); Inf where not reached
%           .reachable Mx1 logical, sample is reached before the vehicle stops
%           .stopS     arc length where the vehicle comes to rest (Inf if not)
%           .ceiling   Mx1 speed ceiling used (m/s)
%           .offsets   Nx1 smoothed lateral offsets at the corridor stations
%           .valid     logical
%           .isSafeStop logical (false here)
%
%   Example:
%       traj = generateTrajectory(corr, dProfile, ego, cfg, riskPerStation);
%
%   Requires: base MATLAB only.
%
%   See also DEFORMTRAJECTORY, SPEEDPROFILE, SCORETRAJECTORY.

if nargin < 5, riskProfile    = []; end
if nargin < 6, stationCeiling = []; end

N = size(corridor.center, 1);
if N < 2 || ~corridor.valid
    traj = emptyTraj(ego);
    return;
end

dProfile = dProfile(:);
if numel(dProfile) ~= N
    dProfile = resampleSeries(dProfile, N);
end

dSmooth = smoothOffsetProfile(dProfile, cfg);    % keeps station 1 at the vehicle

P = frenetToCartesian(corridor.center, corridor.s, dSmooth);
P(1,:) = ego.pos(:).';

% Sample spacing of at least ~0.8 m: on a short (truncated) corridor 41
% points would be packed 0.3 m apart, where millimetre wobbles turn into
% large curvature estimates and spurious feasibility failures.
Lpath = pathArcLength(P);
M = max(5, min(cfg.traj.numPoints, floor(Lpath(end) / 0.8) + 1));
P = resamplePath(P, M);
P = smoothPath(P, cfg.traj.smoothWindow, cfg.traj.smoothPasses, true);
% End-locked smoothing bunches points near the ends (the second point can
% move several metres). Resample again so spacing is uniform for the
% curvature and speed computations (Phase 2).
P = resamplePath(P, M);

sArc = pathArcLength(P);
th   = pathHeading(P);
k    = pathCurvature(P);

% Map per-station quantities onto trajectory samples by arc-length fraction.
if sArc(end) > 1e-6 && corridor.s(end) > 1e-6
    sEq = sArc / sArc(end) * corridor.s(end);
else
    sEq = zeros(M,1);
end
if isempty(riskProfile)
    riskM = zeros(M,1);
else
    riskM = interpStation(corridor.s, riskProfile(:), sEq);
end
if isempty(stationCeiling)
    ceilM = inf(M,1);
else
    ceilM = interpStationMin(corridor.s, stationCeiling(:), sEq);
end

aNow = 0;
if isfield(ego, 'accel') && isfinite(ego.accel)
    aNow = ego.accel;
end
[v, pinfo] = speedProfile(sArc, k, cfg.ego.maxSpeed, ego.speed, cfg, riskM, ceilM, aNow);

times = zeros(M,1);
reachable = true(M,1);
for i = 2:M
    ds    = sArc(i) - sArc(i-1);
    vAvg  = 0.5 * (v(i) + v(i-1));
    if vAvg < 0.05 && v(i) <= 0 && sArc(i) > pinfo.stopS + 0.05
        reachable(i:end) = false;
        times(i:end)     = Inf;
        break;
    end
    times(i) = times(i-1) + ds / max(vAvg, 0.1);
end

traj.pos        = P;
traj.heading    = th;
traj.curvature  = k;
traj.speed      = v;
traj.s          = sArc;
traj.times      = times;
traj.reachable  = reachable;
traj.stopS      = pinfo.stopS;
traj.ceiling    = pinfo.ceiling;
traj.offsets    = dSmooth;
traj.valid      = true;
traj.isSafeStop = false;
traj.emergency  = false;
end

function traj = emptyTraj(ego)
p = ego.pos(:).';
traj.pos        = [p; p];
traj.heading    = [ego.heading; ego.heading];
traj.curvature  = [0; 0];
traj.speed      = [0; 0];
traj.s          = [0; 0];
traj.times      = [0; 0];
traj.reachable  = [true; true];
traj.stopS      = 0;
traj.ceiling    = [0; 0];
traj.offsets    = [0; 0];
traj.valid      = false;
traj.isSafeStop = false;
traj.emergency  = false;
end

function y = resampleSeries(x, n)
x = x(:);
m = numel(x);
if m == n
    y = x;
elseif m < 2
    y = repmat(x(1), n, 1);
else
    y = interp1(linspace(0,1,m).', x, linspace(0,1,n).', 'linear');
end
end

function y = interpStation(sSta, x, sq)
keep = [true; diff(sSta(:)) > 1e-9];
if sum(keep) < 2
    y = repmat(x(1), numel(sq), 1);
    return;
end
y = interp1(sSta(keep), x(keep), min(max(sq, sSta(1)), sSta(end)), 'linear');
end

function y = interpStationMin(sSta, x, sq)
% Speed ceilings must not be diluted by interpolation: take the tighter of
% the two bracketing stations.
sSta = sSta(:);
N = numel(sSta);
y = inf(numel(sq),1);
for i = 1:numel(sq)
    j = find(sSta <= sq(i), 1, 'last');
    if isempty(j), j = 1; end
    j2 = min(j+1, N);
    y(i) = min(x(j), x(j2));
end
end
