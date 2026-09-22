function [vCeil, info] = leadVehicleCeiling(corridor, dPath, preds, obstacles, ego, cfg, vp)
%LEADVEHICLECEILING Speed ceiling that keeps a safe gap to road users in the path.
%
%   COMPONENT STATUS: REAL (constant-deceleration gap keeping)
%
%   [vCeil, info] = LEADVEHICLECEILING(corridor, dPath, preds, obstacles, ego, cfg, vp)
%   returns a per-station speed ceiling that makes the vehicle FOLLOW a
%   slower road user it cannot pass, or come to a controlled stop behind a
%   road user standing in its path, instead of driving into it.
%
%   Why this was added (Phase 2)
%   ----------------------------
%   The original pipeline had exactly two responses to a slower vehicle
%   ahead: bend around it, or perform an emergency stop. On a narrow Indian
%   road, or behind a truck on a highway with oncoming traffic, the correct
%   behaviour is usually neither -- it is to slow down and follow until an
%   overtaking gap opens. That is ordinary driving, and without it the
%   demonstration either collided or emergency-stopped behind every truck.
%
%   Which road users count
%   ----------------------
%   A road user constrains speed if, at the time the ego would reach its
%   current along-road position (clamped to the prediction horizon), its
%   PREDICTED position lies laterally within the planned path's swept
%   width plus cfg.safety.lateralClearance. A pedestrian who will have
%   finished crossing by then does not count (the time-aware risk grid and
%   clearance check handle that case); a slow bicycle that will still be in
%   the path does.
%
%   The gap
%   -------
%   Desired gap to the rear of the road user =
%       cfg.safety.longitudinalGap + cfg.safety.timeGap * vLead
%   where vLead is the road user's speed along the road (0 if it is
%   stationary or oncoming). Before the gap point the ceiling is
%       sqrt(vLead^2 + 2 * cfg.safety.leadDecel * distanceToGapPoint)
%   and beyond it the ceiling tapers from vLead to 0 across one gap length.
%
%   Inputs:
%       corridor  - (usable) corridor struct
%       dPath     - Nx1 planned lateral offset at each station
%       preds     - prediction struct array, same order as obstacles
%       obstacles - obstacle struct array
%       ego       - ego state
%       cfg, vp   - config and vehicle params
%
%   Outputs:
%       vCeil - Nx1 speed ceiling (m/s), Inf where unconstrained
%       info - struct: .active (logical), .id (track id of the binding road
%              user, NaN), .class, .gap (current bumper gap, m), .leadSpeed
%
%   Requires: base MATLAB only.
%
%   See also IRPSCPLANNER, SPEEDPROFILE, CHECKCLEARANCE.

N    = size(corridor.center, 1);
s    = corridor.s(:);
dPath = dPath(:);
vCeil = inf(N, 1);
info = struct('active', false, 'id', NaN, 'class', '', 'gap', Inf, 'leadSpeed', NaN);
if isempty(obstacles) || N < 2 || numel(preds) ~= numel(obstacles)
    return;
end

H = cfg.prediction.horizon;
bestBinding = Inf;

for k = 1:numel(obstacles)
    o = obstacles(k);
    [so, dNow, idx] = projectPointOnPath(corridor.center, o.pos);
    if so <= 0.5 || so >= s(end) - 1e-3
        continue;                    % behind the vehicle or beyond the corridor
    end
    if abs(dNow) > max(corridor.leftDist(min(idx,N)), corridor.rightDist(min(idx,N))) + 3
        continue;                    % well off the road
    end

    tReach = min(so / max(ego.speed, 1.0), H);
    pr = preds(k);
    tp = pr.times(:);
    j  = min(max(find(tp <= tReach, 1, 'last'), 1), numel(tp));
    [sR, dR] = projectPointOnPath(corridor.center, pr.pos(j,:));
    dPathAt = interp1(s, dPath, min(max(sR, s(1)), s(end)), 'linear');

    hc  = corridor.heading(min(idx, N));
    rel = o.heading - hc;
    halfAcross = abs(sin(rel)) * o.length/2 + abs(cos(rel)) * o.width/2;
    halfAlong  = abs(cos(rel)) * o.length/2 + abs(sin(rel)) * o.width/2;
    vAlong = o.vel(:).' * [cos(hc); sin(hc)];
    % Same-direction or stationary road users are followed if they are
    % within the preferred side clearance. ONCOMING road users are only
    % stopped for if the bodies would actually overlap within the HARD
    % minimum clearance: passing oncoming traffic in its own lane is normal
    % driving and is handled by the lateral deformation and the risk-based
    % speed reduction, not by stopping in the road.
    if vAlong < -0.5
        band = vp.halfWidth + halfAcross + cfg.safety.minLateralClearance;
    else
        band = vp.halfWidth + halfAcross + cfg.safety.lateralClearance;
    end
    if abs(dR - dPathAt) >= band
        continue;
    end

    vLead  = max(vAlong, 0);
    gapNeed = cfg.safety.longitudinalGap + cfg.safety.timeGap * vLead;
    sGap = so - halfAlong - vp.frontOverhang - gapNeed;   % rear-axle station

    c = zeros(N,1);
    before = s <= sGap;
    c(before)  = sqrt(vLead^2 + 2 * cfg.safety.leadDecel * (sGap - s(before)));
    c(~before) = vLead * max(0, 1 - (s(~before) - sGap) / max(gapNeed, 1));
    vCeil = min(vCeil, c);

    if sGap < bestBinding
        bestBinding    = sGap;
        info.active    = true;
        info.id        = o.id;
        info.class     = o.class;
        info.gap       = so - halfAlong - vp.frontOverhang;
        info.leadSpeed = vLead;
    end
end
end
