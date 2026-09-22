function [ttc, criticalId, details] = timeToConflict(ego, preds, cfg, vp, path)
%TIMETOCONFLICT Shortest predicted time to a spatial conflict.
%
%   COMPONENT STATUS: REAL
%
%   [ttc, criticalId, details] = TIMETOCONFLICT(ego, preds, cfg, vp)
%   [ttc, criticalId, details] = TIMETOCONFLICT(ego, preds, cfg, vp, path)
%   finds the earliest time at which the ego body and any predicted road
%   user would touch, given how the ego is assumed to move.
%
%   IR-PSC step 6, and a primary trigger for the decision logic.
%
%   How the ego is assumed to move (Phase 2)
%   ----------------------------------------
%   `path` selects the question being asked:
%
%     path = corridor struct  ->  NOMINAL ("do nothing") TTC. The ego keeps
%          its current speed and lateral offset ALONG THE ROAD. This keeps
%          the original design intent -- escalation must not depend on the
%          plan, otherwise a planner that has already dodged would never
%          report danger -- but the earlier straight-line extrapolation
%          drove the ego off the road on every bend and produced false
%          critical TTCs against oncoming traffic on curves.
%
%     path = trajectory struct (fields .pos .heading .times)  ->  PLANNED
%          TTC along the trajectory the vehicle will actually drive,
%          including a planned stop (the vehicle then stays at rest). This
%          is what decides whether an EMERGENCY is genuinely needed.
%
%     path omitted / []  ->  legacy straight-line extrapolation.
%
%   Conflict geometry uses exact body rectangles (BOXDISTANCE). The gap is
%   reduced by cfg.risk.ttcSigmaFactor times the mean predicted standard
%   deviation, so an uncertain prediction triggers earlier.
%
%   Inputs:
%       ego   - ego state struct from makeEgoState()
%       preds - prediction struct array from predictObstacles()
%       cfg   - config struct from irpscConfig()
%       vp    - vehicle params struct from vehicleParams()
%       path  - (optional) corridor struct, trajectory struct, or []
%
%   Outputs:
%       ttc        - seconds to the earliest conflict, Inf if none within
%                    the prediction horizon
%       criticalId - track id responsible for that earliest conflict, NaN
%       details    - struct with fields:
%                    .perObstacle  1xM individual TTC values
%                    .ids          1xM corresponding track ids
%                    .minGap       m, closest predicted approach
%                    .level        'none' | 'warning' | 'critical'
%                    .mode         'nominal' | 'planned' | 'straight'
%
%   Example:
%       ttcNom  = timeToConflict(ego, preds, cfg, vp, corridor);
%       ttcPlan = timeToConflict(ego, preds, cfg, vp, traj);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTEDOCCUPANCYRISK, DECISIONLOGIC, BOXDISTANCE.

if nargin < 5, path = []; end

ttc        = Inf;
criticalId = NaN;
details    = struct('perObstacle', [], 'ids', [], 'minGap', Inf, ...
                    'level', 'none', 'mode', 'straight');

if isempty(preds)
    return;
end

if isstruct(path) && isfield(path, 'times') && isfield(path, 'pos')
    details.mode = 'planned';
elseif isstruct(path) && isfield(path, 'center') && isfield(path, 'valid') && ...
        path.valid && size(path.center,1) >= 2
    details.mode = 'nominal';
end

M      = numel(preds);
perObs = inf(1, M);
ids    = nan(1, M);
minGap = Inf;
k0     = cfg.risk.ttcSigmaFactor;

for k = 1:M
    p  = preds(k);
    tp = p.times(:);
    if isempty(tp)
        continue;
    end
    [egoPos, egoHdg] = egoPoses(ego, tp, path, details.mode);

    gap = inf(numel(tp),1);
    for q = 1:numel(tp)
        CE = egoFootprint(egoPos(q,:), egoHdg(q), vp);
        CO = boxCorners(p.pos(q,:), p.heading(q), 2*obsHalf(p, 'L'), 2*obsHalf(p, 'W'));
        sig = 0.5 * (p.sigmaLong(q) + p.sigmaLat(q));
        gap(q) = boxDistance(CE, CO) - k0 * sig;
    end
    minGap = min(minGap, min(gap));

    hit = find(gap <= 0, 1, 'first');
    if isempty(hit)
        continue;
    end
    if hit == 1
        tk = 0;
    else
        g0 = gap(hit-1);
        g1 = gap(hit);
        frac = g0 / max(g0 - g1, eps);
        tk = tp(hit-1) + frac * (tp(hit) - tp(hit-1));
    end

    perObs(k) = tk;
    ids(k)    = p.id;
    if tk < ttc
        ttc        = tk;
        criticalId = p.id;
    end
end

details.perObstacle = perObs;
details.ids         = ids;
details.minGap      = minGap;

if ttc <= cfg.risk.ttcCritical
    details.level = 'critical';
elseif ttc <= cfg.risk.ttcWarning
    details.level = 'warning';
else
    details.level = 'none';
end
end

% -------------------------------------------------------------------------
function h = obsHalf(p, which)
if isfield(p, 'halfLength') && ~isempty(p.halfLength)
    if which == 'L', h = p.halfLength; else, h = p.halfWidth; end
else
    h = p.radius / sqrt(2);
end
end

function [P, H] = egoPoses(ego, t, path, mode)
T = numel(t);
switch mode
    case 'planned'
        tr = path;
        M  = size(tr.pos,1);
        if isfield(tr, 'reachable') && numel(tr.reachable) == M
            r = find(tr.reachable, 1, 'last');
        else
            r = M;
        end
        tt = tr.times(1:r);
        pp = tr.pos(1:r,:);
        hh = tr.heading(1:r);
        keep = [true; diff(tt(:)) > 1e-9];
        tt = tt(keep);  pp = pp(keep,:);  hh = hh(keep);
        P = zeros(T,2);  H = zeros(T,1);
        vEnd = tr.speed(r);
        for q = 1:T
            if numel(tt) < 2 || t(q) <= tt(1)
                P(q,:) = pp(1,:);  H(q) = hh(1);
            elseif t(q) <= tt(end)
                j = find(tt <= t(q), 1, 'last');
                j = min(j, numel(tt) - 1);
                a = (t(q) - tt(j)) / (tt(j+1) - tt(j));
                P(q,:) = (1-a) * pp(j,:) + a * pp(j+1,:);
                H(q) = atan2((1-a)*sin(hh(j)) + a*sin(hh(j+1)), ...
                             (1-a)*cos(hh(j)) + a*cos(hh(j+1)));
            else
                % Past the end of the plan: at rest if it stops, otherwise
                % continue straight at the final speed.
                dtx = t(q) - tt(end);
                P(q,:) = pp(end,:) + vEnd * dtx * [cos(hh(end)), sin(hh(end))];
                H(q) = hh(end);
            end
        end
    case 'nominal'
        c = path;
        [s0, d0] = projectPointOnPath(c.center, ego.pos);
        sq = s0 + max(ego.speed, 0) * t;
        P = frenetToCartesian(c.center, sq, d0);
        cx = interp1(c.s, cos(c.heading), min(sq, c.s(end)), 'linear', 'extrap');
        cy = interp1(c.s, sin(c.heading), min(sq, c.s(end)), 'linear', 'extrap');
        H = atan2(cy, cx);
        % Beyond the corridor end continue straight along the last heading.
        beyond = sq > c.s(end);
        if any(beyond)
            ext = sq(beyond) - c.s(end);
            P(beyond,:) = c.center(end,:) + ext .* [cos(c.heading(end)), sin(c.heading(end))];
            H(beyond) = c.heading(end);
        end
    otherwise
        fwd = [cos(ego.heading), sin(ego.heading)];
        P = ego.pos(:).' + (max(ego.speed,0) * t(:)) .* fwd;
        H = repmat(ego.heading, T, 1);
end
end
