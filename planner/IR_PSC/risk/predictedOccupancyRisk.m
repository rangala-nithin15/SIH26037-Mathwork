function [risk, worstId] = predictedOccupancyRisk(queryPos, queryTime, preds, egoRadius, cfg)
%PREDICTEDOCCUPANCYRISK Risk of occupying a point in space at a given time.
%
%   COMPONENT STATUS: REAL
%
%   [risk, worstId] = PREDICTEDOCCUPANCYRISK(queryPos, queryTime, preds,
%   egoRadius, cfg) evaluates how dangerous it would be for the ego vehicle
%   to be at each query point at the corresponding query time, given the
%   predicted future occupancy of every tracked road user.
%
%   IR-PSC steps 4 and 6: predicted occupancy, and conflict-risk estimation.
%
%   Model
%   -----
%   Each predicted road user occupies, at each instant, an ellipse centred
%   on its predicted mean position and aligned with its predicted heading.
%   The ellipse semi-axes are
%
%       a_long = egoRadius + obsHalfLength + n * sigmaLong
%       a_lat  = egoRadius + obsHalfWidth  + n * sigmaLat
%
%   so the danger zone grows automatically when the prediction is uncertain.
%   This is the mechanism by which uncertainty changes behaviour rather than
%   merely being reported: an erratic motorcycle with a large sigmaLat
%   produces a wide ellipse and the planner gives it a wider berth, with no
%   special-case code anywhere.
%
%   Normalised distance to the ellipse centre is
%
%       e = sqrt( (along/a_long)^2 + (lat/a_lat)^2 )
%
%   and the raw risk is max(0, 1 - e): 1 at the centre, falling linearly to
%   0 at the ellipse boundary. The result is multiplied by the class risk
%   weight (vulnerable road users score higher) and by the track confidence.
%
%   Risks from several road users are combined with a probabilistic OR,
%   1 - prod(1 - r_k), rather than a sum. Two separate 0.6 hazards should
%   not produce a risk of 1.2.
%
%   Inputs:
%       queryPos  - Qx2 world points to evaluate
%       queryTime - Qx1 (or scalar) times in seconds at which the ego would
%                   be at those points, measured from the planning instant
%       preds     - prediction struct array from predictObstacles()
%       egoRadius - scalar, ego footprint disc radius (m)
%       cfg       - config struct from irpscConfig()
%
%   Outputs:
%       risk    - Qx1 risk in [0, 1]
%       worstId - Qx1 track id of the largest single contributor, NaN if
%                 no road user contributes at that point
%
%   Example:
%       r = predictedOccupancyRisk(traj.pos, traj.times, preds, vp.discRadius, cfg);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTOBSTACLES, CONFLICTRISK, LATERALRISKGRID.

validateattributes(queryPos, {'numeric'}, {'2d','ncols',2,'finite','real'}, ...
                   mfilename, 'queryPos');

Q = size(queryPos,1);
if isscalar(queryTime)
    queryTime = repmat(queryTime, Q, 1);
end
queryTime = queryTime(:);

risk    = zeros(Q,1);
worstId = nan(Q,1);

if isempty(preds)
    return;
end

nSig     = cfg.risk.nSigmaOccupancy;
sigScale = cfg.risk.sigmaScale;
floorVal = cfg.risk.riskFloor;

survive  = ones(Q,1);    % running product of (1 - r_k)
worstVal = zeros(Q,1);

for k = 1:numel(preds)
    p = preds(k);

    % Predicted mean, sigma and heading at each query time.
    tp = p.times(:);
    if numel(tp) < 2
        continue;
    end
    tq = min(max(queryTime, tp(1)), tp(end));    % clamp, never extrapolate

    mx  = interp1(tp, p.pos(:,1),   tq, 'linear');
    my  = interp1(tp, p.pos(:,2),   tq, 'linear');
    sL  = interp1(tp, p.sigmaLong,  tq, 'linear') * sigScale;
    sT  = interp1(tp, p.sigmaLat,   tq, 'linear') * sigScale;
    % Interpolate heading through its unit vector to avoid the +/-pi seam.
    hx  = interp1(tp, cos(p.heading), tq, 'linear');
    hy  = interp1(tp, sin(p.heading), tq, 'linear');
    th  = atan2(hy, hx);

    dx = queryPos(:,1) - mx;
    dy = queryPos(:,2) - my;

    ct = cos(th);  st = sin(th);
    along =  dx .* ct + dy .* st;
    lat   = -dx .* st + dy .* ct;

    % Phase 2: the object's own extent is its half-length along its heading
    % and its half-width across it. The earlier code used the enclosing
    % radius hypot(L,W)/2 on BOTH axes, which made a 2.5 m wide bus look
    % 11 m wide laterally and blocked adjacent lanes that are clear.
    if isfield(p, 'halfLength') && ~isempty(p.halfLength)
        hL = p.halfLength;  hW = p.halfWidth;
    else
        hL = p.radius;      hW = p.radius;
    end
    aLong = egoRadius + hL + nSig * sL;
    aLat  = egoRadius + hW + nSig * sT;
    aLong = max(aLong, 1e-3);
    aLat  = max(aLat,  1e-3);

    e  = sqrt((along ./ aLong).^2 + (lat ./ aLat).^2);
    rk = max(0, 1 - e);

    rk = rk * p.weight * max(min(p.confidence, 1), 0);
    rk = min(rk, 1);
    rk(rk < floorVal) = 0;

    survive = survive .* (1 - rk);

    better = rk > worstVal;
    worstVal(better) = rk(better);
    worstId(better)  = p.id;
end

risk = 1 - survive;
risk = max(0, min(1, risk));
worstId(risk <= floorVal) = NaN;
end
