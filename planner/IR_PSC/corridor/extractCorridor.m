function corridor = extractCorridor(grid, ego, cfg, priorCenter)
%EXTRACTCORRIDOR Build the drivable safety corridor from free space.
%
%   COMPONENT STATUS: REAL
%
%   corridor = EXTRACTCORRIDOR(grid, ego, cfg, priorCenter) estimates the
%   left and right drivable boundaries ahead of the vehicle and the
%   continuous centreline between them.
%
%   This function is the centre of the IR-PSC contribution. It implements
%   the principle that gives the project its name:
%
%       "Lane is an optional cue; drivable space is the primary
%        planning constraint."
%
%   Nothing here looks for a painted line. Boundaries are found by casting
%   rays perpendicular to a seed path until free space ends. On a village
%   road with no markings at all, this still produces a usable corridor.
%
%   IR-PSC steps 1 and 2: estimate drivable space, identify boundaries.
%
%   Algorithm
%   ---------
%     1. SEEDCENTERLINE marches through free space to guess the road course.
%     2. Resample the seed to uniform stations.
%     3. At each station cast one ray left and one ray right, perpendicular
%        to the seed heading, until free space ends.
%     4. Erode both boundaries inward by cfg.corridor.boundaryErode so the
%        corridor never touches the exact edge of measured free space.
%     5. Recompute the centreline as the midpoint of the eroded boundaries,
%        then smooth it. This corrects the seed's greedy lateral bias.
%     6. Blend with the previous frame's centreline for temporal stability,
%        ALIGNED IN SPACE (see below).
%     7. Classify every station as clear, NARROW or BLOCKED and truncate
%        the usable corridor at the first blocked station.
%
%   Why step 5 matters: the seed hugs whichever side had more space. Taking
%   the midpoint of the measured boundaries recentres the corridor, which is
%   what makes the preferred trajectory road-following rather than
%   free-space-hugging.
%
%   Temporal blending (Phase 2 fix)
%   -------------------------------
%   The vehicle moves between planning cycles, so station i of the previous
%   centreline is NOT the same place as station i of the new one. The old
%   code blended them index by index, dragging the corridor backwards and
%   sideways on bends. Each new station is now blended towards the closest
%   point of the previous centreline (a purely lateral correction), and only
%   where the previous centreline actually covers that stretch of road.
%
%   Narrow and blocked stations (Phase 2 fix)
%   -----------------------------------------
%   Previously one station narrower than cfg.corridor.minWidth ANYWHERE in
%   the lookahead made the whole corridor invalid, which triggered an
%   emergency stop tens of metres before a pinch point. Now:
%     - NARROW  : eroded width < cfg.corridor.minWidth. Passable slowly with
%                 reduced side margins (see corridorBounds, irpscPlanner).
%     - BLOCKED : measured free width < ego width + 2 * the hard minimum
%                 clearance cfg.safety.minLateralClearance. The body cannot
%                 fit. The corridor is truncated here (usableLength) and the
%                 planner plans a controlled stop before it.
%   The safety margins themselves are unchanged.
%
%   Inputs:
%       grid        - occupancy grid struct from makeOccupancyGrid()
%       ego         - ego state struct from makeEgoState()
%       cfg         - config struct from irpscConfig()
%       priorCenter - (optional) Mx2 centreline from the previous planning
%                     cycle, used for temporal smoothing. Pass [] on the
%                     first cycle.
%
%   Outputs:
%       corridor - struct with fields:
%           .center      Nx2 centreline points (world frame)
%           .left        Nx2 left boundary points
%           .right       Nx2 right boundary points
%           .s           Nx1 arc length along the centreline
%           .heading     Nx1 centreline heading (rad)
%           .width       Nx1 eroded corridor width at each station (m)
%           .rawWidth    Nx1 measured free width before erosion (m)
%           .leftDist    Nx1 signed distance from centreline to left boundary
%           .rightDist   Nx1 signed distance from centreline to right boundary
%           .leftObserved  Nx1 logical, false where the ray found no edge
%           .rightObserved Nx1 logical, same for the right side
%           .narrow      Nx1 logical, passable only slowly
%           .blocked     Nx1 logical, the vehicle body does not fit
%           .usableLength scalar, arc length to the first blocked station
%           .blockedIdx  index of the first blocked station ([] if none)
%           .minWidth    scalar, narrowest point (m)
%           .valid       logical, true if a usable corridor exists
%           .quality     0..1 corridor confidence (see below)
%           .length      scalar, corridor length in m
%           .stalled     logical, from the seed march
%
%   Quality score
%   -------------
%   quality combines three things that are all genuinely measurable here:
%     - how much of the requested lookahead distance was achieved,
%     - what fraction of stations are at least minWidth wide,
%     - what fraction of boundary rays actually found an edge.
%   A ray that runs to maxRayLength without hitting anything means the road
%   edge was NOT observed (open field, missing return, sensor gap). Treating
%   that as a confident boundary would be dishonest, so it lowers quality
%   and, through it, the planner's confidence. This is what feeds
%   confidence-aware behaviour downstream.
%
%   Example:
%       corridor = extractCorridor(grid, ego, irpscConfig('village'), []);
%
%   Requires: base MATLAB only.
%
%   See also SEEDCENTERLINE, EXTRACTCENTERLINE, CORRIDORBOUNDS, IRPSCPLANNER.

if nargin < 4, priorCenter = []; end

c = cfg.corridor;

% --- 1. Seed through free space -------------------------------------
[seed, seedInfo] = seedCenterline(grid, ego, cfg);

if size(seed,1) < 2
    corridor = emptyCorridor(ego, seedInfo);
    return;
end

% --- 2. Uniform stations ----------------------------------------------
sSeed  = pathArcLength(seed);
nSta   = max(2, floor(sSeed(end) / c.stationStep) + 1);
seedR  = resamplePath(seed, nSta);
seedR  = smoothPath(seedR, 2, 1, true);       % light pre-smoothing
thSeed = pathHeading(seedR);

N          = size(seedR,1);
leftPt     = zeros(N,2);
rightPt    = zeros(N,2);
rawWidth   = zeros(N,1);
leftObs    = false(N,1);
rightObs   = false(N,1);

% --- 3-4. Perpendicular boundary rays, eroded ----------------------------
% All left and right rays in one vectorised query (Phase 2, speed).
dLR = rayCastGridMulti(grid, [seedR; seedR], [thSeed + pi/2; thSeed - pi/2], ...
                       c.maxRayLength, c.rayStep);
for i = 1:N
    hL = thSeed(i) + pi/2;    % left normal
    hR = thSeed(i) - pi/2;    % right normal

    dL = dLR(i);
    dR = dLR(N + i);

    leftObs(i)  = dL < c.maxRayLength - 1e-6;
    rightObs(i) = dR < c.maxRayLength - 1e-6;
    rawWidth(i) = dL + dR;

    dLe = max(dL - c.boundaryErode, 0);
    dRe = max(dR - c.boundaryErode, 0);

    leftPt(i,:)  = seedR(i,:) + dLe * [cos(hL), sin(hL)];
    rightPt(i,:) = seedR(i,:) + dRe * [cos(hR), sin(hR)];
end

% --- 5. Midpoint centreline ---------------------------------------------
centerNow = extractCenterline(leftPt, rightPt, c.centerlineSmooth);
centerNow(1,:) = 0.5 * (leftPt(1,:) + rightPt(1,:));

% --- 6. Spatially aligned temporal blending -------------------------------
center = centerNow;
if ~isempty(priorCenter) && size(priorCenter,1) >= 2 && cfg.ablation.useTemporalSmoothing
    a  = cfg.deform.temporalAlpha;           % 1 = ignore history
    sPrior = pathArcLength(priorCenter);
    for i = 1:N
        [sp, ~, ~, foot] = projectPointOnPath(priorCenter, centerNow(i,:));
        % Only where the previous centreline genuinely covers this place;
        % a projection clamped to either end is not the same stretch of road.
        if sp > 1e-3 && sp < sPrior(end) - 1e-3
            center(i,:) = a * centerNow(i,:) + (1 - a) * foot;
        end
    end
end

center = resamplePath(center, N);
sCen   = pathArcLength(center);
thCen  = pathHeading(center);

% Signed distances along the centreline normal. If blending pushed a
% station outside its own boundaries, fall back to the unblended midpoint.
lDist = zeros(N,1);
rDist = zeros(N,1);
for i = 1:N
    nL = [-sin(thCen(i)), cos(thCen(i))];
    lDist(i) = (leftPt(i,:)  - center(i,:)) * nL.';
    rDist(i) = -(rightPt(i,:) - center(i,:)) * nL.';
    if lDist(i) < 0.05 || rDist(i) < 0.05
        center(i,:) = 0.5 * (leftPt(i,:) + rightPt(i,:));
        lDist(i) = (leftPt(i,:)  - center(i,:)) * nL.';
        rDist(i) = -(rightPt(i,:) - center(i,:)) * nL.';
    end
end
lDist = max(lDist, 0);
rDist = max(rDist, 0);
sCen  = pathArcLength(center);
thCen = pathHeading(center);
width = lDist + rDist;

% --- 7. Narrow / blocked classification ----------------------------------
passWidth = cfg.ego.width + 2 * cfg.safety.minLateralClearance;
blocked   = rawWidth < passWidth;
narrow    = ~blocked & (width < c.minWidth);

blockedIdx = find(blocked, 1, 'first');
if isempty(blockedIdx)
    usableLength = sCen(end);
else
    usableLength = sCen(blockedIdx);
end

corridor.center        = center;
corridor.left          = leftPt;
corridor.right         = rightPt;
corridor.s             = sCen;
corridor.heading       = thCen;
corridor.width         = width;
corridor.rawWidth      = rawWidth;
corridor.leftDist      = lDist;
corridor.rightDist     = rDist;
corridor.leftObserved  = leftObs;
corridor.rightObserved = rightObs;
corridor.narrow        = narrow;
corridor.blocked       = blocked;
corridor.usableLength  = usableLength;
corridor.blockedIdx    = blockedIdx;
corridor.minWidth      = min(width);
corridor.length        = sCen(end);
corridor.stalled       = seedInfo.stalled;

lengthScore   = min(1, corridor.length / max(c.lookaheadDist, eps));
widthScore    = mean(width >= c.minWidth);
observedScore = mean([leftObs; rightObs]);

corridor.quality = max(0, min(1, ...
    0.40 * lengthScore + 0.35 * widthScore + 0.25 * observedScore));

corridor.valid = usableLength >= c.minUsableLength && ...
                 corridor.length >= 2 * c.stationStep;
end

% -------------------------------------------------------------------------
function corridor = emptyCorridor(ego, seedInfo)
p = ego.pos(:).';
corridor.center        = [p; p];
corridor.left          = [p; p];
corridor.right         = [p; p];
corridor.s             = [0; 0];
corridor.heading       = [ego.heading; ego.heading];
corridor.width         = [0; 0];
corridor.rawWidth      = [0; 0];
corridor.leftDist      = [0; 0];
corridor.rightDist     = [0; 0];
corridor.leftObserved  = [false; false];
corridor.rightObserved = [false; false];
corridor.narrow        = [false; false];
corridor.blocked       = [true; true];
corridor.usableLength  = 0;
corridor.blockedIdx    = 1;
corridor.minWidth      = 0;
corridor.length        = 0;
corridor.stalled       = seedInfo.stalled;
corridor.quality       = 0;
corridor.valid         = false;
end
