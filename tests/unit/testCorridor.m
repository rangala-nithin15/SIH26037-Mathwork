function tests = testCorridor
%TESTCORRIDOR Unit tests for occupancy, ray casting and corridor extraction.
%
%   COMPONENT STATUS: REAL
%
%   Run with:  results = runtests('testCorridor')
%
%   The corridor stage is the core of the IR-PSC claim, so these tests check
%   the properties that claim depends on: that boundaries are found from
%   free space alone, that the centreline sits between them, and that
%   unknown space is treated as not drivable.
%
%   NOT YET EXECUTED. MATLAB is not installed on the machine where this was
%   written. Run on the destination laptop and report the real result.
%
%   Requires: base MATLAB only.

tests = functiontests(localfunctions);
end

% =====================================================================
function g = straightRoadGrid()
%STRAIGHTROADGRID A 60 m straight road, 6 m wide, centred on y = 0.
g = buildRoadGrid([0 0; 60 0], 3.0, 0.2);
end

% =====================================================================
function testGridCreation(tc)
g = makeOccupancyGrid(10, 20, 0.5, [-1 -2]);
verifySize(tc, g.occ, [10 20]);
verifyEqual(tc, g.resolution, 0.5);
verifyEqual(tc, g.origin, [-1 -2]);
verifyFalse(tc, any(g.occ(:)));      % starts entirely free
end

function testOutsideGridIsOccupied(tc)
% Unknown space must be treated as NOT drivable -- this is the fail-safe
% behaviour the planner depends on at the edge of sensor coverage.
g = makeOccupancyGrid(10, 10, 1.0, [0 0]);
occ = isOccupiedAt(g, [-100 -100; 500 500]);
verifyTrue(tc, all(occ));
end

function testOccupancyQueryRoundTrip(tc)
g = makeOccupancyGrid(10, 10, 1.0, [0 0]);
g.occ(5, 7) = true;
% Cell (5,7) covers world x = 6, y = 4 under the documented convention.
verifyTrue(tc,  isOccupiedAt(g, [6 4]));
verifyFalse(tc, isOccupiedAt(g, [1 1]));
end

function testRayCastFindsBoundary(tc)
g = straightRoadGrid();
% From the road centre, casting +y must hit the edge at about 3 m.
d = rayCastGrid(g, [30 0], pi/2, 12, 0.05);
verifyEqual(tc, d, 3.0, 'AbsTol', 0.25);
end

function testRayCastReturnsMaxWhenClear(tc)
g = makeOccupancyGrid(200, 200, 0.2, [0 0]);   % entirely free
d = rayCastGrid(g, [20 20], 0, 10, 0.1);
verifyEqual(tc, d, 10, 'AbsTol', 1e-9);
end

function testRayCastFromOccupiedOriginIsZero(tc)
g = makeOccupancyGrid(50, 50, 0.2, [0 0]);
g.occ(:) = true;
d = rayCastGrid(g, [5 5], 0, 10, 0.1);
verifyEqual(tc, d, 0, 'AbsTol', 1e-9);
end

function testBuildRoadGridCentreIsFree(tc)
g = straightRoadGrid();
verifyFalse(tc, isOccupiedAt(g, [30 0]));
end

function testBuildRoadGridOutsideRoadIsOccupied(tc)
g = straightRoadGrid();
verifyTrue(tc, isOccupiedAt(g, [30 6]));    % well beyond the 3 m half-width
end

function testCorridorOnStraightRoad(tc)
g   = straightRoadGrid();
cfg = irpscConfig('urban');
ego = makeEgoState([5 0], 0, 5.0);

corr = extractCorridor(g, ego, cfg, []);

verifyTrue(tc, corr.valid);
verifyGreaterThan(tc, corr.length, 10);
% A 6 m road, minus the configured erosion on each side.
verifyEqual(tc, mean(corr.width), 6.0 - 2*cfg.corridor.boundaryErode, ...
            'AbsTol', 0.8);
end

function testCorridorCentrelineIsCentred(tc)
% The corridor centre must sit near y = 0 on a road centred on y = 0. This
% is the recentring step that stops the greedy seed hugging one side.
g   = straightRoadGrid();
cfg = irpscConfig('urban');
ego = makeEgoState([5 0], 0, 5.0);

corr = extractCorridor(g, ego, cfg, []);
verifyLessThan(tc, max(abs(corr.center(:,2))), 0.6);
end

function testCorridorQualityInRange(tc)
g   = straightRoadGrid();
cfg = irpscConfig('urban');
ego = makeEgoState([5 0], 0, 5.0);
corr = extractCorridor(g, ego, cfg, []);
verifyGreaterThanOrEqual(tc, corr.quality, 0);
verifyLessThanOrEqual(tc, corr.quality, 1);
end

function testCorridorBlockedRoadIsInvalid(tc)
% A fully occupied grid has no drivable space, so the corridor must report
% itself invalid rather than inventing one.
g = makeOccupancyGrid(200, 200, 0.2, [-5 -20]);
g.occ(:) = true;
cfg = irpscConfig('urban');
ego = makeEgoState([5 0], 0, 5.0);

corr = extractCorridor(g, ego, cfg, []);
verifyFalse(tc, corr.valid);
verifyEqual(tc, corr.quality, 0);
end

function testCorridorBoundsRespectVehicleWidth(tc)
g   = straightRoadGrid();
cfg = irpscConfig('urban');
vp  = vehicleParams(cfg);
ego = makeEgoState([5 0], 0, 5.0);

corr = extractCorridor(g, ego, cfg, []);
[dMin, dMax] = corridorBounds(corr, vp.halfWidth, cfg.deform.boundaryMargin);

verifyTrue(tc, all(dMax >= 0));
verifyTrue(tc, all(dMin <= 0));
% On a 6 m road with a 1.68 m vehicle there must be usable lateral room.
verifyGreaterThan(tc, max(dMax), 0.5);
end

function testCorridorBoundsCollapseWhenTooNarrow(tc)
% A road narrower than the vehicle plus margins must report zero room, not
% a negative or fabricated allowance.
% Phase 2: "zero room" is now a zero-WIDTH band (dMin == dMax) at the best
% centred offset, rather than a band forced to contain d = 0 (which was
% wrong whenever the centreline was not exactly centred). The corridor must
% also be flagged blocked and invalid.
g   = buildRoadGrid([0 0; 40 0], 0.8, 0.1);
cfg = irpscConfig('urban');
vp  = vehicleParams(cfg);
ego = makeEgoState([2 0], 0, 2.0);

corr = extractCorridor(g, ego, cfg, []);
[dMin, dMax] = corridorBounds(corr, vp.halfWidth, cfg.deform.boundaryMargin);
verifyTrue(tc, all(dMax - dMin == 0));
verifyTrue(tc, all(abs(dMin) < 0.2));
verifyTrue(tc, all(corr.blocked));
verifyFalse(tc, corr.valid);
end

function testNarrowButPassableGapIsNotBlocked(tc)
% Phase 2: a gap wider than the body plus the hard minimum clearance is
% NARROW (drive slowly) but not BLOCKED, and the corridor stays valid.
cfg = irpscConfig('village');
g   = buildRoadGrid([0 0; 60 0], 1.30, 0.1);        % 2.6 m free width
ego = makeEgoState([2 0], 0, 3.0);
corr = extractCorridor(g, ego, cfg, []);
verifyTrue(tc, corr.valid);
verifyFalse(tc, any(corr.blocked));
verifyTrue(tc, any(corr.narrow));
end

function testExtractCenterlineIsMidpoint(tc)
L = [(0:10).', repmat(2, 11, 1)];
R = [(0:10).', repmat(-2, 11, 1)];
C = extractCenterline(L, R, 0);
verifyEqual(tc, C(:,2), zeros(11,1), 'AbsTol', 1e-12);
end
