function tests = testGeometry
%TESTGEOMETRY Unit tests for the geometry utilities.
%
%   COMPONENT STATUS: REAL
%
%   Run with:  results = runtests('testGeometry')
%
%   These tests check closed-form cases where the correct answer is known
%   analytically, plus the degenerate inputs that break naive
%   implementations: single points, coincident points, zero-length paths and
%   the +/-pi angle wrap.
%
%   NOT YET EXECUTED. This suite has never been run, because MATLAB is not
%   installed on the machine where this project was written. Run it on the
%   destination laptop and report the real result. No test in this project
%   is described as passing until it has actually passed.
%
%   Requires: base MATLAB only (matlab.unittest is part of base MATLAB).

tests = functiontests(localfunctions);
end

% =====================================================================
function testArcLengthStraightLine(tc)
s = pathArcLength([0 0; 3 4]);
verifyEqual(tc, s, [0; 5], 'AbsTol', 1e-12);
end

function testArcLengthThreePoints(tc)
s = pathArcLength([0 0; 3 4; 3 8]);
verifyEqual(tc, s, [0; 5; 9], 'AbsTol', 1e-12);
end

function testArcLengthSinglePoint(tc)
s = pathArcLength([5 7]);
verifyEqual(tc, s, 0);
end

function testResampleUniformSpacing(tc)
Q = resamplePath([0 0; 10 0], 5);
verifySize(tc, Q, [5 2]);
verifyEqual(tc, Q(:,1), (0:2.5:10).', 'AbsTol', 1e-9);
verifyEqual(tc, Q(:,2), zeros(5,1), 'AbsTol', 1e-9);
end

function testResampleZeroLengthPath(tc)
% All points identical: must not produce NaN.
Q = resamplePath([2 3; 2 3; 2 3], 4);
verifySize(tc, Q, [4 2]);
verifyFalse(tc, any(isnan(Q(:))));
verifyEqual(tc, Q, repmat([2 3], 4, 1), 'AbsTol', 1e-12);
end

function testResampleSinglePoint(tc)
Q = resamplePath([1 1], 3);
verifyEqual(tc, Q, repmat([1 1], 3, 1));
end

function testHeadingAlongXAxis(tc)
th = pathHeading([0 0; 1 0; 2 0]);
verifyEqual(tc, th, zeros(3,1), 'AbsTol', 1e-12);
end

function testHeadingAlongYAxis(tc)
th = pathHeading([0 0; 0 1; 0 2]);
verifyEqual(tc, th, repmat(pi/2, 3, 1), 'AbsTol', 1e-12);
end

function testCurvatureOfCircle(tc)
% A circle of radius R must have curvature 1/R everywhere.
R  = 10;
th = linspace(0, pi/2, 60).';
P  = [R*cos(th), R*sin(th)];
k  = pathCurvature(P);
% Interior points only; endpoints are copied from their neighbours.
verifyEqual(tc, abs(k(5:end-5)), repmat(1/R, numel(k)-9, 1), 'AbsTol', 5e-3);
end

function testCurvatureOfStraightLine(tc)
k = pathCurvature([0 0; 1 0; 2 0; 3 0]);
verifyEqual(tc, k, zeros(4,1), 'AbsTol', 1e-9);
end

function testCurvatureCoincidentPointsNoNaN(tc)
% Degenerate triple must give 0, not Inf or NaN.
k = pathCurvature([0 0; 0 0; 0 0; 1 0]);
verifyFalse(tc, any(isnan(k)));
verifyFalse(tc, any(isinf(k)));
end

function testWrapToPiBasic(tc)
verifyEqual(tc, wrapToPiLocal(0), 0, 'AbsTol', 1e-12);
verifyEqual(tc, wrapToPiLocal(3*pi), pi, 'AbsTol', 1e-12);
verifyEqual(tc, wrapToPiLocal(-3*pi), pi, 'AbsTol', 1e-12);
verifyEqual(tc, wrapToPiLocal(pi/2), pi/2, 'AbsTol', 1e-12);
end

function testWrapToPiIntervalIsHalfOpen(tc)
% The documented interval is (-pi, pi], so -pi must map to +pi.
verifyEqual(tc, wrapToPiLocal(-pi), pi, 'AbsTol', 1e-12);
end

function testProjectPointLateralSign(tc)
% Path runs along +x. A point at +y is to the LEFT, so d must be positive.
[s, d] = projectPointOnPath([0 0; 10 0], [4 2]);
verifyEqual(tc, s, 4, 'AbsTol', 1e-9);
verifyEqual(tc, d, 2, 'AbsTol', 1e-9);

[~, dRight] = projectPointOnPath([0 0; 10 0], [4 -2]);
verifyEqual(tc, dRight, -2, 'AbsTol', 1e-9);
end

function testProjectPointClampsToSegment(tc)
% A point beyond the end of the path projects onto the endpoint.
[s, ~, ~, foot] = projectPointOnPath([0 0; 10 0], [15 0]);
verifyEqual(tc, s, 10, 'AbsTol', 1e-9);
verifyEqual(tc, foot, [10 0], 'AbsTol', 1e-9);
end

function testFrenetRoundTrip(tc)
% Cartesian -> Frenet -> Cartesian must return the original point.
P  = [0 0; 5 0; 10 2; 15 6];
pt = [7 3];
[s, d] = projectPointOnPath(P, pt);
Q = frenetToCartesian(P, s, d);
verifyEqual(tc, Q, pt, 'AbsTol', 1e-6);
end

function testFrenetToCartesianOffsetIsLeft(tc)
% Along +x, a positive offset must move the point to +y.
Q = frenetToCartesian([0 0; 10 0], [0; 5; 10], 1.0);
verifyEqual(tc, Q(:,2), ones(3,1), 'AbsTol', 1e-9);
end

function testSmoothPathLocksEnds(tc)
P = [0 0; 1 5; 2 -5; 3 5; 4 0];
Q = smoothPath(P, 2, 2, true);
verifyEqual(tc, Q(1,:),   P(1,:),   'AbsTol', 1e-12);
verifyEqual(tc, Q(end,:), P(end,:), 'AbsTol', 1e-12);
end

function testSmoothPathReducesVariation(tc)
P = [(0:10).', 5*(-1).^(0:10).'];       % maximum zig-zag
Q = smoothPath(P, 2, 2, true);
verifyLessThan(tc, sum(abs(diff(Q(:,2)))), sum(abs(diff(P(:,2)))));
end

function testSmoothSeriesPreservesLength(tc)
x = randn(20,1);
y = smoothSeries(x, 3, 2);
verifySize(tc, y, [20 1]);
end

function testSmoothSeriesConstantIsUnchanged(tc)
x = repmat(4.2, 15, 1);
y = smoothSeries(x, 3, 2);
verifyEqual(tc, y, x, 'AbsTol', 1e-12);
end
