function d = boxDistance(A, B)
%BOXDISTANCE Signed distance between two convex quadrilaterals.
%
%   COMPONENT STATUS: REAL
%
%   d = BOXDISTANCE(A, B) returns the minimum Euclidean distance between the
%   convex polygons A and B when they are separated, and MINUS the smallest
%   separating-axis overlap when they intersect. So d > 0 means a gap of d
%   metres, d <= 0 means contact or penetration.
%
%   Why this replaced the enclosing-circle test
%   -------------------------------------------
%   The earlier clearance and collision checks treated every road user as a
%   circle of radius hypot(length,width)/2. For a bus that is a 5.6 m radius
%   around an object only 1.3 m wide, which made it impossible to pass a bus
%   in an adjacent lane without "violating clearance". Exact rectangle
%   distance removes that artefact without loosening any safety margin.
%
%   Inputs:
%       A, B - Kx2 vertex lists of convex polygons, in consistent winding
%              (BOXCORNERS output qualifies)
%
%   Outputs:
%       d - scalar signed distance in metres
%
%   Example:
%       d = boxDistance(boxCorners([0 0],0,4,2), boxCorners([6 0],0,4,2)); % 2
%
%   Requires: base MATLAB only.
%
%   See also BOXCORNERS, CHECKCLEARANCE.

% Separating axis test: edge normals of both polygons.
minOverlap = Inf;
separated  = false;
polys = {A, B};
for p = 1:2
    P = polys{p};
    K = size(P,1);
    for e = 1:K
        edge = P(mod(e, K) + 1, :) - P(e, :);
        n = [-edge(2), edge(1)];
        nn = hypot(n(1), n(2));
        if nn < 1e-12, continue; end
        n = n / nn;
        pa = A * n.';
        pb = B * n.';
        overlap = min(max(pa), max(pb)) - max(min(pa), min(pb));
        if overlap < 0
            separated = true;
            break;
        end
        minOverlap = min(minOverlap, overlap);
    end
    if separated, break; end
end

if ~separated
    d = -minOverlap;
    return;
end

% Separated convex polygons: the minimum distance is attained between a
% vertex of one and an edge of the other.
d = min(pointsToPolygonEdges(A, B), pointsToPolygonEdges(B, A));
end

function dMin = pointsToPolygonEdges(pts, poly)
K = size(poly,1);
dMin = Inf;
for e = 1:K
    a  = poly(e, :);
    b  = poly(mod(e, K) + 1, :);
    ab = b - a;
    L2 = ab(1)^2 + ab(2)^2;
    if L2 < 1e-12
        t = zeros(size(pts,1),1);
    else
        t = ((pts(:,1) - a(1)) * ab(1) + (pts(:,2) - a(2)) * ab(2)) / L2;
        t = min(max(t, 0), 1);
    end
    fx = a(1) + t * ab(1);
    fy = a(2) + t * ab(2);
    dd = sqrt((pts(:,1) - fx).^2 + (pts(:,2) - fy).^2);
    dMin = min(dMin, min(dd));
end
end
