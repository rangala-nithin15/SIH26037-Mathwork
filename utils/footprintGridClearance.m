function clr = footprintGridClearance(grid, C, searchRadius)
%FOOTPRINTGRIDCLEARANCE Distance from a body rectangle to the nearest occupied cell.
%
%   COMPONENT STATUS: REAL
%
%   clr = FOOTPRINTGRIDCLEARANCE(grid, C, searchRadius) returns the
%   distance in metres between the convex footprint C and the nearest
%   OCCUPIED grid cell within searchRadius of it. Each occupied cell is
%   represented by its centre minus half a cell width (the distance from a
%   cell centre to its edge). Cells inside the footprint give a negative
%   clearance. The grid itself is a discretisation of the world at
%   grid.resolution, so this is accurate to about that resolution; it is
%   not a sub-centimetre geometric guarantee.
%
%   Why this replaced eight-direction ray casting
%   ---------------------------------------------
%   The earlier static check cast 8 rays at 45 degree spacing from three
%   disc centres. At 1.3 m range adjacent rays are ~1 m apart, so a small
%   obstacle (debris, a pole) could sit between two rays unseen, and the
%   discs made the car ~0.23 m wider on each side than it is. This check
%   examines every cell near the true rectangle.
%
%   Outside the grid counts as occupied (fail-safe), consistent with
%   ISOCCUPIEDAT.
%
%   Inputs:
%       grid         - struct from makeOccupancyGrid()
%       C            - Kx2 convex footprint vertices
%       searchRadius - m, how far around the footprint to look
%
%   Outputs:
%       clr - scalar; searchRadius if nothing occupied is that close
%
%   Requires: base MATLAB only.
%
%   See also EGOFOOTPRINT, CHECKCLEARANCE, BOXDISTANCE.

res = grid.resolution;
xMin = min(C(:,1)) - searchRadius;  xMax = max(C(:,1)) + searchRadius;
yMin = min(C(:,2)) - searchRadius;  yMax = max(C(:,2)) + searchRadius;

c0 = floor((xMin - grid.origin(1)) / res) + 1;
c1 = ceil((xMax - grid.origin(1)) / res) + 1;
r0 = floor((yMin - grid.origin(2)) / res) + 1;
r1 = ceil((yMax - grid.origin(2)) / res) + 1;

% Any part of the search window outside the grid is unknown => occupied.
if c0 < 1 || r0 < 1 || c1 > grid.nCols || r1 > grid.nRows
    outside = true;
else
    outside = false;
end
c0 = max(c0, 1);  c1 = min(c1, grid.nCols);
r0 = max(r0, 1);  r1 = min(r1, grid.nRows);

clr = searchRadius;
if c0 <= c1 && r0 <= r1
    sub = grid.occ(r0:r1, c0:c1);
    [rr, cc] = find(sub);
    if ~isempty(rr)
        px = grid.origin(1) + (cc + c0 - 2) * res;
        py = grid.origin(2) + (rr + r0 - 2) * res;
        d  = pointsToConvexSigned([px(:), py(:)], C);
        clr = min(clr, min(d) - res / 2);
    end
end

if outside
    % Distance to the grid edge counts as distance to an obstacle.
    gx0 = grid.origin(1) - res/2;  gx1 = grid.origin(1) + (grid.nCols - 0.5) * res;
    gy0 = grid.origin(2) - res/2;  gy1 = grid.origin(2) + (grid.nRows - 0.5) * res;
    edgeGap = min([min(C(:,1)) - gx0, gx1 - max(C(:,1)), ...
                   min(C(:,2)) - gy0, gy1 - max(C(:,2))]);
    clr = min(clr, edgeGap);
end
end

function d = pointsToConvexSigned(pts, C)
% Signed distance from each point to the convex polygon C (negative inside).
K = size(C,1);
n = size(pts,1);
dEdge  = inf(n,1);
inside = true(n,1);
area2 = sum(C(:,1) .* C([2:K 1],2) - C([2:K 1],1) .* C(:,2));
orient = sign(area2);
if orient == 0, orient = 1; end
for e = 1:K
    a  = C(e, :);
    b  = C(mod(e, K) + 1, :);
    ab = b - a;
    L2 = max(ab(1)^2 + ab(2)^2, 1e-12);
    t  = ((pts(:,1) - a(1)) * ab(1) + (pts(:,2) - a(2)) * ab(2)) / L2;
    t  = min(max(t, 0), 1);
    dd = sqrt((pts(:,1) - a(1) - t*ab(1)).^2 + (pts(:,2) - a(2) - t*ab(2)).^2);
    dEdge = min(dEdge, dd);
    crossZ = ab(1) * (pts(:,2) - a(2)) - ab(2) * (pts(:,1) - a(1));
    inside = inside & (orient * crossZ >= 0);
end
d = dEdge;
d(inside) = -dEdge(inside);
end
