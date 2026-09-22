function C = boxCorners(center, heading, len, wid)
%BOXCORNERS Corners of an oriented rectangle (a vehicle or object footprint).
%
%   COMPONENT STATUS: REAL
%
%   C = BOXCORNERS(center, heading, len, wid) returns the four corners of a
%   rectangle of length `len` (along `heading`) and width `wid`, centred on
%   `center`, in counter-clockwise order starting front-left.
%
%   Inputs:
%       center  - 1x2 [x y] world position of the rectangle CENTRE
%       heading - scalar yaw in radians
%       len     - scalar, extent along the heading (m)
%       wid     - scalar, extent across the heading (m)
%
%   Outputs:
%       C - 4x2 corner coordinates [front-left; rear-left; rear-right; front-right]
%
%   Example:
%       C = boxCorners([0 0], 0, 4, 2);   % [2 1; -2 1; -2 -1; 2 -1]
%
%   Requires: base MATLAB only.
%
%   See also BOXDISTANCE, EGOFOOTPRINT.

center = center(:).';
ct = cos(heading);  st = sin(heading);
hl = len / 2;  hw = wid / 2;
local = [ hl  hw;
         -hl  hw;
         -hl -hw;
          hl -hw];
C = [local(:,1)*ct - local(:,2)*st + center(1), ...
     local(:,1)*st + local(:,2)*ct + center(2)];
end
