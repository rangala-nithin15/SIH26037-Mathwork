function [C, center] = egoFootprint(rearAxlePos, heading, vp)
%EGOFOOTPRINT Corners of the ego body rectangle from its rear-axle pose.
%
%   COMPONENT STATUS: REAL
%
%   [C, center] = EGOFOOTPRINT(rearAxlePos, heading, vp) converts the
%   project's rear-axle reference point into the body rectangle. The body
%   centre lies (length/2 - rearOverhang) ahead of the rear axle.
%
%   The earlier plotting code drew the ego rectangle centred on the rear
%   axle, 1.3 m behind the real body. Every drawing, clearance check and
%   collision check now goes through this single function.
%
%   Inputs:
%       rearAxlePos - 1x2 [x y]
%       heading     - scalar yaw, rad
%       vp          - struct from vehicleParams()
%
%   Outputs:
%       C      - 4x2 corners (see BOXCORNERS)
%       center - 1x2 body centre
%
%   Requires: base MATLAB only.
%
%   See also BOXCORNERS, VEHICLEPARAMS.

fwdOff = vp.length/2 - vp.rearOverhang;
center = rearAxlePos(:).' + fwdOff * [cos(heading), sin(heading)];
C = boxCorners(center, heading, vp.length, vp.width);
end
