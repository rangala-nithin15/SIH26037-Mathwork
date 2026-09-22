function a = wrapToPiLocal(a)
%WRAPTOPILOCAL Wrap angles to the interval (-pi, pi].
%
%   COMPONENT STATUS: REAL
%
%   a = WRAPTOPILOCAL(a) wraps each element of a into (-pi, pi].
%
%   Why not use wrapToPi
%   --------------------
%   MATLAB's wrapToPi ships with Mapping Toolbox. This project deliberately
%   keeps its numerical core dependent on base MATLAB alone, so that every
%   core function can be unit-tested on the destination laptop even before
%   toolbox licensing is settled. This local version is used throughout.
%
%   Inputs:
%       a - numeric array of angles in radians
%
%   Outputs:
%       a - same size, wrapped to (-pi, pi]
%
%   Example:
%       wrapToPiLocal(3*pi)   % -> pi
%
%   Requires: base MATLAB only.

validateattributes(a, {'numeric'}, {'real'}, mfilename, 'a');
a = mod(a + pi, 2*pi) - pi;
% mod maps exactly -pi to -pi; move it to +pi so the interval is (-pi, pi].
a(a == -pi) = pi;
end
