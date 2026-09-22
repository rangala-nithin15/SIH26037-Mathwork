function h = confirmedPotholes(hz)
%CONFIRMEDPOTHOLES Keep only confirmed pothole tracks.
%
%   COMPONENT STATUS: REAL
%
%   h = CONFIRMEDPOTHOLES(hz) returns the elements of the pothole track
%   array hz whose .confirmed flag is true. Planners act only on confirmed
%   potholes, so a single noisy detection cannot swerve the vehicle.
%
%   Requires: base MATLAB only.
%
%   See also POTHOLETRACKER, IRPSCPLANNER.

if isempty(hz)
    h = hz;
    return;
end
keep = false(1, numel(hz));
for i = 1:numel(hz)
    keep(i) = isfield(hz(i), 'confirmed') && logical(hz(i).confirmed);
end
h = hz(keep);
end
