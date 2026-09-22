function c = truncateCorridor(c, lastIdx)
%TRUNCATECORRIDOR Keep only stations 1..lastIdx of a corridor.
%
%   COMPONENT STATUS: REAL
%
%   c = TRUNCATECORRIDOR(c, lastIdx) returns the corridor with every
%   per-station field cut to its first lastIdx stations (at least 2), and
%   .length and .minWidth recomputed. Used by both planners to plan only on
%   the USABLE part of a corridor that is blocked further ahead.
%
%   Requires: base MATLAB only.
%
%   See also EXTRACTCORRIDOR, IRPSCPLANNER, BASELINEPLANNER.

f = {'center','left','right','s','heading','width','rawWidth','leftDist', ...
     'rightDist','leftObserved','rightObserved','narrow','blocked'};
N = size(c.center, 1);
lastIdx = min(max(lastIdx, 2), N);
for i = 1:numel(f)
    if ~isfield(c, f{i}), continue; end
    v = c.(f{i});
    if size(v,1) == N
        c.(f{i}) = v(1:lastIdx, :);
    end
end
c.length   = c.s(end);
c.minWidth = min(c.width);
end
