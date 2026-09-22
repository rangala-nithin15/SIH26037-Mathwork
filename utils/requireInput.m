function requireInput(ok, fname, what)
%REQUIREINPUT Cheap input check used in inner-loop utility functions.
%
%   COMPONENT STATUS: REAL
%
%   REQUIREINPUT(ok, fname, what) throws '<fname>:invalidInput' when ok is
%   false. Phase 2 replaced validateattributes with this in the geometry
%   functions the planner calls tens of thousands of times per run:
%   profiling showed validateattributes alone taking more time than all the
%   planning arithmetic. The checks themselves (shape, finiteness) are kept.
%
%   Requires: base MATLAB only.

if ~ok
    error([fname ':invalidInput'], 'Invalid input to %s: %s', fname, what);
end
end
