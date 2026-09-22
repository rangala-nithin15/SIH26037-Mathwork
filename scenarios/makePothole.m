function ph = makePothole(id, pos, len, wid, depth, varargin)
%MAKEPOTHOLE Ground-truth pothole: a first-class road-surface hazard.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION (planar ellipse with a depth)
%
%   ph = MAKEPOTHOLE(id, pos, len, wid, depth, ...) defines one pothole in a
%   scenario. A pothole is NOT an obstacle and is NOT an occupied grid cell:
%   the road over it is drivable. It is a surface hazard that perception
%   must detect and the planner must either avoid or cross slowly.
%
%   Geometry is an ellipse of length `len` along `yaw` and width `wid`,
%   with a maximum depth `depth`. Severity is derived from depth with
%   CLASSIFYPOTHOLESEVERITY and the default thresholds, and stored here only
%   as the TRUE severity for evaluation and display; the planner never sees
%   ground truth, only the tracked estimate from POTHOLETRACKER.
%
%   What is NOT modelled (stated plainly): vehicle ride response. The
%   kinematic bicycle model has no suspension or vertical dynamics, so
%   driving into a pothole produces no jolt, no damage and no loss of
%   control in simulation. Instead the simulation LOGS every wheel entry
%   with its speed (see runScenario), so pothole handling can be evaluated
%   as "entered at what speed" rather than claimed as ride comfort.
%
%   Inputs:
%       id    - numeric scalar, unique pothole id
%       pos   - 1x2 [x y] centre (m)
%       len   - m, extent along yaw
%       wid   - m, extent across yaw
%       depth - m, maximum depth (e.g. 0.03 minor, 0.07 moderate, 0.15 severe)
%
%   Name-value pairs:
%       'Yaw' - rad, orientation of the long axis. Default 0.
%
%   Outputs:
%       ph - struct with fields .id .pos .length .width .yaw .depth
%            .severity (true severity) .riskLevel
%
%   Example:
%       ph = makePothole(1, [70 1.2], 1.4, 1.0, 0.08, 'Yaw', 0.3);
%
%   Requires: base MATLAB only.
%
%   See also CLASSIFYPOTHOLESEVERITY, DETECTPOTHOLES, POTHOLETRACKER.

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'Yaw', 0, @(x) isnumeric(x) && isscalar(x));
parse(p, varargin{:});

validateattributes(pos,   {'numeric'}, {'vector','numel',2,'finite'}, mfilename, 'pos');
validateattributes(len,   {'numeric'}, {'scalar','positive'}, mfilename, 'len');
validateattributes(wid,   {'numeric'}, {'scalar','positive'}, mfilename, 'wid');
validateattributes(depth, {'numeric'}, {'scalar','positive'}, mfilename, 'depth');

cfg = irpscConfig();
[sev, risk] = classifyPotholeSeverity(depth, cfg);

ph.id        = id;
ph.pos       = pos(:).';
ph.length    = len;
ph.width     = wid;
ph.yaw       = p.Results.Yaw;
ph.depth     = depth;
ph.severity  = sev;
ph.riskLevel = risk;
end
