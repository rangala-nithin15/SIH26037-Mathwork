function obs = makeObstacle(id, class, pos, vel, varargin)
%MAKEOBSTACLE Construct the road-user (obstacle) record used project-wide.
%
%   COMPONENT STATUS: REAL
%
%   obs = MAKEOBSTACLE(id, class, pos, vel, ...) builds one tracked road
%   user in the single canonical format that prediction, risk assessment and
%   the planner all consume. Perception, tracking and the scenario ground
%   truth all emit this same struct, so the planner never needs to know
%   which of them produced it.
%
%   Inputs:
%       id    - numeric scalar, unique track identifier
%       class - char, one of the names returned by objectClasses()
%       pos   - 1x2 [x y] world position in metres
%       vel   - 1x2 [vx vy] world velocity in m/s
%
%   Name-value pairs:
%       'Heading'    - rad. Default: atan2(vy,vx), or 0 if nearly stationary
%       'Length'     - m. Default: class nominal from objectClasses()
%       'Width'      - m. Default: class nominal
%       'Accel'      - 1x2 [ax ay] m/s^2. Default [0 0]
%       'Confidence' - 0..1 detection/track confidence. Default 1.0
%       'Age'        - integer frames the track has existed. Default 1
%       'PosCov'     - 2x2 position covariance. Default 0.3^2 * eye(2)
%       'TruthId'    - id of the ground-truth road user this record came
%                      from, NaN if unknown (e.g. a false-positive track).
%                      Used ONLY by evaluation metrics, never by the planner.
%
%   Outputs:
%       obs - struct with the fields above plus .vulnerable and .agility
%             copied from the class table
%
%   An unrecognised class is mapped to 'unknown' with a warning rather than
%   erroring, so one odd label from perception cannot halt a whole run.
%
%   Example:
%       o = makeObstacle(1, 'autorickshaw', [12 3], [-4 0], 'Confidence', 0.8);
%
%   Requires: base MATLAB only.
%
%   See also OBJECTCLASSES, PREDICTOBSTACLES.

validateattributes(id, {'numeric'}, {'scalar','real'}, mfilename, 'id');
validateattributes(pos, {'numeric'}, {'vector','numel',2,'finite','real'}, mfilename, 'pos');
validateattributes(vel, {'numeric'}, {'vector','numel',2,'finite','real'}, mfilename, 'vel');

[names, info] = objectClasses();
class = char(class);
ci    = find(strcmp(names, class), 1);
if isempty(ci)
    warning('makeObstacle:unknownClass', ...
            'Class "%s" is not in objectClasses(); treating as "unknown".', class);
    class = 'unknown';
    ci    = find(strcmp(names, 'unknown'), 1);
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'Heading',    [],  @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Length',     info(ci).length, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'Width',      info(ci).width,  @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'Accel',      [0 0], @(x) isnumeric(x) && numel(x) == 2);
addParameter(p, 'Confidence', 1.0,   @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
addParameter(p, 'Age',        1,     @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'PosCov',     0.3^2 * eye(2), @(x) isnumeric(x) && isequal(size(x), [2 2]));
addParameter(p, 'TruthId',    NaN,   @(x) isnumeric(x) && isscalar(x));
parse(p, varargin{:});
r = p.Results;

pos = pos(:).';
vel = vel(:).';

if isempty(r.Heading)
    if hypot(vel(1), vel(2)) > 0.1
        heading = atan2(vel(2), vel(1));
    else
        heading = 0;   % stationary: velocity direction is meaningless
    end
else
    heading = r.Heading;
end

obs.id         = id;
obs.class      = class;
obs.pos        = pos;
obs.vel        = vel;
obs.accel      = r.Accel(:).';
obs.heading    = heading;
obs.speed      = hypot(vel(1), vel(2));
obs.length     = r.Length;
obs.width      = r.Width;
obs.confidence = r.Confidence;
obs.age        = r.Age;
obs.posCov     = r.PosCov;
obs.vulnerable = info(ci).vulnerable;
obs.agility    = info(ci).agility;
obs.truthId    = r.TruthId;
end
