function ego = makeEgoState(pos, heading, speed, varargin)
%MAKEEGOSTATE Construct the ego-vehicle state record used project-wide.
%
%   COMPONENT STATUS: REAL
%
%   ego = MAKEEGOSTATE(pos, heading, speed, ...) builds the ego state that
%   the planner, controller and dynamics model exchange.
%
%   Inputs:
%       pos     - 1x2 [x y] rear-axle position in world metres
%       heading - scalar yaw in radians
%       speed   - scalar forward speed in m/s
%
%   Name-value pairs:
%       'YawRate' - rad/s. Default 0
%       'Accel'   - m/s^2 longitudinal. Default 0
%       'Steer'   - rad, current road-wheel angle. Default 0
%       'Time'    - s, simulation time stamp. Default 0
%
%   Outputs:
%       ego - struct with fields .pos .heading .speed .yawRate .accel
%             .steer .time
%
%   Position reference point
%   ------------------------
%   pos is the REAR AXLE centre, not the vehicle centroid. The kinematic
%   bicycle model and the pure-pursuit controller are both formulated about
%   the rear axle, so using it consistently avoids a systematic offset. The
%   footprint discs in vehicleParams() are offset from this same point.
%
%   Example:
%       ego = makeEgoState([0 0], 0, 8.0, 'Time', 1.25);
%
%   Requires: base MATLAB only.
%
%   See also VEHICLEPARAMS, BICYCLEMODELSTEP, PUREPURSUITCONTROL.

validateattributes(pos, {'numeric'}, {'vector','numel',2,'finite','real'}, mfilename, 'pos');
validateattributes(heading, {'numeric'}, {'scalar','finite','real'}, mfilename, 'heading');
validateattributes(speed, {'numeric'}, {'scalar','finite','real'}, mfilename, 'speed');

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'YawRate', 0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Accel',   0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Steer',   0, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'Time',    0, @(x) isnumeric(x) && isscalar(x));
parse(p, varargin{:});

ego.pos     = pos(:).';
ego.heading = heading;
ego.speed   = speed;
ego.yawRate = p.Results.YawRate;
ego.accel   = p.Results.Accel;
ego.steer   = p.Results.Steer;
ego.time    = p.Results.Time;
end
