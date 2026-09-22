function sc = sensorConfig()
%SENSORCONFIG Parameters of the simulated camera, LiDAR and radar.
%
%   COMPONENT STATUS: REAL (the configuration) describing SIMPLIFIED sensors
%
%   sc = SENSORCONFIG() returns the mounting geometry, field of view, range
%   and error characteristics of the three simulated sensors.
%
%   ============================================================
%   WHAT THESE NUMBERS ARE, AND WHAT THEY ARE NOT
%   ============================================================
%   These are PLAUSIBLE values for automotive sensors of each type, chosen
%   so the simulation exhibits realistic failure modes: limited field of
%   view, range-dependent noise, occlusion, missed detections and occasional
%   false positives.
%
%   They are NOT measured from any real sensor, NOT taken from a
%   manufacturer datasheet, and NOT validated against real data. No result
%   produced with them may be described as real sensor performance. What
%   they legitimately support is a study of how the planner behaves when its
%   inputs are imperfect -- which is the point.
%
%   Complementary strengths are modelled deliberately, because that is the
%   entire justification for fusing three sensors:
%       camera - good angular resolution and the only class information;
%                poor range accuracy; degrades in poor visibility
%       lidar  - excellent range and shape; no class information;
%                moderate field of view
%       radar  - direct radial velocity measurement and long range;
%                poor angular resolution; robust to weather
%
%   Outputs:
%       sc - struct with fields .camera, .lidar, .radar, each containing:
%           .name         char
%           .mountPos     1x2 [x y] offset from the rear axle (m)
%           .mountYaw     rad, mounting yaw relative to vehicle forward
%           .fov          rad, total horizontal field of view
%           .maxRange     m
%           .minRange     m
%           .rangeStd     m, 1-sigma range error at 10 m
%           .rangeStdRate m of extra error per metre of range
%           .azimuthStd   rad, 1-sigma bearing error
%           .velStd       m/s, 1-sigma radial velocity error (radar only;
%                         Inf means the sensor does not measure velocity)
%           .pDetect      base probability of detecting a visible object
%           .falseRate    expected false positives per frame
%           .providesClass logical
%           .updateRate   Hz
%
%   Example:
%       sc = sensorConfig();
%       fprintf('camera FOV: %.0f deg\n', rad2deg(sc.camera.fov));
%
%   Requires: base MATLAB only.
%
%   See also SIMULATEDETECTIONS, FUSEDETECTIONS.

% --- Forward camera -----------------------------------------------------
sc.camera.name          = 'camera';
sc.camera.mountPos      = [2.0, 0.0];
sc.camera.mountYaw      = 0;
sc.camera.fov           = deg2rad(90);
sc.camera.maxRange      = 60;
sc.camera.minRange      = 0.5;
sc.camera.rangeStd      = 0.60;    % cameras estimate depth poorly
sc.camera.rangeStdRate  = 0.045;   % and it degrades quickly with distance
sc.camera.azimuthStd    = deg2rad(0.4);   % but bearing is excellent
sc.camera.velStd        = Inf;     % no direct velocity measurement
sc.camera.pDetect       = 0.92;
sc.camera.falseRate     = 0.05;
sc.camera.providesClass = true;    % the only sensor that classifies
sc.camera.updateRate    = 30;

% --- LiDAR --------------------------------------------------------------
sc.lidar.name          = 'lidar';
sc.lidar.mountPos      = [1.5, 0.0];
sc.lidar.mountYaw      = 0;
sc.lidar.fov           = deg2rad(120);
sc.lidar.maxRange      = 80;
sc.lidar.minRange      = 0.8;
sc.lidar.rangeStd      = 0.05;     % excellent range accuracy
sc.lidar.rangeStdRate  = 0.002;
sc.lidar.azimuthStd    = deg2rad(0.25);
sc.lidar.velStd        = Inf;
sc.lidar.pDetect       = 0.95;
sc.lidar.falseRate     = 0.02;
sc.lidar.providesClass = false;    % geometry only, no semantics
sc.lidar.updateRate    = 10;

% --- Forward radar ------------------------------------------------------
sc.radar.name          = 'radar';
sc.radar.mountPos      = [3.2, 0.0];
sc.radar.mountYaw      = 0;
sc.radar.fov           = deg2rad(45);
sc.radar.maxRange      = 120;
sc.radar.minRange      = 1.0;
sc.radar.rangeStd      = 0.25;
sc.radar.rangeStdRate  = 0.004;
sc.radar.azimuthStd    = deg2rad(2.0);    % poor angular resolution
sc.radar.velStd        = 0.10;            % but direct Doppler velocity
sc.radar.pDetect       = 0.88;
sc.radar.falseRate     = 0.15;            % clutter is common
sc.radar.providesClass = false;
sc.radar.updateRate    = 20;
end
