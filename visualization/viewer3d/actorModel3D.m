function model = actorModel3D(className, len, wid, isEgo, pal)
%ACTORMODEL3D Low-polygon 3D model for a road user (display only).
%
%   COMPONENT STATUS: REAL (visualisation geometry; carries no simulation state)
%
%   model = ACTORMODEL3D(className, len, wid) returns a model in the body
%   frame (x forward, y left, z up, origin at the body centre on the
%   ground), built from boxes and low-sided prisms, sized to the object's
%   length and width. model = ACTORMODEL3D(..., true) builds the ego
%   vehicle, which has its own colour scheme and a roof sensor pod so a
%   viewer can tell at a glance which vehicle is the autonomous one.
%   model = ACTORMODEL3D(..., isEgo, pal) uses the colour struct `pal`
%   (a vizConfig().color struct); otherwise the vizConfig defaults are used.
%
%   The models exist to make a scene READABLE -- a bus looks like a bus, a
%   cow like an animal -- not to be realistic. Geometry is deliberately
%   cheap (tens of faces per model, built once and reused) because in GNU
%   Octave the cost of drawing is dominated by the number of graphics
%   property updates, not by the number of faces. Every model is placed each
%   frame at the pose the simulation reports; nothing here moves anything.
%
%   Outputs:
%       model - struct with fields
%           .V      Kx3 vertices (body frame)
%           .F      Fx4 faces (quads; triangles repeat a vertex)
%           .C      Fx3 face colours (RGB)
%           .height scalar, top of the model (m)
%
%   Requires: base MATLAB only.
%
%   See also CREATEDEMOVIEWER, UPDATEDEMOVIEWER, VIZCONFIG.

if nargin < 4 || isempty(isEgo), isEgo = false; end
if nargin < 5 || isempty(pal)
    vc = vizConfig();
    pal = vc.color;
end

V = zeros(0,3);  F = zeros(0,4);  C = zeros(0,3);
glass = pal.egoGlass;
tyre  = [0.070 0.078 0.090];

switch className
    case 'ego'
        body = pal.ego;  accent = pal.egoAccent;
        % Lower body, tapered nose and tail so it does not read as a box.
        [V,F,C] = addTaperBox(V,F,C, [0 0 0.36], [len, wid, 0.52], 0.80, body);
        [V,F,C] = addTaperBox(V,F,C, [-0.02*len 0 0.88], [0.62*len, 0.92*wid, 0.52], 0.88, body);
        [V,F,C] = addBox(V,F,C, [-0.02*len 0 0.94], [0.58*len, 0.94*wid, 0.34], glass);
        [V,F,C] = addBox(V,F,C, [-0.30*len 0 1.16], [0.30*len, 0.78*wid, 0.06], accent); % roof stripe
        % Sensor pod: what makes it read as the autonomous vehicle.
        [V,F,C] = addPrism(V,F,C, [-0.16*len 0 1.20], 0.17, 0.16, 8, [0.137 0.157 0.180]);
        [V,F,C] = addPrism(V,F,C, [-0.16*len 0 1.36], 0.13, 0.07, 8, accent);
        [V,F,C] = addBox(V,F,C, [0.49*len  0.30*wid 0.42], [0.05, 0.22*wid, 0.14], [1.00 0.95 0.80]);
        [V,F,C] = addBox(V,F,C, [0.49*len -0.30*wid 0.42], [0.05, 0.22*wid, 0.14], [1.00 0.95 0.80]);
        [V,F,C] = addBox(V,F,C, [-0.49*len 0 0.46], [0.05, 0.80*wid, 0.10], [0.80 0.20 0.16]);
        [V,F,C] = addWheels(V,F,C, len, wid, tyre);
        h = 1.45;
    case {'car', 'unknown'}
        body = actorColor(pal, className);
        [V,F,C] = addTaperBox(V,F,C, [0 0 0.36], [len, wid, 0.52], 0.82, body);
        [V,F,C] = addTaperBox(V,F,C, [-0.04*len 0 0.86], [0.56*len, 0.90*wid, 0.46], 0.86, body);
        [V,F,C] = addBox(V,F,C, [-0.04*len 0 0.90], [0.52*len, 0.92*wid, 0.30], glass);
        [V,F,C] = addWheels(V,F,C, len, wid, tyre);
        h = 1.35;
    case 'bus'
        body = pal.actor.bus;
        [V,F,C] = addBox(V,F,C, [0 0 1.55], [len, wid, 2.30], body);
        [V,F,C] = addBox(V,F,C, [0 0 2.05], [0.96*len, wid*1.01, 0.70], glass);
        [V,F,C] = addBox(V,F,C, [0 0 2.74], [0.98*len, 0.96*wid, 0.16], body*0.8);
        [V,F,C] = addWheels(V,F,C, len, wid, tyre);
        h = 3.05;
    case 'truck'
        body = pal.actor.truck;
        [V,F,C] = addBox(V,F,C, [-0.14*len 0 1.60], [0.70*len, wid, 2.20], body);
        [V,F,C] = addBox(V,F,C, [ 0.36*len 0 1.20], [0.26*len, 0.94*wid, 1.60], body*1.15);
        [V,F,C] = addBox(V,F,C, [ 0.40*len 0 1.70], [0.16*len, 0.88*wid, 0.42], glass);
        [V,F,C] = addWheels(V,F,C, len, wid, tyre);
        h = 2.90;
    case 'autorickshaw'
        body = pal.actor.autorickshaw;
        [V,F,C] = addTaperBox(V,F,C, [0 0 0.60], [len, wid, 0.80], 0.70, body);
        [V,F,C] = addBox(V,F,C, [-0.08*len 0 1.20], [0.70*len, 0.92*wid, 0.44], glass);
        [V,F,C] = addBox(V,F,C, [-0.05*len 0 1.50], [0.92*len, wid, 0.12], [0.898 0.878 0.831]);
        [V,F,C] = addPrism(V,F,C, [ 0.40*len 0 0.30], 0.30, 0.20, 8, tyre, 'y');
        [V,F,C] = addPrism(V,F,C, [-0.32*len  0.48*wid 0.30], 0.30, 0.18, 8, tyre, 'y');
        [V,F,C] = addPrism(V,F,C, [-0.32*len -0.48*wid 0.30], 0.30, 0.18, 8, tyre, 'y');
        h = 1.70;
    case 'motorcycle'
        body = pal.actor.motorcycle;
        [V,F,C] = addBox(V,F,C, [0 0 0.52], [0.9*len, 0.28, 0.34], body);
        [V,F,C] = addBox(V,F,C, [-0.06*len 0 1.06], [0.34, 0.38, 0.62], body*0.75);   % rider torso
        [V,F,C] = addPrism(V,F,C, [-0.06*len 0 1.48], 0.13, 0.26, 8, [0.180 0.196 0.220]); % helmet
        [V,F,C] = addPrism(V,F,C, [ 0.38*len 0 0.30], 0.30, 0.12, 8, tyre, 'y');
        [V,F,C] = addPrism(V,F,C, [-0.38*len 0 0.30], 0.30, 0.12, 8, tyre, 'y');
        h = 1.70;
    case 'bicycle'
        body = pal.actor.bicycle;
        [V,F,C] = addBox(V,F,C, [0 0 0.60], [0.8*len, 0.10, 0.12], body);
        [V,F,C] = addBox(V,F,C, [-0.04*len 0 1.06], [0.30, 0.34, 0.60], body*1.15);
        [V,F,C] = addPrism(V,F,C, [-0.04*len 0 1.46], 0.12, 0.24, 8, [0.180 0.196 0.220]);
        [V,F,C] = addPrism(V,F,C, [ 0.36*len 0 0.34], 0.34, 0.06, 10, tyre, 'y');
        [V,F,C] = addPrism(V,F,C, [-0.36*len 0 0.34], 0.34, 0.06, 10, tyre, 'y');
        h = 1.65;
    case 'pedestrian'
        body = pal.actor.pedestrian;
        [V,F,C] = addBox(V,F,C, [0 0 0.42], [0.22, 0.30, 0.84], [0.220 0.243 0.302]); % legs
        [V,F,C] = addTaperBox(V,F,C, [0 0 1.08], [0.26, 0.42, 0.52], 0.85, body);     % torso
        [V,F,C] = addPrism(V,F,C, [0 0 1.46], 0.11, 0.22, 8, [0.780 0.663 0.533]);    % head
        h = 1.70;
    case 'animal'
        coat = pal.actor.animal;
        [V,F,C] = addTaperBox(V,F,C, [0 0 0.95], [0.72*len, wid, 0.62], 0.90, coat);
        [V,F,C] = addTaperBox(V,F,C, [0.44*len 0 1.12], [0.26*len, 0.52*wid, 0.40], 0.75, coat*1.05);
        [V,F,C] = addBox(V,F,C, [0.48*len 0 1.38], [0.06, 0.80*wid, 0.06], [0.420 0.380 0.318]); % horns
        [V,F,C] = addBox(V,F,C, [-0.46*len 0 1.10], [0.20*len, 0.10, 0.10], coat*0.85);          % tail
        for sx = [-0.26 0.26]
            for sy = [-0.32 0.32]
                [V,F,C] = addBox(V,F,C, [sx*len sy*wid 0.32], [0.11, 0.11, 0.64], coat*0.82);
            end
        end
        h = 1.55;
    case 'pushcart'
        body = pal.actor.pushcart;
        [V,F,C] = addBox(V,F,C, [0 0 0.80], [len, wid, 0.18], body);
        [V,F,C] = addBox(V,F,C, [0 0 0.98], [0.82*len, 0.82*wid, 0.22], [0.388 0.529 0.290]);
        [V,F,C] = addBox(V,F,C, [-0.46*len 0 1.02], [0.06, 0.60*wid, 0.50], body*0.8);
        [V,F,C] = addPrism(V,F,C, [0 0 0.34], 0.34, 0.08, 10, tyre, 'y');
        h = 1.25;
    otherwise
        [V,F,C] = addBox(V,F,C, [0 0 0.60], [len, wid, 1.20], pal.actor.unknown);
        h = 1.20;
end

if isEgo && ~strcmp(className, 'ego')
    C = 0.5 * C + 0.5 * repmat(pal.egoAccent, size(C,1), 1);
end

model.V = V;  model.F = F;  model.C = C;  model.height = h;
end

% =========================================================================
function col = actorColor(pal, className)
if isfield(pal.actor, className)
    col = pal.actor.(className);
else
    col = pal.actor.unknown;
end
end

function [V,F,C] = addBox(V,F,C, c, sz, col)
hx = sz(1)/2;  hy = sz(2)/2;  hz = sz(3)/2;
v = [-hx -hy -hz; hx -hy -hz; hx hy -hz; -hx hy -hz; ...
     -hx -hy  hz; hx -hy  hz; hx hy  hz; -hx hy  hz] + repmat(c, 8, 1);
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
F = [F; f + size(V,1)];
V = [V; v];
C = [C; shadeFaces(col, 6)];
end

function [V,F,C] = addTaperBox(V,F,C, c, sz, topScale, col)
%ADDTAPERBOX Box whose top face is smaller than its base: cheap way to make
%a shape read as a vehicle body rather than a crate.
hx = sz(1)/2;  hy = sz(2)/2;  hz = sz(3)/2;
tx = hx * topScale;  ty = hy * topScale;
v = [-hx -hy -hz; hx -hy -hz; hx hy -hz; -hx hy -hz; ...
     -tx -ty  hz; tx -ty  hz; tx ty  hz; -tx ty  hz] + repmat(c, 8, 1);
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
F = [F; f + size(V,1)];
V = [V; v];
C = [C; shadeFaces(col, 6)];
end

function [V,F,C] = addPrism(V,F,C, c, radius, height, nSides, col, axis)
%ADDPRISM Low-sided prism (wheels, sensor pods, heads). `axis` is the
%extrusion direction: 'z' (default) or 'y'.
if nargin < 9, axis = 'z'; end
th = linspace(0, 2*pi, nSides+1).';  th(end) = [];
ring = [radius*cos(th), radius*sin(th)];
h = height/2;
switch axis
    case 'y'
        a = [ring(:,1), -h*ones(nSides,1), ring(:,2)];
        b = [ring(:,1),  h*ones(nSides,1), ring(:,2)];
    otherwise
        a = [ring, -h*ones(nSides,1)];
        b = [ring,  h*ones(nSides,1)];
end
n0 = size(V,1);
V = [V; a + repmat(c, nSides, 1); b + repmat(c, nSides, 1)];
nxt = [2:nSides, 1].';
side = [n0 + (1:nSides).', n0 + nxt, n0 + nSides + nxt, n0 + nSides + (1:nSides).'];
capA = [n0 + (1:nSides)];                       % one polygon per cap, as a fan
capB = [n0 + nSides + (1:nSides)];
fan = zeros(0,4);
for i = 2:nSides-1
    fan = [fan; capA(1), capA(i), capA(i+1), capA(i+1)]; %#ok<AGROW>
    fan = [fan; capB(1), capB(i+1), capB(i), capB(i)];   %#ok<AGROW>
end
F = [F; side; fan];
nf = size(side,1) + size(fan,1);
shade = 0.70 + 0.30 * abs(cos(linspace(0, 2*pi, nf).'));
C = [C; repmat(col, nf, 1) .* repmat(shade, 1, 3)];
end

function [V,F,C] = addWheels(V,F,C, len, wid, col)
r = 0.32;
for sx = [-0.32 0.32]
    for sy = [-0.48 0.48]
        [V,F,C] = addPrism(V,F,C, [sx*len, sy*wid, r], r, 0.20, 8, col, 'y');
    end
end
end

function Cf = shadeFaces(col, n)
%SHADEFACES Fixed per-face shading so flat-lit models still read as solid.
shade = [0.72; 1.00; 0.86; 0.94; 0.86; 0.94];
if n ~= 6, shade = ones(n,1); end
Cf = min(repmat(col, n, 1) .* repmat(shade, 1, 3), 1);
end
