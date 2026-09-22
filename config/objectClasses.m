function [names, info] = objectClasses()
%OBJECTCLASSES Road-user classes handled by this project.
%
%   COMPONENT STATUS: REAL (class definitions and nominal physical sizes)
%
%   IMPORTANT HONESTY NOTE
%   ----------------------
%   This function defines the classes the PLANNING and PREDICTION code can
%   reason about. It does NOT mean any detector has been trained on them.
%   As of this writing no detector has been trained or evaluated by this
%   project. A stock COCO-trained detector covers car, bus, truck,
%   motorcycle, bicycle and person, but does NOT cover auto-rickshaw,
%   pushcart, or Indian cattle as distinct classes -- those require a custom
%   annotated dataset. The .cocoNative flag below records that distinction
%   explicitly so no one can mistake intent for capability.
%   See docs/PERCEPTION_NOTES.md.
%
%   Outputs:
%       names - 1xN cell array of class-name strings
%       info  - 1xN struct array with fields:
%                 .name       char
%                 .length     m, nominal bounding-box length
%                 .width      m, nominal bounding-box width
%                 .maxSpeed   m/s, plausible upper speed
%                 .agility    0..1, how quickly it can change direction
%                 .vulnerable logical, true for unprotected road users
%                 .cocoNative logical, true if a stock COCO detector has a
%                             directly corresponding class
%
%   Example:
%       [names, info] = objectClasses();
%       idx = strcmp(names, 'autorickshaw');
%       fprintf('%s is COCO-native: %d\n', info(idx).name, info(idx).cocoNative);
%
%   Requires: base MATLAB only.

mk = @(n,L,W,V,A,Vu,Co) struct('name',n,'length',L,'width',W, ...
        'maxSpeed',V,'agility',A,'vulnerable',Vu,'cocoNative',Co);

info = [ ...
    mk('car',          4.20, 1.75, 22.0, 0.30, false, true ), ...
    mk('bus',         11.00, 2.55, 16.7, 0.10, false, true ), ...
    mk('truck',        8.00, 2.45, 16.7, 0.10, false, true ), ...
    mk('motorcycle',   2.00, 0.75, 25.0, 0.85, true,  true ), ...
    mk('bicycle',      1.75, 0.60,  8.0, 0.75, true,  true ), ...
    mk('autorickshaw', 2.65, 1.40, 13.9, 0.60, true,  false), ...
    mk('pedestrian',   0.60, 0.55,  2.5, 0.95, true,  true ), ...
    mk('pushcart',     1.80, 0.90,  1.8, 0.40, true,  false), ...
    mk('animal',       2.00, 0.80,  5.0, 0.70, true,  false), ...
    mk('unknown',      2.00, 1.00, 15.0, 0.60, true,  false)  ];

names = {info.name};
end
