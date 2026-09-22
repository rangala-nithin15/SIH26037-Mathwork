function V = applyVisualMode(V, mode)
%APPLYVISUALMODE Show or hide whole layer groups for a visual mode.
%
%   COMPONENT STATUS: REAL (visualisation only).
%
%   V = APPLYVISUALMODE(V, mode) switches the viewer between the visual
%   modes defined in VIZCONFIG: full, planning, sensing, minimal. It is
%   called when the viewer is created and when the user presses f/p/s/m --
%   never once per frame, because toggling visibility touches many handles.
%
%   Requires: base MATLAB only.
%
%   See also VIZCONFIG, CREATEDEMOVIEWER, UPDATEDEMOVIEWER.

vc = vizConfig(mode);
% Keep the settings the caller customised (colours, rates, fps), take only
% the mode's layer visibility.
V.vc.visualMode = mode;
V.vc.show = vc.show;
s = V.vc.show;
groups = {'corridor', s.corridor; 'risk', s.risk; 'potholeCost', s.potholeCost; ...
          'prediction', s.prediction; 'predPaths', s.predPaths; 'tracks', s.tracks; ...
          'detections', s.detections; 'potholes', s.potholes; 'preferred', s.preferred; ...
          'rejected', s.rejected; 'trail', s.trail; 'trajectory', s.trajectory};
for i = 1:size(groups,1)
    hs = V.group.(groups{i,1});
    for j = 1:numel(hs)
        if ishandle(hs(j)), set(hs(j), 'Visible', onOff(groups{i,2})); end
    end
end
for i = 1:numel(V.trackLbl)
    if ishandle(V.trackLbl(i)) && ~s.tracks, set(V.trackLbl(i), 'Visible', 'off'); end
end
for i = 1:numel(V.potLbl)
    if ishandle(V.potLbl(i)) && ~s.potholes, set(V.potLbl(i), 'Visible', 'off'); end
end
if ishandle(V.axMap), set(V.axMap, 'Visible', onOff(s.minimap)); end
mapKids = get(V.axMap, 'children');
for i = 1:numel(mapKids), set(mapKids(i), 'Visible', onOff(s.minimap)); end
if ishandle(V.axTs), set(V.axTs, 'Visible', onOff(s.timeseries)); end
tsKids = get(V.axTs, 'children');
for i = 1:numel(tsKids), set(tsKids(i), 'Visible', onOff(s.timeseries)); end
hudKids = get(V.axHud, 'children');
for i = 1:numel(hudKids), set(hudKids(i), 'Visible', onOff(s.hud)); end
set(V.h.modeText, 'String', sprintf('MODE: %s', upper(mode)));
end

function s = onOff(tf)
if tf, s = 'on'; else, s = 'off'; end
end
