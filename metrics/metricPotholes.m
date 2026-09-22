function P = metricPotholes(log)
%METRICPOTHOLES Pothole detection and handling summary for one run.
%
%   COMPONENT STATUS: REAL (evaluation against ground truth)
%
%   P = METRICPOTHOLES(log) reports, for the TRUE potholes in the scenario:
%     .nTrue            number of potholes in the scenario
%     .nConfirmed       how many TRUE potholes were confirmed by the tracker
%     .duplicateTracks  extra confirmed tracks on an already-confirmed pothole
%     .falseConfirmed   confirmed tracks with no true pothole behind them
%     .severityCorrect  confirmed potholes whose estimated severity class
%                       matches the true class
%     .wheelEntries     tyre entries into true potholes (from the run log)
%     .maxEntrySpeed    fastest entry (m/s), 0 if none
%     .entriesBySeverity struct: minor / moderate / severe entry counts
%
%   A pothole that is avoided produces no entry; one that is crossed shows
%   the speed at which it was crossed. Ride response is NOT modelled.
%
%   Requires: base MATLAB only.
%
%   See also RUNSCENARIO, POTHOLETRACKER.

P = struct('nTrue', 0, 'nConfirmed', 0, 'falseConfirmed', 0, 'duplicateTracks', 0, 'severityCorrect', 0, ...
           'wheelEntries', 0, 'maxEntrySpeed', 0, ...
           'entriesBySeverity', struct('minor', 0, 'moderate', 0, 'severe', 0));
if isfield(log, 'truthPotholes')
    P.nTrue = numel(log.truthPotholes);
end
if isfield(log, 'potholeTracks') && ~isempty(log.potholeTracks)
    tr = log.potholeTracks{end};
    seenTruth = [];
    for i = 1:numel(tr)
        if ~tr(i).confirmed, continue; end
        if isnan(tr(i).truthId)
            P.falseConfirmed = P.falseConfirmed + 1;
            continue;
        end
        if any(seenTruth == tr(i).truthId)
            P.duplicateTracks = P.duplicateTracks + 1;   % same pothole twice
            continue;
        end
        seenTruth(end+1) = tr(i).truthId; %#ok<AGROW>
        P.nConfirmed = P.nConfirmed + 1;
        k = find([log.truthPotholes.id] == tr(i).truthId, 1);
        if ~isempty(k) && strcmp(log.truthPotholes(k).severity, tr(i).severity)
            P.severityCorrect = P.severityCorrect + 1;
        end
    end
end
if isfield(log, 'potholeEvents')
    ev = log.potholeEvents;
    P.wheelEntries = numel(ev);
    if ~isempty(ev)
        P.maxEntrySpeed = max([ev.speed]);
        for i = 1:numel(ev)
            if isfield(P.entriesBySeverity, ev(i).severity)
                P.entriesBySeverity.(ev(i).severity) = P.entriesBySeverity.(ev(i).severity) + 1;
            end
        end
    end
end
end
