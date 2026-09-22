function rec = frameRecorder(action, rec, varargin)
%FRAMERECORDER Record viewer frames to a video file (shared by the demos).
%
%   COMPONENT STATUS: REAL (visualisation output only).
%
%   rec = FRAMERECORDER('start', [], videoFile, fps)
%   rec = FRAMERECORDER('write', rec, fig)
%   rec = FRAMERECORDER('stop',  rec)
%
%   MATLAB: frames go straight into VideoWriter (base MATLAB) at `fps`.
%   GNU Octave (no VideoWriter): frames are written as PNG files into
%   <videoFile>_frames/, and on 'stop' they are assembled into the requested
%   video with the ffmpeg that ships with Octave (H.264, plays in PowerPoint,
%   browsers and VLC). If no ffmpeg is found the PNG frames are kept and the
%   exact command to assemble them is printed.
%
%   A recording plays at exactly `fps` frames per second, so the caller
%   must write one frame per 1/fps seconds of simulated time for the video
%   to run in real time. REPLAYDEMO does this; RUNDEMO records one frame per
%   simulation step and therefore uses fps = 1/dt.
%
%   An empty videoFile gives a recorder that does nothing.
%
%   Requires: base MATLAB only (or GNU Octave).
%
%   See also REPLAYDEMO, RUNDEMO.

switch action
    case 'start'
        videoFile = varargin{1};
        fps = varargin{2};
        rec = struct('mode', 'none', 'writer', [], 'folder', '', 'n', 0, ...
                     'file', videoFile, 'fps', fps);
        if isempty(videoFile), return; end
        outDir = fileparts(videoFile);
        if ~isempty(outDir) && exist(outDir, 'dir') ~= 7, mkdir(outDir); end
        if exist('VideoWriter', 'class') == 8 || exist('VideoWriter', 'file') == 2
            try
                w = VideoWriter(videoFile, 'MPEG-4');
            catch
                w = VideoWriter(videoFile);       % e.g. Motion JPEG AVI
            end
            w.FrameRate = fps;
            open(w);
            rec.mode = 'video';  rec.writer = w;
            fprintf('Recording to %s at %g FPS (VideoWriter)\n', videoFile, fps);
        else
            [p, n] = fileparts(videoFile);
            rec.folder = fullfile(p, [n '_frames']);
            if exist(rec.folder, 'dir') == 7
                old = dir(fullfile(rec.folder, 'frame_*.png'));   % no stale frames
                for i = 1:numel(old), delete(fullfile(rec.folder, old(i).name)); end
            else
                mkdir(rec.folder);
            end
            rec.mode = 'png';
            fprintf('Recording PNG frames to %s (assembled at %g FPS when done)\n', rec.folder, fps);
        end

    case 'write'
        fig = varargin{1};
        switch rec.mode
            case 'video'
                writeVideo(rec.writer, getframe(fig));
                rec.n = rec.n + 1;
            case 'png'
                rec.n = rec.n + 1;
                % -S sets the raster size in pixels (GNU Octave print option).
                print(fig, fullfile(rec.folder, sprintf('frame_%05d.png', rec.n)), ...
                      '-dpng', '-S1600,918');
        end

    case 'stop'
        switch rec.mode
            case 'video'
                close(rec.writer);
                fprintf('Video closed: %s (%d frames at %g FPS)\n', rec.file, rec.n, rec.fps);
            case 'png'
                assemble(rec);
        end

    otherwise
        error('frameRecorder:action', 'Unknown action "%s".', action);
end
end

% =========================================================================
function assemble(rec)
%ASSEMBLE Build the video from the PNG frames with ffmpeg, if available.
pattern = fullfile(rec.folder, 'frame_%05d.png');
ff = findFfmpeg();
cmd = sprintf(['"%s" -y -loglevel error -framerate %g -i "%s" -c:v libx264 ' ...
               '-pix_fmt yuv420p -crf 20 -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" "%s"'], ...
              ff, rec.fps, pattern, rec.file);
if isempty(ff)
    fprintf('Saved %d PNG frames in %s. No ffmpeg found; to make the video run:\n  %s\n', ...
            rec.n, rec.folder, strrep(cmd, '""', 'ffmpeg'));
    return;
end
fprintf('Assembling %d frames into %s at %g FPS ...\n', rec.n, rec.file, rec.fps);
[status, out] = system(cmd);
if status == 0
    fprintf('Video written: %s (%d frames, %.1f s at %g FPS). PNG frames kept in %s\n', ...
            rec.file, rec.n, rec.n / rec.fps, rec.fps, rec.folder);
else
    fprintf('ffmpeg failed (%d): %s\nPNG frames kept in %s\n', status, out, rec.folder);
end
end

function ff = findFfmpeg()
ff = '';
if exist('OCTAVE_HOME', 'builtin') || exist('OCTAVE_HOME', 'file')
    cand = fullfile(OCTAVE_HOME(), 'bin', 'ffmpeg.exe');       % bundled on Windows
    if exist(cand, 'file') == 2, ff = cand; return; end
    cand = fullfile(OCTAVE_HOME(), 'bin', 'ffmpeg');
    if exist(cand, 'file') == 2, ff = cand; return; end
end
[status, ~] = system('ffmpeg -version');
if status == 0, ff = 'ffmpeg'; end
end
