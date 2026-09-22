function verifyEmpty(tc, varargin) %#ok<INUSL>
%VERIFYEMPTY Octave compatibility shim for matlab.unittest verification.
%   OCTAVE ONLY. Throws 'octaveShim:verifyFailed' on failure.
if ~isempty(varargin{1}), fail(varargin, 2, 'verifyEmpty'); end
end

function fail(args, msgIdx, what)
msg = '';
if numel(args) >= msgIdx && ischar(args{msgIdx})
    msg = args{msgIdx};
end
error('octaveShim:verifyFailed', '%s failed. %s', what, msg);
end
