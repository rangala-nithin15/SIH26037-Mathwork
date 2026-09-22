function verifyGreaterThan(tc, varargin) %#ok<INUSL>
%VERIFYGREATERTHAN Octave compatibility shim for matlab.unittest verification.
%   OCTAVE ONLY. Throws 'octaveShim:verifyFailed' on failure.
if ~all(varargin{1}(:) > varargin{2}), fail(varargin, 3, 'verifyGreaterThan'); end
end

function fail(args, msgIdx, what)
msg = '';
if numel(args) >= msgIdx && ischar(args{msgIdx})
    msg = args{msgIdx};
end
error('octaveShim:verifyFailed', '%s failed. %s', what, msg);
end
