function verifyFalse(tc, varargin) %#ok<INUSL>
%VERIFYFALSE Octave compatibility shim for matlab.unittest verification.
%   OCTAVE ONLY. Throws 'octaveShim:verifyFailed' on failure.
cond = varargin{1}; if isempty(cond) || any(cond(:)), fail(varargin, 2, 'verifyFalse'); end
end

function fail(args, msgIdx, what)
msg = '';
if numel(args) >= msgIdx && ischar(args{msgIdx})
    msg = args{msgIdx};
end
error('octaveShim:verifyFailed', '%s failed. %s', what, msg);
end
