function [scale] = PowerScale(scaleType, freqRate, win)
% Compute the scaling for power
switch lower(scaleType)
    case 'density'
        scale = 1.0 / (freqRate * sum(win.^2));
    case 'spectrum'
        scale = 1.0 / sum(win).^2;
    otherwise
        scale = 1;
        warning([mfilename ' - Unknown Power Scaling.']);
end
