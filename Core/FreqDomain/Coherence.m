function [xyC] = Coherence(xyP, xxP, yyP)

xyC = abs(xyP).^2 ./ (xxP .* yyP);