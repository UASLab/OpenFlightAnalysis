function [xyC] = Coherence(xyP, xxP, yyP)

xyC = abs(xyP).^2 ./ (xxP .* yyP);

xyC(xyC < 0) = 0.0;
xyC(xyC > 1) = 1.0;