function [gainLoes_dB, phaseLoes_deg, gain2Loes_dB, phase2Loes_deg] = LoesFunc(typeTF, tfParamsValue, freq_rps)
% Calculates the frequency response for a LOES fit.
%
%Usage:  [gainLoes_dB, phaseLoes_deg, gain2Loes_dB, phase2Loes_deg] = LoesFunc(typeTF, tfParamsValue, freq_rps);
%
%Inputs:
% typeTF        - transfer function type; see: 'LoesTFparams'
% tfParamsValue - transfer function parameters
% freq_rps      - frequency of frequency response (rad/sec)
%
%Outputs:
% gainLoes_dB    - magnitude of 1st LOES fit frequency response (dB)
% phaseLoes_deg  - phase of 1st LOES fit frequency response (deg)
% gain2Loes_dB   - magnitude of 2nd LOES fit frequency response (dB)
% phase2Loes_deg - phase of 2nd LOES fit frequency response (deg)
%
%Notes:
% 
%

%Version History: Version 1.0
% 03/07/2006  C. Regan     Initial Release (v1.0)
%


%% Check I/O Arguments
error(nargchk(3, 3, nargin, 'struct'))
error(nargoutchk(0, 4, nargout, 'struct'))


%% Default Values and Constants


%% Check Inputs
% Inputs as column vectors
freq_rps = freq_rps(:);


%%
switch typeTF
    case 1
        % q/dep transfer function
        if tfParamsValue(4) < eps, tfParamsValue(4) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(4), 2);
        tfNum = [tfParamsValue(3), tfParamsValue(3)*tfParamsValue(5)];
        tfDenom = [1, 2*tfParamsValue(1)*tfParamsValue(2), tfParamsValue(2)*tfParamsValue(2)];

        [loesNum, loesDenom] = series(tfNum, tfDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 2
        % nz/dep transfer function
        if tfParamsValue(4) < eps, tfParamsValue(4) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(4), 2);
        tfNum = [tfParamsValue(3)];
        tfDenom = [1, 2*tfParamsValue(1)*tfParamsValue(2), tfParamsValue(2)*tfParamsValue(2)];

        [loesNum, loesDenom] = series(tfNum, tfDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 3
        % q/dep transfer function
        if tfParamsValue(2) < eps, tfParamsValue(2) = eps; end
        if tfParamsValue(3) < eps, tfParamsValue(3) = eps; end
        if tfParamsValue(6) < eps, tfParamsValue(6) = eps; end
        if tfParamsValue(4) < eps, tfParamsValue(4) = 0; end
        if tfParamsValue(7) < eps, tfParamsValue(7) = 0; end

        [pade1Num, pade1Denom] = pade(tfParamsValue(4), 2);
        tf1Num1 = tfParamsValue(1)*[1, 1/tfParamsValue(2), 0];
        tf1Denom1 = [1, 2*tfParamsValue(8)*tfParamsValue(9), tfParamsValue(9)*tfParamsValue(9)];
        tf1Num2 = [1, 1/tfParamsValue(3)];
        tf1Denom2 = [1, 2*tfParamsValue(10)*tfParamsValue(11), tfParamsValue(11)*tfParamsValue(11)];

        [temp1Num, temp1Denom] = series(tf1Num1, tf1Denom1, tf1Num2, tf1Denom2);
        [loes1Num, loes1Denom] = series(temp1Num, temp1Denom, pade1Num, pade1Denom);

        [gainLoes, phaseLoes_deg] = bode(loes1Num, loes1Denom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes);

        % nz/dep transfer function
        [pade2Num, pade2Denom] = pade(tfParamsValue(7), 2);
        tf2Num1 = tfParamsValue(5)*[1, 1/tfParamsValue(6)];
        tf2Denom1 = tf1Denom1;
        tf2Num2 = 1;
        tf2Denom2 = tf1Denom2;

        [temp2Num, temp2Denom] = series(tf2Num1, tf2Denom1, tf2Num2, tf2Denom2);
        [loes2Num, loes2Denom] = series(temp2Num, temp2Denom, pade2Num, pade2Denom);

        [gain2Loes, phase2Loes_deg] = bode(loes2Num, loes2Denom, freq_rps);
        gain2Loes_dB = Mag2DB(gain2Loes);

    case 4
        % q/dep transfer function
        if tfParamsValue(3) < eps, tfParamsValue(3) = 0; end
        if tfParamsValue(5) < eps, tfParamsValue(5) = 0; end

        [pade1Num, pade1Denom] = pade(tfParamsValue(3), 2);
        tf1Num = tfParamsValue(1)*[1, tfParamsValue(2)];
        tf1Denom = [1, 2*tfParamsValue(6)*tfParamsValue(7), tfParamsValue(7)*tfParamsValue(7)];

        [loes1Num, loes1Denom] = series(tf1Num, tf1Denom, pade1Num, pade1Denom);

        [gainLoes, phaseLoes_deg] = bode(loes1Num, loes1Denom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes);

        % nz/dep transfer function
        [pade2Num, pade2Denom] = pade(tfParamsValue(5), 2);
        tf2Num = tfParamsValue(4);
        tf2Denom = tf1Denom;

        [loes2Num, loes2Denom] = series(tf2Num, tf2Denom, pade2Num, pade2Denom);

        [gain2Loes, phase2Loes_deg] = bode(loes2Num, loes2Denom, freq_rps);
        gain2Loes_dB = Mag2DB(gain2Loes);

    case 5
        % q/dep transfer function
        if tfParamsValue(3) < eps, tfParamsValue(3) = 0; end
        if tfParamsValue(5) < eps, tfParamsValue(5) = 0; end

        [pade1Num, pade1Denom] = pade(tfParamsValue(3), 2);
        tf1Num = tfParamsValue(1)*[1, tfParamsValue(2)];
        tf1Denom = [1, 2*tfParamsValue(6)*tfParamsValue(7), tfParamsValue(7)*tfParamsValue(7)];

        [loes1Num, loes1Denom] = series(tf1Num, tf1Denom, pade1Num, pade1Denom);

        [gainLoes, phaseLoes_deg] = bode(loes1Num, loes1Denom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes);

        % aoa/dep transfer function
        [pade2Num, pade2Denom] = pade(tfParamsValue(5), 2);
        tf2Num = tfParamsValue(4);
        tf2Denom = tf1Denom;

        [loes2Num, loes2Denom] = series(tf2Num, tf2Denom, pade2Num, pade2Denom);

        [gain2Loes, phase2Loes_deg] = bode(loes2Num, loes2Denom, freq_rps);
        gain2Loes_dB = Mag2DB(gain2Loes);

    case 6
        % alp/dep transfer function
        if tfParamsValue(4) < eps, tfParamsValue(4) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(4), 2);
        tfNum = [tfParamsValue(3)];
        tfDenom = [1, 2*tfParamsValue(1)*tfParamsValue(2), tfParamsValue(2)*tfParamsValue(2)];

        [loesNum, loesDenom] = series(tfNum, tfDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 7
        % p/dap transfer function
        if tfParamsValue(1) < eps, tfParamsValue(1) = eps; end
        if tfParamsValue(3) < eps, tfParamsValue(3) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(3), 2);
        tfNum = [tfParamsValue(2)];
        tfDenom = [1, 1/tfParamsValue(1)];

        [loesNum, loesDenom] = series(tfNum, tfDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 8
        % p/dap transfer function
        if tfParamsValue(1) < eps, tfParamsValue(1) = eps; end
        if tfParamsValue(2) < eps, tfParamsValue(2) = eps; end
        if tfParamsValue(5) < eps, tfParamsValue(5) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(5), 2);

        tfNum1 = tfParamsValue(6)*[1, 2*tfParamsValue(8)*tfParamsValue(7), tfParamsValue(7)*tfParamsValue(7)];
        tfDenom1 = [1, 2*tfParamsValue(3)*tfParamsValue(4), tfParamsValue(4)*tfParamsValue(4)];

        tfNum2 = [1, 0];
        tfDenom2 = [1, 1/tfParamsValue(1) + 1/tfParamsValue(2), 1/tfParamsValue(1) * 1/tfParamsValue(2)];

        [tempNum, tempDenom] = series(tfNum1, tfDenom1, tfNum2, tfDenom2);
        [loesNum, loesDenom] = series(tempNum, tempDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 9
        % beta/drp transfer function
        if tfParamsValue(4) < eps, tfParamsValue(4) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(4), 2);
        tfNum = [tfParamsValue(3)];
        tfDenom = [1, 2*tfParamsValue(1)*tfParamsValue(2), tfParamsValue(2)*tfParamsValue(2)];

        [loesNum, loesDenom] = series(tfNum, tfDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 10
        % beta/drp transfer function
        if tfParamsValue(3) < eps, tfParamsValue(3) = eps; end
        if tfParamsValue(4) < eps, tfParamsValue(4) = eps; end
        if tfParamsValue(6) < eps, tfParamsValue(6) = eps; end
        if tfParamsValue(7) < eps, tfParamsValue(7) = eps; end
        if tfParamsValue(8) < eps, tfParamsValue(8) = eps; end
        if tfParamsValue(9) < eps, tfParamsValue(9) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(9), 2);
        tfNum1 = tfParamsValue(5)*[1, 1/tfParamsValue(6) + 1/tfParamsValue(7), 1/tfParamsValue(6) * 1/tfParamsValue(7)];
        tfDenom1 = [1, 2*tfParamsValue(1)*tfParamsValue(2), tfParamsValue(2)*tfParamsValue(2)];
        tfNum2 = [1, 1/tfParamsValue(8)];
        tfDenom2 = [1, 1/tfParamsValue(3) + 1/tfParamsValue(4), 1/tfParamsValue(3) * 1/tfParamsValue(4)];

        [tempNum, tempDenom] = series(tfNum1, tfDenom1, tfNum2, tfDenom2);
        [loesNum, loesDenom] = series(tempNum, tempDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 11
        % q/dep transfer function
        if tfParamsValue(7) < eps, tfParamsValue(7) = eps; end
        if tfParamsValue(8) < eps, tfParamsValue(8) = eps; end
        if tfParamsValue(4) < eps, tfParamsValue(4) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(4), 2);
        tfNum1 = tfParamsValue(3)*[1, 2*tfParamsValue(1)*tfParamsValue(2), tfParamsValue(2)*tfParamsValue(2)];
        tfDenom1 = [1, 2*tfParamsValue(5)*tfParamsValue(6), tfParamsValue(6)*tfParamsValue(6)];
        tfNum2 = [1];
        tfDenom2 = [1, 1/tfParamsValue(7) + 1/tfParamsValue(8), 1/tfParamsValue(7) * 1/tfParamsValue(8)];

        [tempNum, tempDenom] = series(tfNum1, tfDenom1, padeNum, padeDenom);
        [loesNum, loesDenom] = series(tempNum, tempDenom, tfNum2, tfDenom2);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    case 12
        % phi/dap transfer function
        if tfParamsValue(1) < eps, tfParamsValue(1) = eps; end
        if tfParamsValue(3) < eps, tfParamsValue(3) = 0; end

        [padeNum, padeDenom] = pade(tfParamsValue(3), 2);
        tfNum = [tfParamsValue(2)];
        tfDenom = [1, 1/tfParamsValue(1), 0];

        [loesNum, loesDenom] = series(tfNum, tfDenom, padeNum, padeDenom);

        [gainLoes_mag, phaseLoes_deg] = bode(loesNum, loesDenom, freq_rps);
        gainLoes_dB = Mag2DB(gainLoes_mag);

    otherwise
        warning('')
end


%% Check Outputs
