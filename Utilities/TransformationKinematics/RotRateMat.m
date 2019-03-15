function [rotRateMat] = RotRateMat(seqList, angleList, units)
% Compute the tranformation matrix for converting between Euler Angle Rates and Body Angle Rates. 
%
%Inputs:
% seqList   - cell array list of sequences
% angleList - cell array angle values associated with the sequence
% units     - units of angles [rad]
%
%Outputs:
% rotRateMat     - Rotation Matrix
%
%Notes:
% Hard-coded transformations perform much faster!
% Uses RotMat.m
% For this Transformation Matrix: T' != T^-1
% Checked with 3-2-1 rotation sequence to give the expected result
%     rotRateMat = [1, 0, -sin(angleList(2), ; 0, cos(angleList(1)), cos(angleList(2))*sin(angleList(1)); 0, -sin(angleList(1)), cos(angleList(2))*cos(angleList(1))];
%     [p;q;r] = rotRateMat * [phiDot; thetaDot; psiDot]

%Version History: Version 1.3
% 09/27/2016  C. Regan     Initial Release (v1.0)
% 09/28/2016  C. Regan     Added vector inputs rather than cell array inputs (v1.1)
% 10/17/2016  C. Regan     Changed to simple and faster input parsing (v1.2)
% 10/17/2016  C. Regan     Added hard-coded '321' case for speed (v1.3)


%% Check I/O Arguments
% Check the number of inputs
narginchk(2, 3);
if nargin < 3
    units = [];
end

% Default Values
if isempty(units), units = 'rad'; end
    
% Check the number of outputs
nargoutchk(0, 1);


%% Check Inputs
if iscell(seqList), seqList = cell2mat(seqList); end
if iscell(angleList), angleList = cell2mat(angleList); end

lenSeq = length(seqList);
lenAngle = length(angleList);
if lenSeq ~= lenAngle, error('Input Sequences must be of the same length'); end

switch lower(units)
    case {'deg'}
        angleList = d2r * angleList;
    otherwise
end

%% Compute the product of the sequence
switch lower(seqList)
    case {'321'}
        rotRateMat = [1, 0, -sin(angleList(2)); ...
            0, cos(angleList(1)), cos(angleList(2))*sin(angleList(1)); ...
            0, -sin(angleList(1)), cos(angleList(2))*cos(angleList(1))];
    otherwise
        seqVec = NaN(1, lenSeq);
        for indxSeq = 1:lenSeq
            seqVec(indxSeq) = str2num(seqList(indxSeq));
        end
        
        [~, rotMatList] = RotMat(seqList, angleList, units);
        
        % Temperary matrices
        rotRateMatList{1} = rotMatList{3} * rotMatList{2};
        rotRateMatList{2} = rotMatList{3};
        rotRateMatList{3} = eye(3);

        % Form the Matrix
        rotRateMat(:, 1) = rotRateMatList{3}(:, seqVec(3));
        rotRateMat(:, 2) = rotRateMatList{2}(:, seqVec(2));
        rotRateMat(:, 3) = rotRateMatList{1}(:, seqVec(1));
end
        


%% Outputs

