function [varargout] = RotMat(seqList, angleList, units)
% OpenFlightAnalysis - RotRateMat
%  Compute the rotation matrix given a sequence and angles.
%
% Inputs:
%  seqList   - cell array list of sequences
%  angleList - cell array angle values associated with the sequence
%  units     - units of angles [rad]
%
% Outputs:
%  rotMat     - Rotation Matrix
%  rotMatList - Cell array list of rotation matrices
%
% Notes:
%  seqList is a cell array of the form {'3','2','1'} or a string of the form '321'
%  angleList is a cell array of the form {a1,a2,a3} or a vector of the form [a1,a2,a3]
%  The angleList associates with sequence described by the seqList:
%  For a '321' it becomes T = R1(a1)*R2(a2)*R3(a3)
%  For a '312' it becomes T = R2(a2)*R1(a1)*R3(a3)
%  For a '123' it becomes T = R3(a3)*R2(a2)*R1(a1)
%  For a '32' it becomes T = R3(a2)*R2(a1)
%  For a '21' it becomes T = R2(a2)*R1(a1)
%
% University of Minnesota
% Aerospace Engineering and Mechanics - UAV Lab
% Copyright (c) 2019 Regents of the University of Minnesota
% See: LICENSE.md for complete license details

% Author: Chris Regan
% 09/20/2016  C. Regan     Initial Release
% 09/23/2016  C. Regan     Switched to using input parsing
% 09/27/2016  C. Regan     Fixed ordering of outputs
% 09/28/2016  C. Regan     Added vector inputs rather than cell array inputs
% 10/06/2016  C. Regan     Fixed Angle Sequencing
% 10/17/2016  C. Regan     Changed to simple and faster input parsing
% 10/17/2016  C. Regan     Added hard-coded '321' case for speed
%

%% Check I/O Arguments
% Check the number of inputs
narginchk(2, 3);
if nargin < 3
    units = [];
end

% Default Values
if isempty(units), units = 'rad'; end

% Check the number of outputs
nargoutchk(0, 2);


%% Constants
r2d = 180/pi;
d2r = 1/r2d;


%% Check Inputs
% Convert a cell array sequence list to a string
if iscell(seqList), seqList = cell2mat(seqList); end
if iscell(angleList), angleList = cell2mat(angleList); end

lenSeq = length(seqList);
lenAngle = length(angleList);
if lenSeq ~= lenAngle, error('Input Sequences must be of the same length'); end

%% Compute the Rotation Matrices

switch lower(units)
    case {'deg'}
        angleList = d2r * angleList;
    otherwise
end

switch lower(seqList)
    case {'321'}
        % Pre-compute the cos and sin of the angle
        cosAngle1 = cos(angleList(1));
        sinAngle1 = sin(angleList(1));
        cosAngle2 = cos(angleList(2));
        sinAngle2 = sin(angleList(2));
        cosAngle3 = cos(angleList(3));
        sinAngle3 = sin(angleList(3));

        % Compute the rotation Matrices
        rotMatList{1} = [1, 0, 0; 0, cosAngle1, sinAngle1; 0, -sinAngle1, cosAngle1];
        rotMatList{2} = [cosAngle2, 0, -sinAngle2; 0, 1, 0; sinAngle2, 0, cosAngle2];
        rotMatList{3} = [cosAngle3, sinAngle3, 0; -sinAngle3, cosAngle3, 0; 0, 0, 1];

        % Compute the Total Rotation Matrix
        rotMat = rotMatList{1} * rotMatList{2} * rotMatList{3};

    otherwise %% The slow way...
        rotMatList = cell(lenSeq, 1);
        rotMatList{1} = eye(3);

        for indxSeq = 1:lenSeq
            seq = seqList(indxSeq);
            angle = angleList(end-indxSeq+1);

            % Pre-compute the cos and sin of the angle
            cosAngle = cos(angle);
            sinAngle = sin(angle);

            % Compute the rotation Matrix
            switch seq
                case {'1'}
                    rotMatList{indxSeq} = [1, 0, 0; 0, cosAngle, sinAngle; 0, -sinAngle, cosAngle];
                case {'2'}
                    rotMatList{indxSeq} = [cosAngle, 0, -sinAngle; 0, 1, 0; sinAngle, 0, cosAngle];
                case {'3'}
                    rotMatList{indxSeq} = [cosAngle, sinAngle, 0; -sinAngle, cosAngle, 0; 0, 0, 1];
                otherwise
                    error('Invalid Sequence Descriptor');
            end
        end
        % Compute the product of the sequence, recall the sequence is reversed for the multiplication
        rotMat = rotMatList{end};

        if lenSeq > 1
            for indxSeq = (lenSeq-1):-1:1
                rotMat = rotMat * rotMatList{indxSeq};
            end
        end
end


%% Outputs
varargout{1} = rotMat;
varargout{2} = rotMatList;
