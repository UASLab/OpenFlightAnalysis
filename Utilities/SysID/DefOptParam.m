function [optParam, param] = DefOptParam(optParam, param, type)
% Pack and UnPack structure fields into vectors for the optimizer


%%
numStatic = length(optParam.static);
numSeg = length(param);
numSegVar = length(optParam.seg);

% Create indices in the optParam Structure
indxArray = 1;
for iStatic = 1:numStatic
    lenCurr = optParam.static{iStatic}.len;
    optParam.static{iStatic}.indx = indxArray + [0:lenCurr - 1];
    indxArray = indxArray + lenCurr;
end
for iSeg = 1: numSeg
    for iVar = 1:numSegVar
        lenCurr = optParam.seg{iVar}.len;
        optParam.seg{iVar}.indx{iSeg} = indxArray + [0:lenCurr - 1];
        indxArray = indxArray + lenCurr;
    end
end

%%

switch lower(type)
    case {'pack'}
        % Loop through Static Parameters
        for iStatic = 1:numStatic
            field = optParam.static{iStatic}.field;
            fieldList = strsplit(field, '.');
            indxCurr = optParam.static{iStatic}.indx;
            optParam.val(indxCurr, :) = param{1}.(fieldList{1}).(fieldList{2});
            if isfield(optParam.static{iStatic}, 'lb')
                optParam.lb(indxCurr, :) = optParam.static{iStatic}.lb;
            else
                optParam.lb(indxCurr, :) = -Inf(size(optParam.val(indxCurr, :)));
            end
            if isfield(optParam.static{iStatic}, 'ub')
                optParam.ub(indxCurr, :) = optParam.static{iStatic}.ub;
            else
                optParam.ub(indxCurr, :) = Inf(size(optParam.val(indxCurr, :)));
            end
        end
        
        % Loop through Segment Variable Parameters
        for iSeg = 1: numSeg
            for iVar = 1:numSegVar
                field = optParam.seg{iVar}.field;
                fieldList = strsplit(field, '.');
                indxCurr = optParam.seg{iVar}.indx{iSeg};
                optParam.val(indxCurr, :) = param{iSeg}.(fieldList{1}).(fieldList{2});
                if isfield(optParam.seg{iVar}, 'lb')
                    optParam.lb(indxCurr, :) = optParam.seg{iVar}.lb;
                else
                    optParam.lb(indxCurr, :) = -Inf(size(optParam.val(indxCurr, :)));
                end
                if isfield(optParam.seg{iVar}, 'ub')
                    optParam.ub(indxCurr, :) = optParam.seg{iVar}.ub;
                else
                    optParam.ub(indxCurr, :) = Inf(size(optParam.val(indxCurr, :)));
                end
            end
        end
        
    case {'unpack'}
        
        % Loop through Static Parameters
        for iStatic = 1:numStatic
                field = optParam.static{iStatic}.field;
                fieldList = strsplit(field, '.');
                indxCurr = optParam.static{iStatic}.indx;
            for iSeg = 1: numSeg
                param{iSeg}.(fieldList{1}).(fieldList{2}) = optParam.val(indxCurr);
            end
        end
        
        % Loop through Segment Variable Parameters
        for iSeg = 1: numSeg
            for iVar = 1:numSegVar
                field = optParam.seg{iVar}.field;
                fieldList = strsplit(field, '.');
                indxCurr = optParam.seg{iVar}.indx{iSeg};
                param{iSeg}.(fieldList{1}).(fieldList{2}) = optParam.val(indxCurr);
            end
        end
        
    otherwise
end

