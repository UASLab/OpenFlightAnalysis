function [config] =  ReadConfig(fileRead)
%%

config = jsondecode(fileread(fileRead));

