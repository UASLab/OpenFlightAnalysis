"""
University of Minnesota
Aerospace Engineering and Mechanics - UAV Lab
Copyright 2019 Regents of the University of Minnesota
See: LICENSE.md for complete license details

Author: Chris Regan
"""

import numpy as np

def SensorErrorModel(xMeas, param):
    ''' Compute the "True" Measure based on the error model parameters.
    %
    %Inputs:
    % xMeas   - measures
    % param   - error model parameters [errorType = 'None']
    %
    %Outputs:
    % xTrue   - true
    %
    %Notes:
    % There are no internal unit conversions.
    % Generally, Flight data analysis uses the "-" version of the models, simulation would use "+"
    '''
    
    ## Compute the True Measure    
    if param['errorType'].lower() == 'none':
        xTrue = xMeas
    elif param['errorType'].lower() == 'bias+': # Bias
        xTrue = xMeas + param['bias']
    elif param['errorType'].lower() == 'scalebias+': # Scale and Bias
        xTrue = param['K'] * xMeas + param['bias']
    else:
        print('Unkown Option')


    return xTrue
