#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 28 15:27:10 2018

@author: louismueller
"""

import json
    
def jsonRead(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
        f.close()
    return data
