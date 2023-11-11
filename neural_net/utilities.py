#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import datetime

def get_current_timestamp():
    current_local_time = datetime.now()
    milliseconds = current_local_time.microsecond // 1000
    
    return current_local_time.strftime("%Y-%m-%d-%H-%M-%S") + f"-{milliseconds:03d}"

