## this file contains some of the functions for properly interfacing with the LiDARs
## for some of the more knitty-gritty serial communication code, see HWserialConn.py


import numpy as np

from Map import Map
import GF.generalFunctions as GF #(homemade) some useful functions for everyday ease of use
from HWserialConn import kartMCUserialClass
from log.HWserialConnLogging import kartMCUserialLogger
