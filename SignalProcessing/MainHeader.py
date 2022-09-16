"""
This file is used to import all libraries
"""
import scipy.fft as scipyFFT            #Fast Fourier Transform from Scipy
from scipy import signal                #Signal Processing library

#to make the executable as small as possible only import what you need
import numpy as np                         #generic data analysis/storage
import numpy.ma as ma                       #creating masks for analysis

import codecs                           #csv file has annoying encoding
import matplotlib.pyplot as plt         #graphing functions
import math                             #sine, pi

#benchmarking
import timeit
import time
""""
from numpy import loadtxt as loadtxt      #reading csv file
from numpy import linspace as linspace    #creating linear time axis

from tkinter import Button              #genric buttons
from tkinter import Spinbox             #spin dials
from tkinter import Label               #widget label
from tkinter import Text                #text boxes
from tkinter import END                 #END of text box
from tkinter import Tk                  #Window
"""
from tkinter import filedialog as fd    #file chooser
import tkinter as tk                    #gui
from tkinter.font import Font

from FileSystems import *
from MotorClass import *
from GraphingFunctions import *
from CurrentAnalysis import *
from FilterClass import *

from Analysis import *
from main import *
from GUI import *


