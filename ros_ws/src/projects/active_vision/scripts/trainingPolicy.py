#!/usr/bin/env python2

# import rospkg
import sys, csv, time, random, math
import numpy as np
from os import path
from matplotlib import pyplot as plt

def helpDisp(text):
    print(text)
    print('\n-----Policy Training Help-----\n')
    print('Arguments : [Directory] [CSV filename]')
    print('Directory : Directory where csv and pcd files are there (./DataRecAV/)')
    print('CSV filename : CSV file name (Data.csv)')
    print('\n-----End Help-----\n')
    sys.exit()

#Function to read the input data file
def readInput(fileName,dim):
    states = []
    dir = []
    with open(fileName) as csv_file:
        # Reading the csv file
        csv_reader = csv.reader(csv_file, delimiter=',')
        for line in csv_reader:
            # Going through it line by line as storing it in a list
            if(len(line) != dim+4): helpDisp("ERROR : Data set inconsistent")
            states.append([float(i) for i in line[2:dim+2]])
            dir.append([float(i) for i in line[dim+3]])

	return np.asarray(states),np.asarray(dir)

if __name__ == "__main__":

    if len(sys.argv) != 3: helpDisp("ERROR : Incorrent number of arguments")

    # rospack = rospkg.RosPack()
    # path = rospack.get_path('active_vision')

    dir = sys.argv[1]
    file = sys.argv[2]

    stateVec,dirVec = readInput(dir+file,52)
    print(dirVec[0][0])
