#!/usr/bin/env python2

# import rospkg
import sys, csv, time, random, math
import numpy as np
from os import path
from matplotlib import pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA

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
            currentDirection = float(line[dim+3])
            #Remove final images
            if(-1 != currentDirection):
                states.append([float(i) for i in line[2:dim+2]])
                dir.append([currentDirection])

	return np.asarray(states),np.asarray(dir)

def paperPipeline(filename, dim=52):
    stateVec, dirVec = readInput(filename, dim)
    scaler = StandardScaler()
    stateVec = scaler.fit_transform(stateVec)

    pca = PCA(n_components=26)
    pca.fit(stateVec)
    statePCA = pca.transform(stateVec)

    lda = LDA()
    lda.fit(statePCA, dirVec.ravel())
    ouput = lda.transform(statePCA)

    print(stateVec.shape)
    print(statePCA.shape)
    print(ouput.shape)
    return pca, lda


if __name__ == "__main__":

    if len(sys.argv) != 3: helpDisp("ERROR : Incorrent number of arguments")

    # rospack = rospkg.RosPack()
    # path = rospack.get_path('active_vision')

    dir = sys.argv[1]
    file = sys.argv[2]
    paperPipeline(dir+file)
    
