#!/usr/bin/env python2

# import rospkg
import sys, csv, time, random, math
import numpy as np
from os import path
from matplotlib import pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.model_selection import train_test_split, KFold
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score

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

def calculatePCALDA(X, y):
    scaler = StandardScaler()
    Xscaled = scaler.fit_transform(X)

    pca = PCA(n_components=26)
    pca.fit(Xscaled)
    Xcomponents = pca.transform(Xscaled)
    
    lda = LDA()
    lda.fit(Xcomponents, y)

    return pca, lda

def pipeLine(X, pca, lda):
    scaler = StandardScaler()
    Xscaled = scaler.fit_transform(X)

    X1 = pca.transform(Xscaled)
    X2 = lda.transform(X1)

    return X2

def calculatePCA(X, y):
    scaler = StandardScaler()
    Xscaled = scaler.fit_transform(X)

    pca = PCA(n_components=7)
    pca.fit(Xscaled)

    return pca, 0

def PCApipeLine(X, pca, lda):
    scaler = StandardScaler()
    Xscaled = scaler.fit_transform(X)
    X1 = pca.transform(Xscaled)

    return X1

def paperPipeline(filename, dim=52):
    stateVec, dirVec = readInput(filename, dim)

    X_train, X_test, y_train, y_test = train_test_split(stateVec, dirVec.ravel(), test_size=0.2)
    
    pca, lda = calculatePCALDA(X_train, y_train)
    scaler = StandardScaler()
    testScaled = scaler.fit_transform(X_test)
    X1 = pca.transform(testScaled)
    y1 = lda.predict(X1)
    for i in range(y1.shape[0]):
        print(y1[i], y_test[i])

def kFold(filename, calcFunc, useFunc, dim=52, k=5):
    stateVec, dirVec = readInput(filename, dim)
    dirVec = dirVec.ravel()
    kFold = KFold(n_splits=k, random_state=None)
    model = LogisticRegression(solver='liblinear')
    accuracies = []
    for train_index, test_index in kFold.split(stateVec):
        X_train, X_test = stateVec[train_index, :], stateVec[test_index, :]
        y_train, y_test = dirVec[train_index], dirVec[test_index]
        cPCA, cLDA = calcFunc(X_train, y_train)
        X_train_vec = useFunc(X_train, cPCA, cLDA)
        X_test_vec = useFunc(X_test, cPCA, cLDA)
        model.fit(X_train_vec, y_train)
        pred_values = model.predict(X_test_vec)
        #pred_values = np.ones(pred_values.shape)
        acc = accuracy_score(pred_values, y_test)
        accuracies.append(acc)
    avg_accuracy = sum(accuracies)/k
    for acc in accuracies:
        print("%1.2f" % acc)
    print("Overall average: %1.2f" % (avg_accuracy))


if __name__ == "__main__":

    if len(sys.argv) != 3: helpDisp("ERROR : Incorrent number of arguments")

    # rospack = rospkg.RosPack()
    # path = rospack.get_path('active_vision')

    dir = sys.argv[1]
    file = sys.argv[2]
    paperPipeline(dir+file)
    kFold(dir+file, calculatePCA, PCApipeLine)
    
