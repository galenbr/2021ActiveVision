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

class methodPipeline(object):
    def __init__(self):
        self.ready = False
        return self

    def calculateFunction(self, X, y):
        return self.ready

    def normalize(self, X):
        scaler = StandardScaler()
        Xscaled = scaler.fit_transform(X)
        return Xscaled

    def applyFunction(self, X):
        return self.ready

class PCALDAPipeline(methodPipeline):
    def __init__(self, n_components):
        super(methodPipeline, self).__init__()
        self.n_components = n_components

    def calculateFunction(self, X, y):
        X = self.normalize(X)
        self.pca = PCA(self.n_components)
        X_components = self.pca.fit_transform(X)
        
        self.lda = LDA()
        self.lda.fit(X_components, y)
        self.ready = True
        return methodPipeline.calculateFunction(self, X, y)

    def applyFunction(self, X):
        if(not self.ready):
            return methodPipeline.applyFunction(self, X)
        else:
            X = self.normalize(X)
            X1 = self.pca.transform(X)
            X2 = self.lda.transform(X1)
            return X2

    def predictWFunction(self, X):
        if(not self.ready):
            return methodPipeline.applyFunction(self, X)
        else:
            X = self.normalize(X)
            X1 = self.pca.transform(X)
            X2 = self.lda.predict(X1)
            return X2

class PCAPipeline(methodPipeline):
    def __init__(self, n_components):
        super(methodPipeline, self).__init__()
        self.n_components = n_components

    def calculateFunction(self, X, y):
        X = self.normalize(X)
        self.pca = PCA(self.n_components)
        X_components = self.pca.fit_transform(X)

        self.ready = True
        return methodPipeline.calculateFunction(self, X, y)

    def applyFunction(self, X):
        if(not self.ready):
            return methodPipeline.applyFunction(self, X)
        else:
            X = self.normalize(X)
            X1 = self.pca.transform(X)
            return X1

    def predictWFunction(self, X):
        if(not self.ready):
            return methodPipeline.applyFunction(self, X)
        else:
            X = self.normalize(X)
            X1 = self.pca.predict(X)
            return X1

class BrickPipeline(methodPipeline):
    def __init__(self):
        super(methodPipeline, self).__init__()

    def calculateFunction(self, X, y):
        return methodPipeline.calculateFunction(self, X, y)

    def applyFunction(self, X):
        return methodPipeline.applyFunction(self, X)

    def predictWFunction(self, X):
        return np.ones(X.shape[0])

class BrickModel():
    def fit(self, X, y):
        return

    def predict(self, X):
        return np.ones(X.shape[0])

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

def kFold(filename, pipeline, model, dim=52, k=5):
    stateVec, dirVec = readInput(filename, dim)
    dirVec = dirVec.ravel()
    kFold = KFold(n_splits=k, random_state=None)
    accuracies = []
    for train_index, test_index in kFold.split(stateVec):
        X_train, X_test = stateVec[train_index, :], stateVec[test_index, :]
        y_train, y_test = dirVec[train_index], dirVec[test_index]
        pipeline.calculateFunction(X_train, y_train)
        X_train_vec = pipeline.applyFunction(X_train)
        X_test_vec = pipeline.applyFunction(X_test)
        model.fit(X_train_vec, y_train)
        pred_values = model.predict(X_test_vec)
        #pred_values = np.ones(pred_values.shape)
        acc = accuracy_score(pred_values, y_test)
        accuracies.append(acc)
    avg_accuracy = sum(accuracies)/k
    for acc in accuracies:
        print("%1.2f" % acc)
    print("Overall average: %1.2f" % (avg_accuracy))

def standardRun(file):
    model = LogisticRegression(solver='liblinear', multi_class='auto')
    print("*****%s*****" % file)
    print("*****PCA-LDA*****")
    kFold(file, PCALDAPipeline(26), model)
    print("*****PCA*****")
    kFold(file, PCAPipeline(7), model)
    print("*****Brick*****")
    kFold(file, PCAPipeline(7), BrickModel())

if __name__ == "__main__":
    #Galen mode for convenience- feel free to edit for your local files
    if len(sys.argv) == 1:
        files = ["../DataRecAV/stateVec.csv", "../GitData1/Step1_NormalVP/dataNormalVPStep1_stateVec.csv", "../GitData1/Step2_NormalVP/dataNormalVPStep2_stateVec.csv"]
        for file in files:
            standardRun(file)

    elif len(sys.argv) != 3: helpDisp("ERROR : Incorrent number of arguments")
    else:
        dir = sys.argv[1]
        file = sys.argv[2]
        standardRun(dir+file)
    
