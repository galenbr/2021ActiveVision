#!/usr/bin/env python2

# import rospkg
import sys, time, random, math, rospy, os
import numpy as np
from matplotlib import pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.linear_model import LogisticRegression
from pickle import dump
import pickle

from active_vision.srv import trainedPolicySRV,trainedPolicySRVResponse

def helpDisp(text):
    print(text)
    print('\n-----Policy Training Help-----\n')
    print('Arguments : [Directory] [BaseName] [Type]')
    print('Directory : Directory where csv and pcd files are there (./DataRecAV/)')
    print('BaseName : Base/Common names of the .pkl file')
    print('Type : \'PCALDA\', \'PCA\' ')
    print('\n-----End Help-----\n')
    sys.exit()

scaler = None
pca = None
lda = None
lr = None

def PCA_LDA_LR_Server(req):
    global scaler,pca,lda,lr
    stateVec = np.asarray(req.stateVec.data)
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    predDir = lr.predict(lda.transform(pca.transform(scaler.transform(stateVec))))
    rospy.loginfo("PCA_LDA_LR service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

def PCA_LR_Server(req):
    global scaler,pca,lr
    stateVec = np.asarray(req.stateVec.data)
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    predDir = lr.predict(pca.transform(scaler.transform(stateVec)))
    rospy.loginfo("PCA_LR service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

if __name__ == "__main__":
    if len(sys.argv) != 4 and len(sys.argv) != 6: helpDisp("ERROR : Incorrent number of arguments")

    os.chdir(os.path.expanduser("~"))

    dir = sys.argv[1]
    commonName = sys.argv[2]
    type = sys.argv[3]

    rospy.init_node('trainedPolicyServer')
    if type == "PCA_LDA_LR":
        scaler = pickle.load(open(dir+commonName+'PL_scaler.pkl', 'rb'))
        pca = pickle.load(open(dir+commonName+'PL_pca.pkl', 'rb'))
        lda = pickle.load(open(dir+commonName+'PL_lda.pkl', 'rb'))
        lr = pickle.load(open(dir+commonName+'PL_lr.pkl', 'rb'))
        s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, PCA_LDA_LR_Server)
        rospy.loginfo("PCA_LDA_LR Trained policy service ready.")
        rospy.spin()
    elif type == "PCA_LR":
        scaler = pickle.load(open(dir+commonName+'P_scaler.pkl', 'rb'))
        pca = pickle.load(open(dir+commonName+'P_pca.pkl', 'rb'))
        lr = pickle.load(open(dir+commonName+'P_lr.pkl', 'rb'))
        s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, PCA_LR_Server)
        rospy.loginfo("PCA_LR Trained policy service ready.")
        rospy.spin()
    else:
        helpDisp("ERROR : Type")
