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
import rospy
from active_vision.srv import trainedPolicySRV,trainedPolicySRVResponse
from trainingPolicy import PCALDAPipeline, PCAPipeline

model = None
pipeline = None

def helpDisp(text):
    print(text)
    print('\n-----Policy Training Help-----\n')
    print('Arguments : [BaseName]')
    print('BaseName : Base/Common names of the .pkl file')
    print('\n-----End Help-----\n')
    sys.exit()

def predictionServer(req):
    global pipeline,model
    stateVec = np.asarray(req.stateVec.data)
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    predDir = model.predict(pipeline.applyFunction(stateVec))
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

if __name__ == "__main__":
    if len(sys.argv) != 2 and len(sys.argv) != 4: helpDisp("ERROR : Incorrent number of arguments")

    os.chdir(os.path.expanduser("~"))

    dir = rospy.get_param("/active_vision/data_dir")
    type = rospy.get_param("/active_vision/model_type")
    PCA_components = rospy.get_param("/active_vision/PCA_component_number")
    PCALDA_components = rospy.get_param("/active_vision/PCALDA_component_number")
    commonName = sys.argv[1]

    rospy.init_node('trainedPolicyServer')
    model = pickle.load(open(dir+commonName+'_lr.pkl', 'rb'))
    if type == "PCA_LDA":
        pipeline = PCALDAPipeline(PCALDA_components).loadModel(dir+commonName)
    elif type == "PCA":
        pipeline = PCAPipeline(PCA_components).loadModel(dir+commonName)
    print(pipeline.ready)
    s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, predictionServer)
    rospy.loginfo("Trained policy service ready.")
    rospy.spin()