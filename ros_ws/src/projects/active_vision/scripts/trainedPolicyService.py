#!/usr/bin/env python2
import sys, time, random, math, rospy, os, rospkg
import numpy as np
from matplotlib import pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis as LDA
from sklearn.linear_model import LogisticRegression
from pickle import dump
import pickle
from active_vision.srv import trainedPolicySRV,trainedPolicySRVResponse
from trainingPolicy import PCALDAPipeline, PCAPipeline, readInput

#Currently only 1 model. TODO: add more/incorporate with pipeline class
model = LogisticRegression(solver='liblinear', multi_class='auto')
pipeline = None

def helpDisp(text):
    print(text)
    print('\n-----Policy Training Help-----\n')
    print('Arguments : [StateVec]')
    print('StateVec : StateVec CSV file name')
    print('\n-----End Help-----\n')
    sys.exit()

#Takes a state vector input and returns a predicted direction
def predictionServer(req):
    global pipeline,model
    stateVec = np.asarray(req.stateVec.data)
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    predDir = model.predict(pipeline.applyFunction(stateVec))
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

if __name__ == "__main__":

    os.chdir(os.path.expanduser("~"))

    #Load parameters from yaml
    rospy.init_node('trainedPolicyServer')
    type = rospy.get_param("/active_vision/policyTester/policy")
    PCA_components = rospy.get_param("/active_vision/policyTester/PCA/PCAcomponents")
    PCALDA_components = rospy.get_param("/active_vision/policyTester/PCALDA/PCAcomponents")
    stVecfile = rospy.get_param("/active_vision/policyTester/csvStVec")

    #Hardcoded state vector location- TODO: look into adding this to
    # the yaml?
    csvDir = rospkg.RosPack().get_path('active_vision') + rospy.get_param("/active_vision/policyTester/storageDir")

    #stateVec=List of all raw states
    #dirVec=List of the direction taken for each state
    #directions=N(1),NE(2),E(3),SE(4),S(5),SW(6),W(7),NW(8)
    stateVec, dirVec = readInput(csvDir+stVecfile, 52)
    dirVec = dirVec.ravel()

    if type == "PCA_LDA":
        pipeline = PCALDAPipeline(PCALDA_components)
        pipeline.calculateFunction(stateVec, dirVec)
        model.fit(pipeline.applyFunction(stateVec), dirVec)
    elif type == "PCA":
        pipeline = PCAPipeline(PCA_components)
        pipeline.calculateFunction(stateVec, dirVec)
        model.fit(pipeline.applyFunction(stateVec), dirVec)
    s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, predictionServer)
    rospy.loginfo("Trained policy service ready.")
    rospy.spin()
