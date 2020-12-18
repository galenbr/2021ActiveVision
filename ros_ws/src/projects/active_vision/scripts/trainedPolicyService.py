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

model = LogisticRegression(solver='liblinear', multi_class='auto')
pipeline = None

def helpDisp(text):
    print(text)
    print('\n-----Policy Training Help-----\n')
    print('Arguments : [StateVec]')
    print('StateVec : StateVec CSV file name')
    print('\n-----End Help-----\n')
    sys.exit()

def predictionServer(req):
    global pipeline,model
    stateVec = np.asarray(req.stateVec.data)
    stateVec = stateVec.reshape((1,stateVec.shape[0]))
    if pipeline != None:
        predDir = model.predict(pipeline.applyFunction(stateVec))
    else:
        predDir = model.predict(stateVec)
    rospy.loginfo("Service called. Direction -> "+str(predDir))
    return trainedPolicySRVResponse(direction=predDir)

if __name__ == "__main__":

    os.chdir(os.path.expanduser("~"))

    rospy.init_node('trainedPolicyServer')

    type = rospy.get_param("/active_vision/policyTester/policy")
    PCA_components = rospy.get_param("/active_vision/policyTester/PCA/PCAcomponents")
    PCALDA_components = rospy.get_param("/active_vision/policyTester/PCALDA/PCAcomponents")

    csvDir = rospkg.RosPack().get_path('active_vision') + "/misc/State_Vector/"
    stVecfile = rospy.get_param("/active_vision/policyTester/csvStVec")
    stateVec, dirVec = readInput(csvDir+stVecfile, 52)
    dirVec = dirVec.ravel()

    if type == "AS_IT_IS":
        model.fit(stateVec, dirVec)
    elif type == "PCA_LDA":
        pipeline = PCALDAPipeline(PCALDA_components)
        pipeline.calculateFunction(stateVec, dirVec)
        model.fit(pipeline.applyFunction(stateVec), dirVec)
    elif type == "PCA":
        pipeline = PCAPipeline(PCA_components)
        pipeline.calculateFunction(stateVec, dirVec)
        model.fit(pipeline.applyFunction(stateVec), dirVec)
    # print(pipeline.ready)
    s = rospy.Service('/active_vision/trained_policy', trainedPolicySRV, predictionServer)
    rospy.loginfo("Trained policy service ready.")
    rospy.spin()
