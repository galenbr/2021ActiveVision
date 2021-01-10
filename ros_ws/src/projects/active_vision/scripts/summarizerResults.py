#!/usr/bin/env python2
import sys, csv,os, copy, io
import numpy as np
from prettytable import PrettyTable
import matplotlib.pyplot as plt
from collections import OrderedDict
from summarizerDataCollected import readInput
from PIL import Image

heuristicPolicies = ["Heuristic","Random","Brick"]
maxSteps = 5

graphColors =  ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
graphMarkers = ["o", "v", "^", "<", ">", "*", "+", "s"]

#Convert a Matplotlib figure to a PIL Image and return it
def fig2img(fig):
    buf = io.BytesIO()
    fig.savefig(buf)
    buf.seek(0)
    img = Image.open(buf)
    return img

#Convert the plots to a jpg format
def plots2jpg(plots,path,prefix):
    if len(plots) > 0:
        dims = plots[0].size
        nFigures = np.ceil(len(plots)/6.0)
        ctr = 1
        for figID in range(len(plots)):
            idx = figID % 6
            if idx == 0:
                remainingPlots = len(plots) - idx
                rows = min(2,remainingPlots)
                cols = min(3,(remainingPlots+1)/2)
                newImg = Image.new('RGB', (cols*dims[0], rows*dims[1]))
            newImg.paste(plots[idx],((idx/2)%3*dims[0],idx%2*dims[1]))
            if idx == 5 or figID == len(plots) - 1:
                newPath = path+prefix+str(ctr)+".jpg"
                newImg.save(newPath)
                print("Graph saved to : "+newPath[newPath.rfind('/')+1:])
                ctr += 1

'''Read file and summarize information about differnt experiments.
'''
def genSummary(path,fileNames):
    # Bins for storing the number of data points with x steps for successful grasp
    # 0,1,2,3...max,>max
    bins = [0]*(maxSteps+2)

    # Setting up the dictionaries which will store the summaries
    policyWise = {}
    stVecWise = {}
    for file in fileNames:
        data = readInput(path+file)
        policy = file.split(":")[0]
        stVec = file.split(":")[1]

        # Extracting the unique objects
        objects = []
        for each in data:
            obj = each[0]+"-"+each[1]+"-"+each[2]
            objects.append(obj)
        objects = list(dict.fromkeys(objects))

        # print(policy,stVec,objects)
        for obj in objects:
            keyPolicy = obj+"*"+policy
            keyStVec = obj+"*"+stVec
            if policy in heuristicPolicies:
                policyWise[keyPolicy] = np.array(bins)
            else:
                if policyWise.has_key(keyPolicy) == False:
                    policyWise[keyPolicy] = {}
                policyWise[keyPolicy][stVec] = np.array(bins)
                if stVecWise.has_key(keyStVec) == False:
                    stVecWise[keyStVec] = {}
                stVecWise[keyStVec][policy] = np.array(bins)

    # Updating the generated summary
    for file in fileNames:
        data = readInput(path+file)
        policy = file.split(":")[0]
        stVec = file.split(":")[1]

        for each in data:
            obj = each[0]+"-"+each[1]+"-"+each[2]
            keyPolicy = obj+"*"+policy
            keyStVec = obj+"*"+stVec
            nSteps = max(1,min((len(each)-14)/3-1,maxSteps+1))
            if policy in heuristicPolicies:
                policyWise[keyPolicy][nSteps] += 1
            else:
                policyWise[keyPolicy][stVec][nSteps] += 1
                stVecWise[keyStVec][policy][nSteps] += 1

    # for k, v in policyWise.items():
    #     print '%60s' % k, ' : ', v
    # print("-------")
    # for k, v in stVecWise.items():
    #     print '%60s' % k, ' : ', v
    # print("----------------")
    policyWise =  OrderedDict(sorted(policyWise.items()))
    stVecWise =  OrderedDict(sorted(stVecWise.items()))
    return policyWise,stVecWise

#Generate a bar graph of the summary
def graphSummary(path,policyWise,stVecWise):

    xAxis = [str(i) for i in range(maxSteps+1)]

    # Creating stVecWise plots i.e. policy comparisons
    plots = []
    for key, value in stVecWise.items():
        keywords = key.split("*")
        fig, ax = plt.subplots(1,1); ax = np.ravel(ax)
        fig.set_size_inches(6, 4)
        fig.suptitle("Step number distribution - Comparison of various policies", fontsize='medium')
        idx = 0
        # print key, value
        for policy, summary in value.items():
            temp = np.cumsum(summary)
            nData = temp[-1]
            avg = round(np.sum(np.array(summary)*np.array(range(len(summary))))/float(nData),1)
            temp = temp / float(temp[-1])
            # print "\t", policy, "--->" ,temp
            ax[0].plot(xAxis,temp[0:maxSteps+1]*100,color=graphColors[idx], marker=graphMarkers[idx], label = policy + " (" + str(avg) + ")")
            idx = idx + 1

        # Adding the heuristic result if available
        for policy in heuristicPolicies:
            keyPolicy = keywords[0]+"*"+policy
            if policyWise.has_key(keyPolicy):
                temp = np.cumsum(policyWise[keyPolicy])
                nData = temp[-1]
                avg = round(np.sum(np.array(policyWise[keyPolicy])*np.array(range(len(policyWise[keyPolicy]))))/float(nData),1)
                temp = temp / float(temp[-1])
                # print "\t", policy, "--->" ,temp
                ax[0].plot(xAxis,temp[0:maxSteps+1]*100,color=graphColors[idx], marker=graphMarkers[idx], label = policy + " (" + str(avg) + ")")
                idx = idx + 1

        objDetails = keywords[0].split("-")
        titleStr = "{} : Roll - {} , Pitch - {} \n State Vec : {}".format(
                    objDetails[0],
                    str(int(round(np.degrees(float(objDetails[1])),0))),
                    str(int(round(np.degrees(float(objDetails[2])),0))),
                    keywords[1])
        ax[0].set_title(titleStr , fontsize='small')

        ax[0].grid(axis='y')
        ax[0].set_ylabel('Success percentage', fontsize='small')

        ax[0].set_xlabel('Step number that leads to a successful grasp (Avg. steps in legend)', fontsize='small')

        ax[0].legend(fontsize='small')
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])

        plots.append(fig2img(fig))

    plots2jpg(plots,path,"policyComparison_")

    # Creating policyWise plots i.e. state vector comparisons
    plots = []
    for key, value in policyWise.items():
        # Skipping the heuristics
        if isinstance(value,dict) == False:
            continue
        # Skipping if only one state vector is there
        if len(value) < 2:
            continue

        keywords = key.split("*")
        fig, ax = plt.subplots(1,1); ax = np.ravel(ax)
        fig.set_size_inches(6, 4)
        fig.suptitle("Step number distribution - Comparison of various state vectors", fontsize='medium')
        idx = 0

        for stVec, summary in value.items():
            temp = np.cumsum(summary)
            nData = temp[-1]
            avg = round(np.sum(np.array(summary)*np.array(range(len(summary))))/float(nData),1)
            temp = temp / float(temp[-1])
            # print "\t", policy, "--->" ,temp
            ax[0].plot(xAxis,temp[0:maxSteps+1]*100,color=graphColors[idx], marker=graphMarkers[idx], label = stVec + " (" + str(avg) + ")")
            idx = idx + 1

        objDetails = keywords[0].split("-")
        titleStr = "{} : Roll - {} , Pitch - {} \n Policy : {}".format(
                    objDetails[0],
                    str(int(round(np.degrees(float(objDetails[1])),0))),
                    str(int(round(np.degrees(float(objDetails[2])),0))),
                    keywords[1])
        ax[0].set_title(titleStr , fontsize='small')

        ax[0].grid(axis='y')
        ax[0].set_ylabel('Success percentage', fontsize='small')

        ax[0].set_xlabel('Step number that leads to a successful grasp (Avg. steps in legend)', fontsize='small')

        ax[0].legend(fontsize='small')
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])

        plots.append(fig2img(fig))

    plots2jpg(plots,path,"stVecComparison_")

    # Heuristics only comparison
    plots = []
    unique = []
    for key, value in policyWise.items():
        # Skipping non heuristics
        if isinstance(value,dict):
            continue

        keywords = key.split("*")
        if keywords[0] not in unique:
            unique.append(keywords[0])

            fig, ax = plt.subplots(1,1); ax = np.ravel(ax)
            fig.set_size_inches(6, 4)
            fig.suptitle("Step number distribution - Comparison of various heuristics", fontsize='medium')
            idx = 0

            # Adding the heuristic result if available
            for policy in heuristicPolicies:
                keyPolicy = keywords[0]+"*"+policy
                if policyWise.has_key(keyPolicy):
                    temp = np.cumsum(policyWise[keyPolicy])
                    nData = temp[-1]
                    avg = round(np.sum(np.array(policyWise[keyPolicy])*np.array(range(len(policyWise[keyPolicy]))))/float(nData),1)
                    temp = temp / float(temp[-1])
                    # print "\t", policy, "--->" ,temp
                    ax[0].plot(xAxis,temp[0:maxSteps+1]*100,color=graphColors[idx], marker=graphMarkers[idx], label = policy + " (" + str(avg) + ")")
                    idx = idx + 1

            objDetails = keywords[0].split("-")
            titleStr = "{} : Roll - {} , Pitch - {}".format(
                        objDetails[0],
                        str(int(round(np.degrees(float(objDetails[1])),0))),
                        str(int(round(np.degrees(float(objDetails[2])),0))))
            ax[0].set_title(titleStr , fontsize='small')

            ax[0].grid(axis='y')
            ax[0].set_ylabel('Success percentage', fontsize='small')

            ax[0].set_xlabel('Step number that leads to a successful grasp (Avg. steps in legend)', fontsize='small')

            ax[0].legend(fontsize='small')
            fig.tight_layout(rect=[0, 0.03, 1, 0.95])

            plots.append(fig2img(fig))

    plots2jpg(plots,path,"heuristicComparison_")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Incorrect number of arguments")
        sys.exit()

    path = sys.argv[1]

    reqCSVs = []
    for root,dirs,files in os.walk(path):
        for file in files:
           if file.endswith(":dataRec.csv"):
               reqCSVs.append(file)

    policyWise,stVecWise = genSummary(path,reqCSVs)
    graphSummary(path,policyWise,stVecWise)
