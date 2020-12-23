#!/usr/bin/env python2
import sys, csv,os, copy
import numpy as np
from prettytable import PrettyTable
import matplotlib.pyplot as plt

# Y-limit for histogram
minYaw = 0
maxYaw = 180
# Path length cutoff for analysis
nStepsSplit = 3

#Function to read the input data file
def readInput(fileName):
    data = []
    with open(fileName) as csv_file:
        # Reading the csv file
        csv_reader = csv.reader(csv_file, delimiter=',')
        for line in csv_reader:
            # Going through it line by line as storing it in a list
            data.append([i for i in line[:]])

	return np.asarray(data)

#Read file and plot camera angle versus initial direction chosen. TODO:
# Move to a separate file? Refactor for more general data analysis?
def PCASummary(fileName):
    data = readInput(fileName)
    plt.figure(figsize=(10,9))
    plt.xlabel('horizontal',fontsize=20)
    plt.ylabel('azimuthal',fontsize=20)
    plt.title("Prism_10x8x4_Pose_1.57x1.57 Direction analysis",fontsize=20)
    h = []
    a = []
    d = []
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red',
              'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray']
    labels = ['N','NE','E','SE','S','SW','W','NW']
    for i in range(len(data)):
        each = data[i]
        if(each[1]==each[2]):
            nSteps = (len(each)-14)/3-1
            for step in range(nSteps):
                offset = step*3
                h.append(np.degrees(float(each[15+offset])))
                a.append(np.degrees(float(each[16+offset])))
                d.append(int(each[12])-1)
    h = np.array(h)
    a = np.array(a)
    d = np.array(d)
    for j in range(8):
        target_index = d == j
        plt.scatter(h[target_index], a[target_index], c=colors[j], label=labels[j])
    plt.legend()
    plt.show()

'''Read file and summarize information about each interval of camera data.
summary = [[objName, objRoll, objPitch, [YawData]]]
YawData = [yawEndAngle, #ofDataPoints, minSteps, avgSteps, maxSteps, #stepsBelownStepsSplit, #stepsAbovenStepsSplit, [dataPoints]]'''
def genSummary(fileName,nBins):
    bins = []
    for bin in range(nBins):
        bins.append([(bin+1)*360/nBins,0,0,0,0,0,0,[]])

    #Finding all unique configurations
    data = readInput(fileName)
    summary = []
    unique_list = []
    for each in data:
        param = each[0]+"-"+each[1]+"-"+each[2]
        if param not in unique_list:
            unique_list.append(param)
            summary.append([each[0],float(each[1]),float(each[2]),copy.deepcopy(bins)])
        index = unique_list.index(param)
        nSteps = (len(each)-14)/3-1
        binIndex = int(int(round(np.degrees(float(each[3]))))/(360/nBins))

        summary[index][3][binIndex][1] += 1
        summary[index][3][binIndex][3] += nSteps  # Will be dividived my nData in the end
        if summary[index][3][binIndex][1] == 1:
            summary[index][3][binIndex][2] = nSteps
            summary[index][3][binIndex][4] = nSteps
        else:
            summary[index][3][binIndex][2] = min(nSteps,summary[index][3][binIndex][2])
            summary[index][3][binIndex][4] = max(nSteps,summary[index][3][binIndex][4])
        if nSteps <= nStepsSplit:
            summary[index][3][binIndex][5] += 1
        else:
            summary[index][3][binIndex][6] += 1
        summary[index][3][binIndex][7].append(nSteps)

    #Calculating the average
    for each in summary:
        for eachYaw in each[3]:
            if eachYaw[1] != 0:
                eachYaw[3] = round(eachYaw[3]/float(eachYaw[1]),2)
                eachYaw[5] = round(eachYaw[5]/float(eachYaw[1]),2)*100
                eachYaw[6] = round(eachYaw[6]/float(eachYaw[1]),2)*100

    return summary

#Output neat table of summary.
def printSummary(summary):
    t = PrettyTable(['Bins(Yaw)', '#Data' , 'Min Steps', 'Avg Steps', 'Max Steps', '% <= '+ str(nStepsSplit) + ' steps', '% > '+ str(nStepsSplit)  + ' steps'])
    for each in summary:
        t.clear_rows()
        t.title = each[0] + " - Roll : " + str(int(round(np.degrees(each[1]),0))) + " , Pitch : " + str(int(round(np.degrees(each[2]),0)))
        for eachYaw in each[3]:
            if eachYaw[0] >= minYaw and eachYaw[0] <= maxYaw:
                if eachYaw[1] > 0:
                    t.add_row([eachYaw[0] ,eachYaw[1], eachYaw[2], eachYaw[3], eachYaw[4] , eachYaw[5] , eachYaw[6]])
                else:
                    t.add_row([eachYaw[0] , " ", " " , " " , " " , " ", " "])
        print(t)

#Generate a bar graph of the summary
def graphSummary(path,summary):
    # Extracting different objects
    unique_list = []
    nUnique = []
    for each in summary:
        if each[0] not in unique_list:
            unique_list.append(each[0])
            nUnique.append(1)
        else:
            nUnique[unique_list.index(each[0])]+=1

    # Plotting each object in a different graph
    for obj,nPlots in zip(unique_list,nUnique):
        rows = min(3,nPlots); cols = int((nPlots-1)/3)+1
        fig, ax = plt.subplots(rows,cols); ax = np.ravel(ax)
        fig.set_size_inches(cols*7, 3*rows)
        fig.suptitle(obj+' - Box plot of number of steps for a successful grasp', fontsize='medium')
        idx = 0
        for each in summary:
            if each[0] == obj:
                degree = []; number = []
                minGrasp = []; meanGrasp = []; maxGrasp = []
                split1 = []; split2 = []
                bxPlt = [];
                titleStr = "Config : Roll - {} , Pitch - {}".format(str(int(round(np.degrees(each[1]),0))), str(int(round(np.degrees(each[2]),0))))
                for eachYaw in each[3]:
                    if eachYaw[0] >= minYaw and eachYaw[0] <= maxYaw:
                        degree.append(int(eachYaw[0]))
                        number.append(eachYaw[1])
                        minGrasp.append(float(eachYaw[2]))
                        meanGrasp.append(float(eachYaw[3]))
                        maxGrasp.append(float(eachYaw[4]))
                        split1.append(float(eachYaw[5]))
                        split2.append(float(eachYaw[6]))
                        bxPlt.append(eachYaw[7])

                ax[idx].set_title(titleStr, fontsize='small')

                bp = ax[idx].boxplot(bxPlt,positions=np.arange(len(bxPlt)),meanline=True,showmeans=True,patch_artist=True)
                for median in bp['medians']:
                    median.set(color ='red',linewidth = 2)
                for mean in bp['means']:
                    mean.set(color ='blue',linewidth = 1)
                for patch in bp['boxes']:
                    patch.set_facecolor('lightgreen')

                ax[idx].grid(axis='y')
                ax[idx].set_yticks(np.arange(8))
                ax[idx].set_ylabel('# Steps to grasp', fontsize='x-small')
                ax[idx].yaxis.set_tick_params(labelsize=7)
                ax[idx].tick_params(axis='y', labelcolor="k")
                ax[idx].set_ylim(bottom=0)
                ax[idx].set_ylim(top=8)

                ax[idx].set_xlabel('Yaw bins\n(# Datapoints)', fontsize='x-small')
                ax[idx].xaxis.set_tick_params(labelsize=7)
                # ax[idx].set_xticklabels(["<"+str(s) for s in degree])
                ax[idx].set_xticklabels(["<"+str(s)+"\n("+str(n)+")" for s,n in zip(degree,number)])

                # binSize = degree[1]-degree[0]
                # ax[idx].set_xticklabels(["["+str(s-binSize)+","+str(s)+")" for s in degree])
                # ax[idx].xaxis.set_tick_params(rotation=90)

                # ax2.grid(axis='y')
                # ax2.set_title('Split of datapoints based on #Steps')
                # ax2.set_xlabel('Yaw')
                # ax2.set_xticks(degree)
                # ax2.set_xticklabels(["<"+str(s) for s in degree])
                # ax2.set_ylim(0, 100)
                # ax2.set_ylabel('% Datapoints', color='k')
                # ax2.bar(np.asarray(degree)-2, split1, width=4, color="tab:green", label='<= '+ str(nStepsSplit))
                # ax2.bar(np.asarray(degree)+2, split2, width=4, color="tab:red", label='> '+ str(nStepsSplit))
                # ax2.legend()

                idx+=1

        fig.legend([bp['medians'][0], bp['means'][0]], ['median', 'mean'],loc='lower center',ncol=2,fontsize='x-small')
        # fig.legend()
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        if(mode == "GRAPH_SAVE"):
            newPath = path[:path.rfind('/')+1] + obj+'_summary' + ".png"
            plt.savefig(newPath, dpi = 600)
            print("Graph saved to : "+newPath[newPath.rfind('/')+1:])
        elif(mode == "GRAPH_VIEW"):
            plt.show()

#Save the summary for future use.
def saveToCSV(path,summary):
    newPath = path[:-4] + "_summary.csv"
    with open(newPath, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        for each in summary:
            for eachYaw in each[3]:
                if eachYaw[0] >= minYaw and eachYaw[0] <= maxYaw:
                    csvwriter.writerow([each[0], each[1], each[2], eachYaw[0] ,eachYaw[1], eachYaw[2], eachYaw[3], eachYaw[4], eachYaw[5], eachYaw[6]])

    print("Data Written to : "+newPath)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Incorrect number of arguments")
        sys.exit()

    path = sys.argv[1]
    mode = sys.argv[2]
    if mode!='GRAPH_VIEW' and mode!='GRAPH_SAVE' and mode!='PRINT' and mode!='CSV' :
        mode = 'GRAPH_VIEW'

    nBins = 360/10

    summary = genSummary(path,nBins)
    if(mode == "PRINT"):
        printSummary(summary)
    elif(mode == "GRAPH_VIEW" or mode == "GRAPH_SAVE"):
        graphSummary(path,summary)
    elif(mode == "CSV"):
        saveToCSV(path,summary)
    elif(mode == "PCA"):
        PCASummary(path)
