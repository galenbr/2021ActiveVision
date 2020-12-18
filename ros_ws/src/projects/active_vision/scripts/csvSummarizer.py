#!/usr/bin/env python2

# import rospkg
import sys, csv,os, copy
import numpy as np
from prettytable import PrettyTable
import matplotlib.pyplot as plt

# mode = "GRAPH_VIEW"
mode = "GRAPH_SAVE"
# mode = "PRINT"
# mode = "CSV"
minYaw = 0
maxYaw = 180
nStepsSplit = 5

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

def genSummary(fileName,nBins):
    #Bins based on stepSize for yaw and space for nData,min,avg,max steps, split1, split2
    bins = []
    for bin in range(nBins):
        bins.append([(bin+1)*360/nBins,0,0,0,0,0,0])

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

    #Calculating the average
    for each in summary:
        for eachYaw in each[3]:
            if eachYaw[1] != 0:
                eachYaw[3] = round(eachYaw[3]/float(eachYaw[1]),2)
                eachYaw[5] = round(eachYaw[5]/float(eachYaw[1]),2)*100
                eachYaw[6] = round(eachYaw[6]/float(eachYaw[1]),2)*100

    return summary

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

def graphSummary(path,summary):
    for each in summary:
        # Create two subplots and unpack the output array immediately
        fig, (ax1, ax2) = plt.subplots(2, 1)
        fig.set_size_inches(13, 10)
        degree = []; number = []
        minGrasp = []; meanGrasp = []; maxGrasp = []
        split1 = []; split2 = []
        titleStr = "{}-Roll_{}_Pitch_{}".format(each[0], str(int(round(np.degrees(each[1]),0))), str(int(round(np.degrees(each[2]),0))))
        fig.suptitle(titleStr)
        for eachYaw in each[3]:
            if eachYaw[0] >= minYaw and eachYaw[0] <= maxYaw:
                degree.append(int(eachYaw[0]))
                number.append(eachYaw[1])
                minGrasp.append(float(eachYaw[2]))
                meanGrasp.append(float(eachYaw[3]))
                maxGrasp.append(float(eachYaw[4]))
                split1.append(float(eachYaw[5]));
                split2.append(float(eachYaw[6]));

        ax1.set_title('# Datapoints & Min-Avg-Max Steps for a grasp')
        ax1.set_xticks(degree)
        ax1.set_xticklabels(["<"+str(s) for s in degree])
        ax1.set_yticks(np.arange(max(maxGrasp)))
        ax1.grid(axis='y')
        ax1.set_xlabel('Yaw')
        ax1.set_ylabel('Steps to grasp')
        ax1.tick_params(axis='y', labelcolor="k")
        ax1.bar(degree, maxGrasp, width=8, color="tab:red", label='Max Steps')
        ax1.bar(degree, meanGrasp, width=8, color="tab:orange", label='Avg Steps')
        ax1.bar(degree, minGrasp, width=8, color="tab:green", label='Min Steps')
        ax1.legend()
        ax1A = ax1.twinx(); color = 'tab:blue'
        ax1A.set_ylabel('# Datapoints', color=color)
        ax1A.plot(degree, number, color=color, marker = "o")
        ax1A.tick_params(axis='y', labelcolor=color)

        ax2.grid(axis='y')
        ax2.set_title('Split of datapoints based on #Steps')
        ax2.set_xlabel('Yaw')
        ax2.set_xticks(degree)
        ax2.set_xticklabels(["<"+str(s) for s in degree])
        ax2.set_ylim(0, 100)
        ax2.set_ylabel('% Datapoints', color='k')
        ax2.bar(np.asarray(degree)-2, split1, width=4, color="tab:green", label='<= '+ str(nStepsSplit))
        ax2.bar(np.asarray(degree)+2, split2, width=4, color="tab:red", label='> '+ str(nStepsSplit))
        ax2.legend()

        if(mode == "GRAPH_SAVE"):
            newPath = path[:path.rfind('/')+1] + titleStr + ".png"
            plt.savefig(newPath)
            print("Graph saved to : "+newPath)
        elif(mode == "GRAPH_VIEW"):
            plt.show()

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
    os.chdir(os.path.expanduser("~"))

    # path = '/home/diyogon/Documents/WPI/Berk_Lab/mer_lab/ros_ws/src/projects/active_vision/data/2020_12_15_230829_dataRec.csv'
    path = './DataCollected/prismData/Prism_20x6x5_2000pts/Prism_20x6x5_2000pts.csv'
    # path = './DataCollected/prismData/Prism_10x8x4_2000pts/Prism_10x8x4_2000pts.csv'
    nBins = 360/10

    summary = genSummary(path,nBins)
    if(mode == "PRINT"):
        printSummary(summary)
    elif(mode == "GRAPH_VIEW" or mode == "GRAPH_SAVE"):
        graphSummary(path,summary)
    elif(mode == "CSV"):
        saveToCSV(path,summary)
