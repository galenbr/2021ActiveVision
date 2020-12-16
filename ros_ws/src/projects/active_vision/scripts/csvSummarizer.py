#!/usr/bin/env python2

# import rospkg
import sys, csv,os, copy
import numpy as np
from prettytable import PrettyTable

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
    #Bins based on stepSize for yaw and space for nData,min,avg,max steps
    bins = []
    for bin in range(nBins):
        bins.append([(bin+1)*360/nBins,0,0,0,0])

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
        binIndex = int(int(np.degrees(float(each[3])))/(360/nBins))

        summary[index][3][binIndex][1] += 1
        summary[index][3][binIndex][3] += nSteps  # Will be dividived my nData in the end
        if summary[index][3][binIndex][1] == 1:
            summary[index][3][binIndex][2] = nSteps
            summary[index][3][binIndex][4] = nSteps
        else:
            summary[index][3][binIndex][2] = min(nSteps,summary[index][3][binIndex][2])
            summary[index][3][binIndex][4] = max(nSteps,summary[index][3][binIndex][4])

    #Calculating the average
    for each in summary:
        for eachYaw in each[3]:
            if eachYaw[1] != 0:
                eachYaw[3] = round(eachYaw[3]/float(eachYaw[1]),2)

    return summary

def printSummary(summary):
    t = PrettyTable(['Bins(Yaw)', '#Data' , 'Min Steps', 'Avg Steps', 'Max Steps'])
    for each in summary:
        t.clear_rows()
        t.title = each[0] + " - Roll : " + str(round(np.degrees(each[1]),0)) + " , Pitch : " + str(round(np.degrees(each[2]),0))
        for eachYaw in each[3]:
            if eachYaw[1] > 0:
                t.add_row([eachYaw[0] ,eachYaw[1], eachYaw[2], eachYaw[3], eachYaw[4]])
            else:
                t.add_row([eachYaw[0] ,' ', ' ', ' ', ' '])
        print(t)

def saveToCSV(path,summary):
    newPath = path[:-4] + "_summary.csv"
    with open(newPath, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        for each in summary:
            for eachYaw in each[3]:
                csvwriter.writerow([each[0], each[1], each[2], eachYaw[0] ,eachYaw[1], eachYaw[2], eachYaw[3], eachYaw[4]])

    print("Data Written to : "+newPath)

if __name__ == "__main__":
    os.chdir(os.path.expanduser("~"))

    # path = './DataCollected/prismData/Prism_10x8x4_2000pts/Prism_10x8x4_2000pts.csv'
    path = './DataCollected/prismData/Prism_20x6x5_2000pts/Prism_20x6x5_2000pts.csv'
    nBins = 360/10

    summary = genSummary(path,nBins)
    printSummary(summary)
    # saveToCSV(path,summary)
