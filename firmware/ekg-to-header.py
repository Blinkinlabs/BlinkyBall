#!/bin/python

# Samples taken from here:
# https://github.com/brunoprog64/ecg-tesis/blob/master/EKG-Analysis/normal-ecg-01.csv


import csv

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]


with open('ekg-example.csv', 'rb') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar='\'')

    minVal = 9999.0
    maxVal = -9999.0
    data = []

    for row in spamreader:
        minVal = min(minVal, float(row[1]))
        maxVal = max(maxVal, float(row[1]))
        data.append(float(row[1]))

    minVal = -.2

    print "#define EKG_DATA_LENGTH %i"%(len(data))
    print "const uint8_t ekgData[EKG_DATA_LENGTH] PROGMEM = {"

    i = 0
    for row in data:
        
        print "%3i,"%(int(max(0,scale(row, (minVal, maxVal), (0,255))))),
        i += 1
        if i > 9:
            print ""
            i = 0
    
    print "};"
