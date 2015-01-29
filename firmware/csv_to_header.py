import csv

data = []

with open('seven_cycles.csv', 'rb') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        for entry in row:
            data.append(int(entry))

print "#define EKG_DATA_LENGTH %i"%(len(data))
print "const uint8_t ekgData[EKG_DATA_LENGTH] PROGMEM = {"

i = 0
for value in data:
    # Do a simple square brightness correction on the data
    value = pow(value/255.0, 2)*255

    print "%3i,"%(value),
    i += 1
    if i > 9:
        print ""
        i = 0

print "};"
