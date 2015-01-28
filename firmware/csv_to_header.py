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
for row in data:
    print "%3i,"%(row),
    i += 1
    if i > 9:
        print ""
        i = 0

print "};"
