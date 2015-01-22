import math

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]
    
minVal = 9999.0
maxVal = -9999.0
data = []

counts = 500

for xInt in range(0,counts):

    x = float(xInt)/counts
    value = (math.exp(-300 * pow(x-.02,2)) + math.exp(-100 * pow(x-0.19,2))) / 1.056

    minVal = min(minVal, value)
    maxVal = max(maxVal, value)
    data.append(value)

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


