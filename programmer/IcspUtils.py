import subprocess
import time
import optparse
import sys
import cStringIO
import signal

TIMEOUT = 10

class Alarm(Exception):
    pass

def alarm_handler(signum, frame):
    raise Alarm

def writeFuses(eFuses, hFuses, lFuses, programmer="usbtiny"):
  """
  Attempt to write the fuses of the attached Atmega device.

  """
  command = [
    "avrdude",
    "-c", programmer,
    "-p", "attiny85",
    "-B", "200",
    "-e",
    "-U", "efuse:w:%#02X:m" % eFuses,
    "-U", "hfuse:w:%#02X:m" % hFuses,
    "-U", "lfuse:w:%#02X:m" % lFuses,
  ]

  s = open('result.log','w')
  e = open('errorresult.log','w')
  result = 1

  signal.signal(signal.SIGALRM, alarm_handler)
  signal.alarm(TIMEOUT)
  try:
    result = subprocess.call(command, stdout=s, stderr=e)
    signal.alarm(0)
  except Alarm:
    pass

  s.close()
  e.close()

  s = open('result.log','r')
  e = open('errorresult.log','r')
  stdout = s.readlines()
  stderr = e.readlines()
  s.close()
  e.close()

  return result, stdout, stderr
  
def loadFlash(flashFile, programmer="usbtiny"):
  """
  Attempt to write a .hex file to the flash of the attached Atmega device.
  @param portName String of the port name to write to
  @param flashFile Array of file(s) to write to the device
  """
  command = [
    "avrdude",
    "-c", programmer,
    "-p", "attiny85",
    "-B", "5",
    "-U" "flash:w:%s:i" % flashFile,
  ]

  s = open('result.log','w')
  e = open('errorresult.log','w')
  result = 1

  signal.signal(signal.SIGALRM, alarm_handler)
  signal.alarm(TIMEOUT)
  try:
    result = subprocess.call(command, stdout=s, stderr=e)
    signal.alarm(0)
  except Alarm:
    pass

  s.close()
  e.close()

  s = open('result.log','r')
  e = open('errorresult.log','r')
  stdout = s.readlines()
  stderr = e.readlines()
  s.close()
  e.close()

  return result, stdout, stderr


if __name__ == '__main__':
  parser = optparse.OptionParser()

  parser.add_option("-p", "--port", dest="portname",
                    help="serial port (ex: /dev/ttyUSB0)", default="/dev/ttyACM0")
  (options, args) = parser.parse_args()

  port=options.portname

  lockFuses = 0x2F
  eFuses    = 0xCB
  hFuses    = 0xD8
  lFuses    = 0xFF
  
  returnCode = writeFuses(port, lockFuses, eFuses, hFuses, lFuses)

 
  if (returnCode[0] != 0):
    print "FAIL. Error writing the fuses!"
    exit(1)
  print "PASS. Fuses written correctly"

  productionFile = "firmware/BlinkyTape-Production.hex"

  returnCode = loadFlash(port, productionFile)

  if (returnCode[0]!= 0):
    print "FAIL. Error programming bootloader!"
    exit(1)
  print "PASS. Bootlaoder programmed successfully"
