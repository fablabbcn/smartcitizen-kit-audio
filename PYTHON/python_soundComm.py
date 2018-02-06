import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import sounddevice as sd
import sys, glob, serial

def serial_ports():
	"""Lists serial ports

	:raises EnvironmentError:
		On unsupported or unknown platforms
	:returns:
		A list of available serial ports
	"""
	if sys.platform.startswith('win'):
		ports = ['COM' + str(i + 1) for i in range(256)]

	elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
		# this is to exclude your current terminal "/dev/tty"
		ports = glob.glob('/dev/tty[A-Za-z]*')

	elif sys.platform.startswith('darwin'):
		ports = glob.glob('/dev/tty.*')

	else:
		raise EnvironmentError('Unsupported platform')

	result = []

	for port in ports:
		try:
			s = serial.Serial(port)
			s.close()
			result.append(port)
		except (OSError, serial.SerialException):
			pass
	return result

def getSerialData():
	SerialData = np.zeros()

	index = -1 
	if ser.inWaiting():
		index = index + 1
		bufferSerial = ser.readline().decode('utf-8')[:-1]
		try:
			bS =  bufferSerial.strip('\t\n\r').split('\t')
			SerialData[index]=float(bS)	
		except:
			pass
	## close the serial connection
	return SerialData

# Retrieve serial ports and pick the usb 
ports = serial_ports()
# ser = serial.Serial(ports[1], 115200)
# print ser

# Global parameters
fs = 44100       # sampling rate, Hz, must be integer
carrierFreq = np.array([3000,4100,5200,6300,7400,8500,9600,10700])
carrierFreqNum = carrierFreq.shape[0];
durationInterCode = 3 #Seconds between each code sent
interCode = np.zeros(fs*durationInterCode)

# Pulses
numberPulses = 16		# total number of pulses
# Create random words with carrierFreqNum
# initializer = np.random.rand(carrierFreqNum,numberPulses) #Random case
initializer = np.ones((carrierFreqNum,numberPulses)) #Worst case, all active
word = (initializer>0.5)

# Define Duration variations
durationPulseVector = np.array([0.3, 0.5, 0.7, 1, 1.5])
interPulseDurationVector = np.array([0, 0.1, 0.2, 0.5])

# Iterate over them
for n in range(durationPulseVector.shape[0]):
	
	durationPulse = durationPulseVector[n] # total duration of the pulse
	
	for m in range(interPulseDurationVector.shape[0]):
		code = np.zeros(numberPulses)
		interPulseDuration = interPulseDurationVector[m]
		interPulse = (interPulseDuration>0)
		duration = int(fs*(durationPulse*numberPulses+interPulseDuration*(numberPulses-1)))
		timeVector = np.arange(fs*durationPulse)
		print "Total pulse duration: " + str(duration)
		print "Interpulse?: "+ str(interPulse)

		samples = np.zeros(duration)

		lastSample = 0
		pulse = False

		# generate samples, note conversion to float32 array
		for j in range (numberPulses):
			activeFreqs = word[:,j]
			# print activeFreqs
			for i in range(carrierFreqNum):
				if activeFreqs[i]:
					samples[lastSample:lastSample+timeVector.shape[0]] += activeFreqs[i]*(np.sin(2*np.pi*timeVector*carrierFreq[i]/fs))
					pulse = True
				# Convert word to code
				if word[i,j]:
					code[j] += int(np.power(2,i))
			
			if pulse:
				# divide by max to normalize
				samples[lastSample:lastSample+int(fs*durationPulse)] /= np.max(samples[lastSample:lastSample+int(fs*durationPulse)]) 
				lastSample += int(timeVector.shape[0] + fs*interPulseDuration)
				pulse = False

		print "Code Sent:"
		print code 

		# Play it
		sd.play(samples, fs)
		status = sd.wait()
		# codeReceived = getSerialData()
		if status:
		    print('Error during playback: ' + str(status))
		sd.play(interCode, fs)
		status = sd.wait()
		if status:
		    print('Error during playback: ' + str(status))

# plot the samples
# fig = plt.figure()
# plt.plot(samples)
# # plt.axis([0, 1000, -1.5, 1.5])
# plt.title("Chunk of data")
# plt.xlabel("Sample number")
# plt.ylabel("RMS (dB)")
# plt.show()
