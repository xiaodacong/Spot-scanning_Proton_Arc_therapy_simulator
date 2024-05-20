import numpy as np
# Add path to module
import sys
import csv
sys.path.append('H:\onedrive\controller design\CalculateSegmentVelocity')

ID = '2211904'
filename = 'Sorted_ControlPoint_P1_2211904_2.5deg_delete spot.csv'
#######################
tolDeliveryTimeConstantSpeed = 0

myPlanDir = ['H:\\OneDrive\\ControlDesign\\PlanCSV\\P1\\',str(ID),'\\']
# myPlanDir = ['D:\OneDrive\ControlDesign\PlanCSV\',machineModel,'\',str(ID),'\\']

machineModel = 'P1'  # 'PPlus';%% PPlus

tol = 1.0  # degree % if tolerance is too large the delivery window is nagtive the code is equil to variable tol.
method = 1  # dynamic: 0, step: 1

tolModel = 1  # without tol:0,  with tol: 1

# bufferModel = 1  # Linear
# bufferModel = 2  # MinVelocity
bufferModel = 3  # Polynominal

# myPlanFilename = myPlanDir + filename

myPlanFilename = 'ControllerInput_P1.csv'

# data = np.genfromtxt(myPlanFilename, delimiter=',', skip_header=2)
data = np.genfromtxt(myPlanFilename, delimiter=',', skip_header=1)
#print(data)
TableAngArray = data[:, 0]
AngleArray = data[:, 1]
ActAngleArray = data[:, 2]
EnergyArray = data[:, 3]
Model_LST = data[:, 4]
Model_BST = data[:, 5]
Model_SST = data[:, 6]
Model_SSP = data[:, 7]
dataShape = np.shape(data)
if dataShape[1] == 10:
    Model_LSTInBeam = data[:, 9]
else:
    Model_LSTInBeam = np.zeros((1, dataShape[0]))
# Constants
AngGapIni = 4
maxAcc = 0.6
ascendELST = 5
gantryAnglePrecision = 0.05

# Initialize arrays
uniActAngleArray = np.unique(ActAngleArray)
uniTableAngArray = np.zeros_like(uniActAngleArray)
uniEnergyArray = np.zeros_like(uniActAngleArray)
uniModel_LST = np.zeros_like(uniActAngleArray)
uniModel_BST = np.zeros_like(uniActAngleArray)
uniModel_SST = np.zeros_like(uniActAngleArray)
uniModel_SSP = np.zeros_like(uniActAngleArray)
uniModel_BeamSwitch = np.zeros_like(uniActAngleArray)
uniModel_LSTInBeam = np.zeros_like(uniActAngleArray)
uniSpotDT = np.zeros_like(uniActAngleArray)
ascendArray = np.zeros_like(uniActAngleArray)

# Open file
titleName = ['uniTableAngArray', 'uniActAngleArray', 'uniEnergyArray', 'uniLayerSwitchTime(s)', 'uniBurstSwitchTime(s)', 'uniSpotSwitchTime(s)', 'uniSpotSpillTime(s)']
#myPlanFilename = myPlanDir + 'unique' + filename
fid = open(myPlanFilename+'.new', 'w')
fid.write(','.join(titleName) + '\n')

# Loop over unique angles
for i in range(len(uniActAngleArray)):
    angIndex = np.where(ActAngleArray == uniActAngleArray[i])[0]
    #print(ActAngleArray)
    #print("###########")
    #print(uniActAngleArray)
    uniTableAngArray[i] = TableAngArray[angIndex[0]]
    uniEnergyArray[i] = EnergyArray[angIndex[0]]
    uniModel_LST[i] = Model_LST[angIndex[0]]
    uniModel_BST[i] = np.sum(Model_BST[angIndex])
    uniModel_SST[i] = np.sum(Model_SST[angIndex])
    uniModel_SSP[i] = np.sum(Model_SSP[angIndex])
    if Model_LST[angIndex[0]] >= ascendELST:
        ascendArray[i] = 1

    # Calculate spot delivery time
    if len(angIndex) == 1:
        uniSpotDT[i] = uniModel_BST[i] + uniModel_SST[i] + uniModel_SSP[i] + uniModel_LSTInBeam[i]
    else:
        uniSpotDT[i] = np.sum(Model_LST[angIndex[1:]]) + uniModel_BST[i] + uniModel_SST[i] + uniModel_SSP[i]

    # Write data to file
    data = [uniTableAngArray[i], uniActAngleArray[i], uniEnergyArray[i], uniModel_LST[i], uniModel_BST[i], uniModel_SST[i], uniModel_SSP[i]]
    fid.write(','.join(str(x) for x in data) + '\n')

# Close file
fid.close()

# Initialize variables
actVelocitySequence = []
timeSequence = []
angleSequence = []
beamONSequence = []
accSequence = []
Vmax = []
beamONStartSequenceTemp = []
beamONEndSequenceTemp = []
TotTime = 0
TotAng = 0
ascendIndex = np.where(ascendArray == 1)[0]
angleSampling = np.diff(uniActAngleArray)
angleSampling = np.insert(angleSampling, 0, AngGapIni)
toleArrayTemp = tol * np.ones_like(angleSampling)
ELST = uniModel_LST
predictDeliverTime = uniSpotDT
startVelocity = 0
############################
if bufferModel == 1:
    # linear relative to predicted time
    # %%relative minRatio:0.5  maxRatio :1
    reMax = 1
    reMin = 0.1
    maxDeliveryTime = max(predictDeliverTime)
    minDeliveryTime = min(predictDeliverTime)
    ratioTolArray = (reMax - reMin) / (maxDeliveryTime - minDeliveryTime) * (predictDeliverTime - minDeliveryTime) + reMin
    # ratioTolArray = -(reMax -reMin)/(maxDeliveryTime -minDeliveryTime )*(predictDeliverTime- minDeliveryTime ) + reMax;%minDeliveryTime
    toleArray = toleArrayTemp * ratioTolArray
    bufferArray = toleArrayTemp * (1 - ratioTolArray)
    bufferName = 'Linear'
    
elif bufferModel == 2:
    # using min predicted velocity to generate constant velocity
    # % add buffer to achieve constant speed
    maxDeliveryTime = max(predictDeliverTime)
    minDeliveryTime = min(predictDeliverTime)
    minV = 2 * tol / maxDeliveryTime
    vArrayTest = 2 * toleArrayTemp / predictDeliverTime
    minDeliveryWindowArray = minV * predictDeliverTime
    bufferArray = (2 * toleArrayTemp - minDeliveryWindowArray) / 2
    toleArray = minDeliveryWindowArray / 2
    bufferName = 'MinVelocity'
    
elif bufferModel == 3:
    # polynominal
    # %% polynominal fit delivery time vs delivery window
    if 0: # backup
        predictDeliverTimeTemp = predictDeliverTime
        predictDeliverTimeTemp[predictDeliverTime > 5.0] = 5.0  # The delivery window is 1.9224 when delivery time =5.0 if delivery time >5 delivery windown will decrease even to <0
        deliveryWindowArrayTemp = -0.0058 * predictDeliverTimeTemp ** 4 + 0.0953 * predictDeliverTimeTemp ** 3 - 0.5997 * predictDeliverTimeTemp ** 2 + 1.7533 * predictDeliverTimeTemp - 0.1373
        deliveryWindowArray = deliveryWindowArrayTemp
        toleArray = deliveryWindowArray / 2
        # bufferArray = (toleArrayTemp - deliveryWindowArray)/2;
        bufferArray = (toleArrayTemp * 2 - deliveryWindowArray) / 2
        bufferName = 'Poly'  # 'Polynominal'
    predictDeliverTimeTemp = predictDeliverTime
    predictDeliverTimeTemp[predictDeliverTime > 5.0] = 5.0  # The delivery window is 1.9224 when delivery time =5.0 if delivery time >5 delivery windown will decrease even to <0
    deliveryWindowArrayTemp = -0.0058 * predictDeliverTimeTemp ** 4 + 0.0953 * predictDeliverTimeTemp ** 3 - 0.5997 * predictDeliverTimeTemp ** 2 + 1.7533 * predictDeliverTimeTemp - 0.1373
    deliveryWindowArray = deliveryWindowArrayTemp * (tol * 2) / 2
    toleArray = deliveryWindowArray / 2
    # bufferArray = (toleArrayTemp - deliveryWindowArray)/2;
    bufferArray = (toleArrayTemp * 2 - deliveryWindowArray) / 2
    bufferName = 'Poly'
else:
    print('Select correct buffer model!')

##############

#if method == 1:
#    actVelocitySequence, timeSequence, beamONSequence = calculateSegmentStepDeliverVelocitySequenceWithTolAndBuffer(ELST, predictDeliverTime, angleSampling, startVelocity, gantryAnglePrecision, maxAcc, toleArray, bufferArray)
#    deliver_model = 'Step'
#else:
#    actVelocitySequence, timeSequence, beamONSequence = calculateSegmentDynamicDeliverVelocitySequenceWithTol(ELST, predictDeliverTime, angleSampling, startVelocity, gantryAnglePrecision, maxAcc, tol)
#    deliver_model = 'Dynamic'
actVelocitySequence = []
timeSequence = []
beamONSequence = []
deliver_model = 'Step'

with open('data.csv', newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    next(reader)
    next(reader)
    for row in reader:
        actVelocitySequence.append(float(row[0]))
        timeSequence.append(float(row[1]))
        if (float(row[2])>0):
            beamONSequence.append(float(row[2]))
# adjustAngArray

beamONStartSequenceTemp = []
beamONEndSequenceTemp = []

for k in range(0, len(beamONSequence)//2):
    beamONStartSequenceTemp.append(beamONSequence[2*k])
    beamONEndSequenceTemp.append(beamONSequence[2*k + 1])

tolDeliveryTimeConstantSpeed = 0

for k in range(len(beamONEndSequenceTemp)):
    #print(timeSequence)
    #print("#######")
    #print(beamONEndSequenceTemp)
    endIndex = timeSequence.index(beamONEndSequenceTemp[k])
    endVelocity = actVelocitySequence[endIndex]
    
    # startIndex = actVelocitySequence.index(endVelocity)
    startIndex = max([i for i in range(0, endIndex) if actVelocitySequence[i] == endVelocity] or [-1])
    
    if startIndex == -1:
        deliveryTimeConstantSpeed = 0
    else:
        deliveryTimeConstantSpeed = timeSequence[endIndex] - timeSequence[startIndex]
    
    tolDeliveryTimeConstantSpeed += deliveryTimeConstantSpeed

print(tolDeliveryTimeConstantSpeed)
###############
# calculate beam ON and energy switch
beamONStartSequence = np.zeros(len(Model_LST))
beamONEndSequence = np.zeros(len(Model_LST))
ELSTStartSequence = np.zeros(len(Model_LST))
ELSTEndSequence = np.zeros(len(Model_LST))
ELSTStartEnergy = np.zeros(len(Model_LST))
ELSTEndEnergy = np.zeros(len(Model_LST))

num = 0
for l in range(len(uniActAngleArray)):
    angIndex = np.where(ActAngleArray == uniActAngleArray[l])[0]
    for m in range(len(angIndex)):
        if m == 0:
            beamONStartSequence[num] = beamONStartSequenceTemp[l]
            beamONEndSequence[num] = beamONStartSequenceTemp[l] + Model_BST[num] + Model_SST[num] + Model_SSP[num]
        else:
            beamONStartSequence[num] = beamONEndSequence[num-1] + Model_LST[num]
            beamONEndSequence[num] = beamONStartSequence[num] + Model_BST[num] + Model_SST[num] + Model_SSP[num]
        num += 1
        
for i in range(len(Model_LST)):
    if i == 0:
        ELSTStartSequence[i] = 0
        ELSTStartEnergy[i] = 0
    else:
        ELSTStartSequence[i] = beamONStartSequence[i] - Model_LST[i]
        ELSTStartEnergy[i] = EnergyArray[i]
        
    ELSTEndSequence[i] = beamONStartSequence[i]
    ELSTEndEnergy[i] = EnergyArray[i]
##############
# convert ELST within control point to beam on
beamONStartSequence = beamONStartSequenceTemp
beamONEndSequence = beamONEndSequenceTemp

t_end = abs(actVelocitySequence[-1])/maxAcc
t_end = timeSequence[-1] + t_end
actVelocitySequence = [0] + actVelocitySequence + [0]
timeSequence = [0] + timeSequence + [t_end]

Tot_static = sum(Model_LST) + sum(Model_BST) + sum(Model_SST) + sum(Model_SSP)
Tot_rotation = timeSequence[-1]

import matplotlib.pyplot as plt

plt.plot(timeSequence, actVelocitySequence)
plt.grid(True)
plt.xlabel('time(s)')
plt.ylabel('velocity(degree/s)')
plt.title('actV')
plt.legend(['actV'])
plt.show()

# convert sequence to discrete sequence according to discrete time
deltaT = 0.1
t = np.arange(0, timeSequence[-1] + 1 * deltaT, deltaT)
actVList = np.zeros_like(t)
angList = np.zeros_like(t)
TotAng = 0

####################

# Convert time and velocity sequences to unique and sorted arrays

timeSequenceTemp, ia, ic = np.unique(timeSequence, return_index=True, return_inverse=True)
print(timeSequenceTemp)
print(np.asarray(ia,dtype=int))
print('########')
print(ic)
actVelocitySequenceTemp=[]
for i in ia:
    actVelocitySequenceTemp.append( actVelocitySequence[i])

# Interpolate velocity sequence to discrete time sequence
actVList = np.interp(t, timeSequenceTemp, actVelocitySequenceTemp)
actVList[np.isnan(actVList)] = 0

# Calculate angular position at each time step
TotAng = 0
angList = np.zeros_like(t)
for k in range(len(t)):
    TotAng += actVList[k] * deltaT
    angList[k] = TotAng

# Add offset to angular position
angList = angList + ActAngleArray[0] - AngGapIni + tol

# Initialize beam-on and ELST lists
actBeamONList = np.zeros_like(t)
actELSTList = np.zeros_like(t)

# Calculate beam-on and ELST periods
actDeliverWindow = np.zeros(len(beamONStartSequence))
for n in range(len(beamONStartSequence)):
    beamONStartIndex = round(beamONStartSequence[n] / deltaT) + 1
    beamONEndIndex = round(beamONEndSequence[n] / deltaT) + 1
    
    beamONStartAng = angList[beamONStartIndex]
    beamONEndAng = angList[beamONEndIndex]
    actDeliverWindow[n] = beamONEndAng - beamONStartAng
    
    actBeamONList[beamONStartIndex:beamONEndIndex] = 10
    
    EnergyStartIndex = round(ELSTStartSequence[n] / deltaT)
    if EnergyStartIndex == 0:
        EnergyStartIndex = 1
        
    EnergyEndIndex = round(ELSTEndSequence[n] / deltaT)
    actELSTList[EnergyStartIndex:EnergyEndIndex] = np.interp(t[EnergyStartIndex:EnergyEndIndex], 
                                                             [ELSTStartSequence[n], ELSTEndSequence[n]], 
                                                             [ELSTStartEnergy[n], ELSTEndEnergy[n]])
###################
# set NaN values to 0 in actELSTList
actELSTList[np.isnan(actELSTList)] = 0

expectDeliverWindowRation = tol * 2 * len(actDeliverWindow) / (ActAngleArray[-1] - ActAngleArray[0])
actDeliverWindowRatio = np.sum(actDeliverWindow) / (ActAngleArray[-1] - ActAngleArray[0])

ascendAngList = []
ascendTimeList = []
descendAngList = []
descendTimeList = []

for k in range(len(ELSTEndSequence)):
    flagIndex = round(ELSTEndSequence[k] / deltaT) + 1
    flagIndex = round(flagIndex - Model_LST[k] / deltaT)
    if flagIndex == 0:
        flagIndex = 1

    if Model_LST[k] >= ascendELST:
        ascendAngList.append(angList[flagIndex])
        ascendTimeList.append(t[flagIndex])
    else:
        descendAngList.append(angList[flagIndex])
        descendTimeList.append(t[flagIndex])

ascendAngList = ascendAngList[1:]
ascendTimeList = ascendTimeList[1:]

###############

import matplotlib.pyplot as plt
import os

t1 = t.reshape(-1, 1)
angList1 = angList.reshape(-1, 1)
actBeamONList1 = actBeamONList.reshape(-1, 1)

controllerOutput = deliver_model + filename[:-10] + bufferName + 'Buffer' + 'IniAng' + str(AngGapIni) + ' static=' + str(round(Tot_static)) + ' rot=' + str(round(Tot_rotation))
fig, ax1 = plt.subplots()
ax1.plot(t, actVList)
ax1.set_xlabel('Time(s)', fontsize=16)
ax1.set_ylabel('Gantry Velocity(degree/s)', fontsize=16)
ax1.tick_params(axis='y')
ax2 = ax1.twinx()
ax2.plot(t, angList)
ax2.set_ylabel('Gantry Angle(degree)', fontsize=16)
ax2.tick_params(axis='y')
plt.title(controllerOutput, fontsize=16)
ax1.grid(True)
ax1.plot(t, actBeamONList, 'g-')
ax1.plot(ascendTimeList, ascendAngList, 'k+')
ax1.plot(descendTimeList, descendAngList, '.')
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.gcf().set_size_inches(12, 8)
ax1.legend(['Gantry Velocity', 'Gantry Angle', 'Beam ON', 'Energy Switch Ascending', 'Energy Switch Descending'], loc='upper left')
plt.tight_layout()
plt.show()
outputPath = './'#os.path.join('H:', os.sep, 'OneDrive', 'ControlDesign', 'ControllerResult', machineModel, ID)
if not os.path.exists(outputPath):
    os.makedirs(outputPath)

plt.savefig(os.path.join(outputPath, controllerOutput+'.tiff'))
plt.savefig(os.path.join(outputPath, controllerOutput+'.jpg'))

if 1:
    ## write controller output to csv file
    tOut = t.T
    angListOut = angList.T
    beamONOut = actBeamONList.T
    titleName = ['Time(s)', 'GantryAngle(degree)', 'BeamON']
    fileName = outputPath + controllerOutput + 'Controller.csv'
    print(beamONOut)
    with open(fileName, mode='w',newline='') as file:
        writer = csv.writer(file)
        writer.writerow(titleName)
        for i in range(len(tOut)):
            beamONOut[i]=161.0+beamONOut[i]
            writer.writerow([tOut[i], angListOut[i], (beamONOut[i])])

uniTimeSequence, ia, ic = np.unique(timeSequence, return_index=True, return_inverse=True)
#uniActVelocitySequnece = actVelocitySequence[ia]
uniActVelocitySequnece=[]
for i in ia:
    actVelocitySequenceTemp.append( actVelocitySequence[i])
uniTimeSequenceTemp = uniTimeSequence
uniActVelocitySequneceTemp = uniActVelocitySequnece

meanV = np.mean(actVList)
stdV = np.std(actVList)
TV = sum(abs(np.diff(actVList)))
print('The first ELST is ')
print(ELST[0])