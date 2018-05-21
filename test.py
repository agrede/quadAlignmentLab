import threading
from queue import Queue
import numpy as np
import PyCmdMessenger as pcm
from time import sleep


def getValues(dta):
    tmp = np.array(dta).reshape(4, 4)
    rtn = ""
    for k in [0, 1, 3]:
        rtn += (
            "Values %d: %0.4e, %0.4e, %0.4e, %0.4e\n" %
            tuple(np.hstack(([k], tmp[k, :]))))
    return rtn


def getDeltas(dta):
    tmp = np.array(dta).reshape(4, 2)
    rtn = ""
    for k in [0, 1, 3]:
        rtn += ("Deltas %d: %0.4e, %0.4e\n" %
                tuple(np.hstack(([k], tmp[k, :]))))
    return rtn


def getOffsets(dta):
    return ("Offsets: %0.4e, %0.4e, %0.4e\n" % tuple(dta))


cmdsA = [
    ["kGetValues", "".join(["f" for x in range(16)])],
    ["kGetDeltas", "".join(["f" for x in range(8)])],
    ["kGetOffsets", "fff"]]
cmdsB = [
    ["kRecievePIDLogging", "".join(["llfb" for x in range(4)])],
    ["kGetPIDEnabled", "?"],
    ["kSetPIDEnabled", "?"],
    ["kSetPIDLoggingEnabled", "?"],
    ["kGetPIDLoggingEnabled", "?"],
    ["kGetErrorCellEnabled", "?"],
    ["kSetErrorCellEnabled", "?"],
    ["kSheetCenter", ""],
    ["kSetTarget", "fff"],
    ["kGetTarget", "fff"],
    ["kSetErrorScale", "f"],
    ["kGetErrorScale", "f"],
    ["kSetEMOCountThreshold", "b"],
    ["kGetEMOCountThreshold", "b"],
    ["kSetStabilityCountThreshold", "b"],
    ["kGetStabilityCountThreshold", "b"],
    ["kSetStabilityThreshold", "f"],
    ["kGetStabilityThreshold", "f"],
    ["kSetFeedForwardEnabled", "?"],
    ["kGetFeedForwardEnabled", "?"],
    ["kSetGlobalPosition", "ff"],
    ["kGetGlobalPosition", "ff"],
    ["kSetPanelOrientation", "ff"],
    ["kGetPanelOrientation", "ff"],
    ["kSetRTC", "L"],
    ["kGetRTC", "L"]
]
fnsA = {
    'kGetValues': getValues,
    'kGetDeltas': getDeltas,
    'kGetOffsets': getOffsets}

ardA = pcm.ArduinoBoard("/dev/ttyACM0", baud_rate=9600)
# ardB = pcm.ArduinoBoard("/dev/ttyACM1", baud_rate=115200)

cA = pcm.CmdMessenger(ardA, cmdsA)
# cB = pcm.CmdMessenger(ardB, cmdsB)


def recieve(cmds):
    while(True):
        if (ardA.comm.in_waiting):
            try:
                cmd, dta, tme = cA.receive()
            except ValueError:
                cmds.put("Wrong Number of Arguments\n")
                continue
            if cmd in fnsA:
                cmds.put(fnsA[cmd](dta))
        # if (ardA.comm.in_waiting):
        #     cmd, dta, tme = cA.recieve()
        #     if cmd in fnsA:
        #         fnsB[cmd](dta)


def display(cmds):
    while(True):
        inpt = input("tracking>").split()
        if len(inpt) > 0:
            if inpt[0] == 'values':
                cA.send("kGetValues")
            elif inpt[0] == 'deltas':
                cA.send("kGetDeltas")
            elif inpt[0] == 'offsets':
                cA.send("kGetOffsets")
            elif inpt[0] == 'quit':
                break
        sleep(0.1)
        while not cmds.empty():
            print(cmds.get())


commands = Queue()

reciever = threading.Thread(target=recieve, args=(commands,),
                            daemon=True)
displayer = threading.Thread(target=display, args=(commands,),
                             daemon=False)

if __name__ == "__main__":
    reciever.start()
    displayer.start()
