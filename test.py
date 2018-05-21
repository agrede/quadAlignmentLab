import threading
from queue import Queue
import numpy as np
import PyCmdMessenger as pcm
from time import sleep
from datetime import datetime


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

fnsA = {
    'kGetValues': getValues,
    'kGetDeltas': getDeltas,
    'kGetOffsets': getOffsets}


def getTF(dta):
    if dta[0]:
        return "True\n"
    else:
        return "False\n"


def getTarget(dta):
    return ("Target: %0.4e, %0.4e, %0.4e\n" % tuple(dta))


def getFloat(dta):
    return ("%0.4e\n" % dta[0])


def getByte(dta):
    return ("%d\n" % dta[0])


def getCoords(dta):
    return ("%0.4e, %0.4e\n" % tuple(dta))


def getDate(dta):
    tme = datetime.fromtimestamp(dta[0])
    return (tme.isoformat()+"\n")


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

fnsB = {
    'kGetPIDEnabled': getTF,
    'kGetPIDLoggingEnabled': getTF,
    'kGetErrorCellEnabled': getTF,
    'kGetTarget': getTarget,
    'kGetErrorScale': getFloat,
    'kGetEMOCountThreshold': getByte,
    'kGetStabilityCountThreshold': getByte,
    'kGetStabilityThreshold': getFloat,
    'kGetFeedForwardEnabled': getTF,
    'kGetGlobalPosition': getCoords,
    'kGetPanelOrientation': getCoords,
    'kGetRTC': getDate
}

ardA = pcm.ArduinoBoard("/dev/ttyACM0", baud_rate=115200)
ardB = pcm.ArduinoBoard("/dev/ttyACM1", baud_rate=115200)

cA = pcm.CmdMessenger(ardA, cmdsA)
cB = pcm.CmdMessenger(ardB, cmdsB)


def sndrcvBool(msgr, msg, inpt):
    if len(inpt) > 1:
        msgr.send("kSet"+msg, bool(inpt[2]))
    else:
        msgr.send("kGet"+msg)


def sndrcvFloats(msgr, msg, inpt, N=1):
    if len(inpt) == 1+N:
        msgr.send("kSet"+msg, *[float(x) for x in inpt[1:]])
    else:
        msgr.send("kGet"+msg)


def sndrcvByte(msgr, msg, inpt, N=1):
    if len(inpt) == 1+N:
        msgr.send("kSet"+msg, *[int(x) for x in inpt[1:]])
    else:
        msgr.send("kGet"+msg)


def recieve(cmds):
    while(True):
        if (ardA.comm.in_waiting):
            try:
                cmd, dta, tme = cA.receive()
            except ValueError:
                cmds.put("Wrong Number of Arguments\n")
                continue
            except EOFError as err:
                cmds.put(str(err)+"\n")
                continue
            if cmd in fnsA:
                cmds.put(fnsA[cmd](dta))
        if (ardB.comm.in_waiting):
            try:
                cmd, dta, tme = cB.receive()
            except ValueError:
                cmds.put("Wrong Number of Arguments\n")
                continue
            except EOFError as err:
                cmds.put(str(err)+"\n")
                continue
            if cmd in fnsB:
                cmds.put(fnsB[cmd](dta))


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
            elif inpt[0] == "pid":
                sndrcvBool(cB, "PIDEnabled", inpt)
            elif inpt[0] == "pidlog":
                sndrcvBool(cB, "PIDLoggingEnabled", inpt)
            elif inpt[0] == "errcell":
                sndrcvBool(cB, "ErrorCellEnabled", inpt)
            elif inpt[0] == "target":
                sndrcvFloats(cB, "Target", inpt, 3)
            elif inpt[0] == "errscale":
                sndrcvFloats(cB, "ErrorScale", inpt)
            elif inpt[0] == "emocnt":
                sndrcvByte(cB, "EMOCountThreshold", inpt)
            elif inpt[0] == "stbcnt":
                sndrcvByte(cB, "StabilityCountThreshold", inpt)
            elif inpt[0] == "stbthr":
                sndrcvFloats(cB, "StabilityThreshold", inpt)
            elif inpt[0] == "ff":
                sndrcvBool(cB, "FeedForwardEnabled", inpt)
            elif inpt[0] == "gposition":
                sndrcvFloats(cB, "GlobalPosition", inpt, 2)
            elif inpt[0] == "orientation":
                sndrcvFloats(cB, "PanelOrientation", inpt, 2)
            elif inpt[0] == "rtc":
                if len(inpt) > 1:
                    cB.send("kSetRTC", int(datetime.now().timestamp()))
                else:
                    cB.send("kGetRTC")
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
