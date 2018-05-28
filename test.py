import threading
from queue import Queue
import numpy as np
import PyCmdMessenger as pcm
from time import sleep
from datetime import datetime


log = open("test.log", "w")


def getLog(dta):
    log.write(",".join([str(x) for x in dta])+"\n")
    return None


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


def getEncoderCounts(dta):
    return ("%d, %d, %d\n" % tuple(dta))


def getEncoderStats(dta):
    rtn = ""
    for k, d in enumerate(dta):
        rtn += "Enc %d: " % k
        rtn += ("Neg " if (d & 1) else "Pos ")
        rtn += ("Up " if ((d >> 1) & 1) else "Dn ")
        rtn += ("Pl " if ((d >> 2) & 1) else "")
        rtn += ("" if ((d >> 3) & 1) else "Disabled ")
        rtn += ("Index " if ((d >> 4) & 1) else "")
        rtn += ("Cmp " if ((d >> 5) & 1) else "")
        rtn += ("Und " if ((d >> 6) & 1) else "")
        rtn += ("Ovr " if ((d >> 7) & 1) else "")
        rtn += "\n"
    return rtn


def getPIDCoeff(dta):
    hdr = ["Kp: ", "Ki: ", "Kd: "]
    rtn = ""
    for k, h in enumerate(hdr):
        rtn += h
        rtn += "%0.4e, %0.4e, %0.4e\n" % tuple(dta[k::3])
    return rtn


def getTimes(dta):
    return ("%d, %d, %d\n" % tuple(dta))


cmdsB = [
    ["kRecievePIDLogging", "".join(["fffbbb" for x in range(3)])+"?"],
    ["kSetEncoder", "?"],
    ["kGetEncoder", "?"],
    ["kSetDrive", "?"],
    ["kGetDrive", "?"],
    ["kSetLogging", "?"],
    ["kGetLogging", "?"],
    ["kSetErrorCell", "?"],
    ["kGetErrorCell", "?"],
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
    ["kSetFeedForward", "?"],
    ["kGetFeedForward", "?"],
    ["kSetGlobalPosition", "ff"],
    ["kGetGlobalPosition", "ff"],
    ["kSetPanelOrientation", "ff"],
    ["kGetPanelOrientation", "ff"],
    ["kSetRTC", "L"],
    ["kGetRTC", "L"],
    ["kGetEncoderCounts", "fff"],
    ["kGetEncoderStats", "bbb"],
    ["kGetPIDCoeff", "".join(["fff" for x in range(3)])],
    ["kGetTimes", "LLL"]
]

fnsB = {
    'kRecievePIDLogging': getLog,
    'kGetDrive': getTF,
    'kGetLogging': getTF,
    'kGetErrorCell': getTF,
    'kGetTarget': getTarget,
    'kGetErrorScale': getFloat,
    'kGetEMOCountThreshold': getByte,
    'kGetStabilityCountThreshold': getByte,
    'kGetStabilityThreshold': getFloat,
    'kGetFeedForward': getTF,
    'kGetGlobalPosition': getCoords,
    'kGetPanelOrientation': getCoords,
    'kGetRTC': getDate,
    'kGetEncoder': getTF,
    'kGetEncoderCounts': getEncoderCounts,
    'kGetEncoderStats': getEncoderStats,
    'kGetPIDCoeff': getPIDCoeff,
    'kGetTimes': getTimes
}

ardA = pcm.ArduinoBoard("/dev/ttyACM0", baud_rate=115200)
ardB = pcm.ArduinoBoard("/dev/ttyACM1", baud_rate=115200)

cA = pcm.CmdMessenger(ardA, cmdsA)
cB = pcm.CmdMessenger(ardB, cmdsB)


def sndrcvBool(msgr, msg, inpt):
    if len(inpt) > 1:
        if inpt[1] == "t":
            msgr.send("kSet"+msg, True)
        elif inpt[1] == "f":
            msgr.send("kSet"+msg, False)
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
            sleep(0.01)
            try:
                cmd, dta, tme = cB.receive()
            except ValueError:
                cmds.put("Wrong Number of Arguments\n")
                continue
            except EOFError as err:
                cmds.put(str(err)+"\n")
                continue
            if cmd in fnsB:
                tmp = fnsB[cmd](dta)
                if (tmp is not None):
                    cmds.put(tmp)


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
                sndrcvBool(cB, "Drive", inpt)
            elif inpt[0] == "pidlog":
                sndrcvBool(cB, "Logging", inpt)
            elif inpt[0] == "enc":
                sndrcvBool(cB, "Encoder", inpt)
            elif inpt[0] == "errcell":
                sndrcvBool(cB, "ErrorCell", inpt)
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
                sndrcvBool(cB, "FeedForward", inpt)
            elif inpt[0] == "gposition":
                sndrcvFloats(cB, "GlobalPosition", inpt, 2)
            elif inpt[0] == "orientation":
                sndrcvFloats(cB, "PanelOrientation", inpt, 2)
            elif inpt[0] == "rtc":
                if len(inpt) > 1:
                    cB.send("kSetRTC", int(datetime.now().timestamp()))
                else:
                    cB.send("kGetRTC")
            elif inpt[0] == "encc":
                cB.send("kGetEncoderCounts")
            elif inpt[0] == "encs":
                cB.send("kGetEncoderStats")
            elif inpt[0] == "cnt":
                cB.send("kSheetCenter")
            elif inpt[0] == "tms":
                cB.send("kGetTimes")
            elif inpt[0] == "pidcoeff":
                cB.send("kGetPIDCoeff")
            elif inpt[0] == 'quit':
                break
        sleep(0.1)
        while not cmds.empty():
            print(cmds.get())
    log.close()


commands = Queue()

reciever = threading.Thread(target=recieve, args=(commands,),
                            daemon=True)
displayer = threading.Thread(target=display, args=(commands,),
                             daemon=False)

if __name__ == "__main__":
    reciever.start()
    displayer.start()
