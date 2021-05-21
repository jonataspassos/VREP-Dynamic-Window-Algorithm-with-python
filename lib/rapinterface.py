# simRemoteApi.start(19999)
from . import sim

import time

res, objs = None, None

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

def start(init,loop,finnaly, t = 60):
    try:
        if clientID!=-1:
            print ('Connected to remote API server')

            # Now try to retrieve data in a blocking fashion (i.e. a service call):
            res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
            if res==sim.simx_return_ok:
                print ('Number of objects in the scene: ',len(objs))
            else:
                print ('Remote API function call returned with error code: ',res)
            print("press CTRL + C to stop...")
            time.sleep(2)

            # Now send some data to CoppeliaSim in a non-blocking fashion:
            printr('Hello CoppeliaSim!')

            init()

            startTime=time.time()
            sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
            while time.time()-startTime < t and not loop():
                pass

            finnaly()
            # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
            sim.simxGetPingTime(clientID)

            # Now close the connection to CoppeliaSim:
            sim.simxFinish(clientID)
        else:
            print ('Failed connecting to remote API server')
        print ('Program ended')
    except KeyboardInterrupt as identifier:
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
        print ('Program stoped by user')
        

def printr (s):
    sim.simxAddStatusbarMessage(clientID,s,sim.simx_opmode_oneshot)

def simple_distance(sensor):
    # distancia euclidiana entre p1 e p2
    _0,detected,p,_2,_3 = sim.simxReadProximitySensor(clientID,sensor,sim.simx_opmode_streaming)
    return detected,(p[0]**2 + p[1]**2 + p[2]**(1/2))

def getobject(name):
    res,handle = sim.simxGetObjectHandle(clientID,name,sim.simx_opmode_blocking)
    return handle

def getcollection(name):
    res, collection = sim.simxGetCollectionHandle(clientID,name,sim.simx_opmode_blocking)
    return collection

def getposition(name, ref):
    _, position = sim.simxGetObjectPosition(clientID,name,ref,sim.simx_opmode_streaming)
    return position

def getpositionb(name, ref):
    _, position = sim.simxGetObjectPosition(clientID,name,ref,sim.simx_opmode_blocking)
    return position

def getorientation(name, ref):
    _, orientation = sim.simxGetObjectOrientation(clientID,name,ref,sim.simx_opmode_streaming)
    return orientation

def getorientationb(name, ref):
    _, orientation = sim.simxGetObjectOrientation(clientID,name,ref,sim.simx_opmode_blocking)
    return orientation

def setvelocity(name, vel):
    sim.simxSetJointTargetVelocity(clientID,name, vel,sim.simx_opmode_streaming)

def getDistanceCollection():
    _,handles,_2,r,_4= sim.simxGetObjectGroupData(clientID,sim.sim_object_proximitysensor_type,13,sim.simx_opmode_streaming)
    return _,handles,_2,r,_4


def sleep(t):
    time.sleep(t)