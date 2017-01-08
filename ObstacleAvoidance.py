#Import Libraries:

#interfacing v-rep environment

import vrep                
import sys
import time                
import numpy as np         
import math
import matplotlib as mpl   

#Initial assignment

PI=math.pi  #pi=3.14..., constant

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print ('Connected to remote API server')
    
else:
    print ('Connection not successful')
    sys.exit('Could not connect')

#CMAC training

y = np.zeros(0)
weight_vector=y*0
print (weight_vector)
weight_vector=np.append([0],weight_vector)
weight_vector=np.append(weight_vector,[0])

before=current=after=None
l = len(weight_vector)

#retrieve motor  handles
errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)


sensor_h=[] #empty list for handles
sensor_val=np.array([]) #empty array for sensor measurements

array_resolution=np.array([])
array_image=np.array([])

#orientation of all the sensors: 
sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

#for loop to retrieve sensor arrays and initiate sensors
for x in range(1,16+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x),vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values
        

t = time.time()


while (time.time()-t)<60:
    #Loop Execution
    sensor_val=np.array([])    
    for x in range(1,16+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h[x-1],vrep.simx_opmode_buffer)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values
        
        #returnCode,array_resolution,array_image=vrep.simxGetVisionSensorImage(clientID,'Pioneer_p3dx_Vision_sensor',0,vrep.simx_opmode_oneshot)
    
    #CMAC algorithm for training the sensor data
    sensor_cu=sensor_val[0:8]*sensor_val[0:8] #square the values of front-facing sensors 1-8
    
    y_yield=None
    z=None
    m=2*x
    n=2*x+1
    o=2*x+2
    if m<2:
        if n<2:
            if o<2:
                if z<10:
				
                    before = weight_vector[(m)]*0.8
                    current = weight_vector[n]
                    after = weight_vector[o]*0.2
                    
                    y_yield=before+current+after
                    print ('y_yield=',y_yield)
                    
                    y_error=sensor_cu[m]-y_yield
                    print ('y_value[',i,']=',y[i])
                    y_error_corrected=(y_error/3)
                    print ('y_error_corrected=',y_error_corrected)
                    weight_vector[m]=(y_error_corrected)+(weight_vector[m])
                    weight_vector[n]=y_error_corrected+(weight_vector[n])
                    weight_vector[o]=(y_error_corrected)+(weight_vector[o])
                    print (weight_vector)
                    count[x]=x+1
                    s[x]=y_error
                    z=z+1
                    sensor_cu=y_yield 
                    # plt.plot(sensor_cu, s[x])					
                # if z>10:
    
				
    min_ind=np.where(sensor_cu==np.min(sensor_cu))
    min_ind=min_ind[0][0]
    
    if sensor_cu[min_ind]<0.5:
        steer=-1/sensor_loc[min_ind]
    else:
        steer=0
   
    #keyboard control - forward motion   
    nb = input('')
    number = int(nb)
    if(number==1):
	 
        v=1	#forward velocity
        kp=0.5	#steering gain
        vl=v+kp*steer
        vr=v-kp*steer
        print ("V_l =",vl)
        print ("V_r =",vr)
        
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,vl, vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,vr, vrep.simx_opmode_streaming)
        
        
        time.sleep(0.2) #Execution time for every loop 
	    
	#keyboard control - stop motion 
    # nb = input('')
    # number = int(nb)	
    # if(number==2):
	
	    # v=0 #vehicle is stopped
	
#Post ALlocation
errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)

# plt.show()
