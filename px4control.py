from dronekit import connect,VehicleMode
import argparse
import math
import time

''' Argument Parse '''

parser = argparse.ArgumentParser(description='Print out vehicle state information.')
parser.add_argument('--connect', default='com3', help="vehicle connection target. Default 'com3'")
parser.add_argument('--baud', default='115200', help="vehicle baud rate. Default '115200'")
args = parser.parse_args()

''' Global Variables '''
vehicle = connect(args.connect,baud=args.baud,wait_ready=True)
altCheck = 0
CAMERA_ANGLE = 20
run = True

''' Metric Conversions '''
deg radianToDegree(arg):
	return float(arg*180/Math.pi)
deg degreeToRadian(arg):
	return float(arg*Math.pi/180)
def meterToFeet(arg):
	return float(arg*3.28084)
def feetToMeter(arg):
	return float(arg/3.28084)

''' Check Up  & Set Up '''
def checkVehicle(v):
	error = []
	if(v):
		print "Attitude ", v.attitude
		if(type(v.attitude) is not dict):
			error.append('Gyro Error')
		print "Battery ", v.battery
		if(v.battery.voltage ==0):
			error.append('Battery Voltage 0')
		print "Channels ", v.channels
		if(type(v.channels) is not dict):
			error.append('No channel info')
		print "GPS ", v.gps_0
		if(v.gps_0.satellites_visible==0):
			error.append('GPS Error: No satelites')
		elif(v.gps_0.fix<2):
			error.append('No GPS lock')
		print 'Vehicle Airspeed: ', v.airspeed
		print 'Vehicle Ground Speed', v.groundspeed
		if(v.airspeed==0 or v.groundspeed==0 ):
			error.append('Airspeed Sensor error')
		print "Velocity: %s" % v.velocity
		if(v.velocity[0] ==0):
			error.append('Check Accelerometer')
			
		print "Arming status: ", v.armed
	return error
def Armable(v):
	checkUp = False
	error = checkVehicle(v)
	if(len(error)!=0):
		print error
	else:
		checkUp = True
		vehicle.add_attribute_listener()
	print 'Armable: ', checkUp
def setMode():
	vehicle.mode = VehicleMode("STABILIZE")
	print vehicle.mode

''' Listeners '''

def CheckGPSLock(self,value):
	print 'GPS fix changed to: ',value
	if(value>1):
		Armable(vehicle)
	
def altitudeCheck(self,value):
	if(meterToFeet(value.alt)>100):
		altCheck = 1
		print 'Altitude: ', meterToFeet(value.alt)
def armPX4():
	if(vehicle.is_armable):
		ready  = input('Arm? 0/1 : ')
		if(ready == 1):
			vehicle.armed = True
		else:
			abort = input('Abort? 0/1 :')
			if(abort == 1):
				print 'Aborted. Pull the plug.'
				shutdownPX4()
			else:
				armPX4()
	else:
		print 'Pixhawk no longer Armable' 
		Armable(vehicle)

def listeners():
	vehicle.add_attribute_listener('gps_0',CheckGPSLock)
	vehicle.add_attribute_listener('location.global_relative_frame',altitudeCheck)
	vehicle.add_attribute_listener('is_armable',armPX4)
	vehicle.add_attribute_listener('armed',waitOnLock)


''' Drop '''
def dropPayload():
	vehicle.channels.override['5'] = 1500

def getTimeToDrop(b,r):
	return float((b-r)/vehicle.airspeed)
def getRange(t):
	return float(vehicle.airspeed*t)
def getTimeOfFlight():
	return math.sqrt(2*vehicle.location.global_relative_frame.alt*9.8)
def getDistanceToTarget(DIP_angle):
	angle = float(radianToDegree(vehicle.attitude.pitch)+CAMERA_ANGLE+DIP_angle)
	b = float(vehicle.location.global_relative_frame/math.tan(angle))
	return b

def getCountDown(DIP_angle):
	t = getTimeOfFlight()
	r = getRange(t)
	b = getDistanceToTarget(DIP_angle)
	countdown = getTimeToDrop(b,r)
	return countdown
def dropPackage(DIP_angle):
	countdown = getCountDown(DIP_angle)
	i = countdown
	while(i>0):
		i = i - 0.01
		time.sleep(0.01)
	dropPayload()

''' DIP '''
def waitOnLock():
	if(vehicle.armed):
		flag = input('Start DIP? : 0/1')
		if(flag == 1):
			dipFunction()
	
''' Worker '''	
	
def main(v):
	print 'System Status: ', v.system_status
	print 'Vehicle Mode: ', v.mode
	if(v.mode.name!='STABILIZE'):
		setMode()
	print 'Armed: ', v. armed
	Armable(v)
	listeners()
	print "Is Armable: ", v.is_armable
	
	
''' Shutdown '''

def shutdownPX4():
	run = False
	vehicle.armed = False
	vehicle.close()
	print "DC vehicle"
	
		

main(vehicle)
while(run):



	