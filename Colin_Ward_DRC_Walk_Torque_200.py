#!/usr/bin/env python
import hubo_ach as ha
import ach
import sys
import time
from ctypes import *
import math
# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#Below are the length's of the Legs of the robot
l1=340.03
l2=340.38
l3=114.97
anglee=0
# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()
for arg in sys.argv:
    argsteplength=arg
# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)
#This uses two IK formulas to lean the robot depending on the leg it has moved forwards The name's within the method may
#be missleading
def IKLeanForwards(leg,angle1static,angle2static,angle3static,angle1kinetic,angle2kinetic,angle3kinetic,leanangle):
	#Below are arrays of motors which are determined by the current leg being moved
	Legdecision1=[ha.LHR , ha.RHR]
	Legdecision2=[ha.RHR , ha.LHR]
	Legdecision3=[ha.LAR, ha.RAR]
	Legdecision4=[ha.RAR , ha.LAR]
	Legdecision5=[ha.LKN, ha.RKN]
	Legdecision6=[ha.RKN, ha.LKN]
	Legdecision7=[ha.LAP , ha.RAP]
	Legdecision8=[ha.RAP , ha.LAP]
	Legdecision9=[ha.LHP, ha.RHP]
	Legdecision10=[ha.RHP , ha.LHP]
	#we want the foot to be level on the ground so we must set the angle with the ground to zero
	anglee=0
	xe=(l1+l2+l3)*.85
	ye=0
	#xe is the position we want the leg(The inverse kinematics are actually 
	#inverted so this is actually the height of the waist)
	#ye is the x position in reality
	#Below are Kinematic formulas
	#Here is a resource to understanding these kinematic formula's
	#http://ocw.mit.edu/courses/mechanical-engineering/2-12-introduction-to-robotics-fall-2005/lecture-notes/chapter4.pdf
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	angle2leanR=2*math.atan(math.sqrt(((l1+l2)**2-(xw**2+yw**2))/(-1*(l1-l2)**2+(xw**2+yw**2))))
	angle1leanR=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3leanR=anglee-angle1leanR-angle2leanR
	#We need to find the difference in the proposed angle and the current angle so we can
	#have a stable and slower transition.
	LegmovementKneeLeanR=-angle2static+angle2leanR
	LegmoventmentHipLeanR=-(angle1static)+angle1leanR
	Leanangle3R=angle3leanR-angle3static
	#ye=115
	#Now we need to use the users step length for the transition (we need to convert to millimeters)
	ye=float(argsteplength)*10**3
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	angle2leanL=2*math.atan(math.sqrt(((l1+l2)**2-(xw**2+yw**2))/(-1*(l1-l2)**2+(xw**2+yw**2))))
	angle1leanL=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3leanL=anglee-angle1leanL-angle2leanL
	LegmovementKneeLeanL=-angle2kinetic+angle2leanL
	LegmoventmentHipLeanL=-(angle1kinetic)+angle1leanL
	Leanangle3L=angle3leanL-angle3kinetic
	#This loop sets all the motor values to the values we found
	for l in range(0,1500):
		time.sleep(.008)
		ref.ref[Legdecision1[leg]]=ref.ref[Legdecision1[leg]]-leanangle/1500
		ref.ref[Legdecision2[leg]]=ref.ref[Legdecision2[leg]]-leanangle/1500
		ref.ref[Legdecision3[leg]]=ref.ref[Legdecision3[leg]]+leanangle/1500
		ref.ref[Legdecision4[leg]]=ref.ref[Legdecision4[leg]]+leanangle/1500
		ref.ref[Legdecision5[leg]]=ref.ref[Legdecision5[leg]]+LegmovementKneeLeanL/1500
		ref.ref[Legdecision6[leg]]=ref.ref[Legdecision6[leg]]+LegmovementKneeLeanR/1500
		ref.ref[Legdecision7[leg]]=ref.ref[Legdecision7[leg]]+Leanangle3L/1500
		ref.ref[Legdecision8[leg]]=ref.ref[Legdecision8[leg]]+Leanangle3R/1500
		ref.ref[Legdecision9[leg]]=ref.ref[Legdecision9[leg]]+LegmoventmentHipLeanL/1500
		ref.ref[Legdecision10[leg]]=ref.ref[Legdecision10[leg]]+LegmoventmentHipLeanR/1500
		r.put(ref)
	return angle1leanR,angle2leanR,angle3leanR,angle1leanL,angle2leanL,angle3leanL
#This lifts the selected leg
def LiftLeg(leg,angle1,angle2,angle3):
	Legdecision1=[ha.RKN , ha.LKN]
	Legdecision2=[ha.RHP , ha.LHP]
	Legdecision3=[ha.RAP, ha.LAP]
	#More kinematics applications note it lifts the leg to 60% of the standard height	
	xe=(l1+l2+l3)*.6
	ye=0
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	angle2lift=2*math.atan(math.sqrt(((l1+l2)**2-(xw**2+yw**2))/(-1*(l1-l2)**2+(xw**2+yw**2))))
	angle1lift=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3lift=anglee-angle1lift-angle2lift
	LegmovementKnee=-angle2+angle2lift
	LegmovementHip=-(angle1)+angle1lift
	LegmovementAnkle=-(angle3)+angle3lift
	#print angle1lift
	#print angle2lift
	#print LegmovementHip
	#print LegmovementKnee
	for l in range(0,1500):
		#This sleep determines how long the motor takes to do its range of motion
		time.sleep(.001)
		ref.ref[Legdecision3[leg]]=ref.ref[Legdecision3[leg]]+LegmovementAnkle/1500
		ref.ref[Legdecision1[leg]]=ref.ref[Legdecision1[leg]]+LegmovementKnee/1500
		ref.ref[Legdecision2[leg]]=ref.ref[Legdecision2[leg]]+LegmovementHip/1500
		#Sets the motor values
		r.put(ref)
	#Return the values we have operated on
	return angle1lift,angle2lift,angle3lift
#Put the Right foot down
def FootDown(leg,angle1lift,angle2lift,angle3lift):
	Legdecision1=[ha.RKN , ha.LKN]
	Legdecision2=[ha.RHP , ha.LHP]
	Legdecision3=[ha.RAP, ha.LAP]
	#Sets the leg to the original starting position height
	xe=(l1+l2+l3)*.85
	#ye=-115
	#Sets the step size or the length of travel
	ye=-float(argsteplength)*10**3
	anglee=0
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	angle2down=2*math.atan(math.sqrt(((l1+l2)**2-(xw**2+yw**2))/(-1*(l1-l2)**2+(xw**2+yw**2))))
	angle1down=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3down=anglee-angle1down-angle2down
	LegmovementKneeDown=-angle2lift+angle2down
	LegmoventmentHipDown=-(angle1lift)+angle1down
	for l in range(0,1500):
		time.sleep(.001)
		ref.ref[Legdecision1[leg]]=ref.ref[Legdecision1[leg]]+LegmovementKneeDown/1500
		ref.ref[Legdecision2[leg]]=ref.ref[Legdecision2[leg]]+LegmoventmentHipDown/1500
		ref.ref[Legdecision3[leg]]=ref.ref[Legdecision3[leg]]+(angle3down-angle3lift)/1500
		r.put(ref)
	return angle1down,angle2down,angle3down

#Main first gets the robot in the ready position and begins the initial lean and then it runs a loop of the walking
#code
def main():
	ref.ref[ha.LKN] = 0
	ref.ref[ha.RKN] = 0
	ref.ref[ha.LAP] = 0
	ref.ref[ha.RAP] = 0
	ref.ref[ha.LHP] = 0
	ref.ref[ha.RHP] = 0
	# Print out the actual position of the LEB
	#print "Joint = ", state.joint[ha.LEB].pos
	l1=340.03
	l2=340.38
	l3=114.97
	xe=(l1+l2+l3)*.85
	ye=0
	anglee=0
	leg=0
	i=0
	xw=xe-l3*math.cos(anglee)
	yw=ye-l3*math.sin(anglee)
	#print xw
	#print yw
	angle2=math.pi-math.acos((l1**2+l2**2-xw**2-yw**2)/(2*l1*l2))
	#angle2=2*math.atan(math.sqrt(((l1+l2)**2-(xw**2+yw**2))/(-1*(l1-l2)**2+(xw**2+yw**2))))
	angle1=math.atan(yw/xw)-math.acos((l1**2-l2**2+xw**2+yw**2)/(2*l1*math.sqrt(xw**2+yw**2)))
	angle3=anglee-angle1-angle2
	# Print out the Left foot torque in X
	#print "Mx = ", state.ft[ha.HUBO_FT_L_FOOT].m_x
	#print angle1
	#print angle2
	# Write to the feed-forward channel This is the set up phase
	for l in range(0,1200):
		time.sleep(.001)
		ref.ref[ha.LKN]=ref.ref[ha.LKN]+angle2/1200
		ref.ref[ha.RKN]=ref.ref[ha.RKN]+angle2/1200
		ref.ref[ha.LAP]=ref.ref[ha.LAP]+(angle3)/1200
		ref.ref[ha.RAP]=ref.ref[ha.RAP]+(angle3)/1200
		ref.ref[ha.LHP]=ref.ref[ha.LHP]+(angle1)/1200
		ref.ref[ha.RHP]=ref.ref[ha.RHP]+(angle1)/1200
		r.put(ref)
	#Prepare for lean manuever
	height=l1*math.cos(angle1)+l2*math.cos(angle2+angle1)+l3*math.cos(angle3+angle1+angle2)
	leanangle=math.asin(83.43/height)
	ref.ref[ha.LHR] = 0
	ref.ref[ha.RHR] = 0
	ref.ref[ha.LAR] = 0
	ref.ref[ha.RAR] = 0
	ref.ref[ha.LSR] = (math.pi)*.05
	ref.ref[ha.RSR]	= -(math.pi)*.05
	for l in range(0,1500):
		time.sleep(.001)
		ref.ref[ha.LHR]=ref.ref[ha.LHR]-leanangle/1500
		ref.ref[ha.RHR]=ref.ref[ha.RHR]-leanangle/1500
		ref.ref[ha.LAR]=ref.ref[ha.LAR]+leanangle/1500
		ref.ref[ha.RAR]=ref.ref[ha.RAR]+leanangle/1500
		r.put(ref)
	angle1Left=angle1
	angle1Right=angle1
	angle2Left=angle2
	angle2Right=angle2
	angle3Left=angle3
	angle3Right=angle3
	distancetrav=0
	#Loop here which just keeps the robot walking
	i=0
	while(i<((5)/float(argsteplength))):
		angle1Right,angle2Right,angle3Right=LiftLeg(leg,angle1Right,angle2Right,angle3Right)
		angle1Right,angle2Right,angle3Right=FootDown(leg,angle1Right,angle2Right,angle3Right)
		time.sleep(.5)
		distancetrav=float(argsteplength)+distancetrav
		print "Distance traveled: ",distancetrav
		if(i==0):
			leanangle=leanangle*-2
		else:
			leanangle=leanangle*-1
		angle1Left,angle2Left,angle3Left,angle1Right,angle2Right,angle3Right=IKLeanForwards(leg,angle1Right,angle2Right,angle3Right,angle1Left,angle2Left,angle3Left,leanangle)
		leg=(leg+1)%2
		i=i+1
	# Close the connection to the channels
	r.close()
	s.close()
main()
