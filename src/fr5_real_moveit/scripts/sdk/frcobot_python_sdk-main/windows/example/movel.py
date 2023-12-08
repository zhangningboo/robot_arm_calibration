import frrpc

# A connection is established with the robot controller. A successful connection returns a robot object
robot = frrpc.RPC('192.168.3.211')

J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
eP1=[0.000,0.000,0.000,0.000]
dP1=[10.000,10.000,10.000,0.000,0.000,0.000]
J2=[123.709,-121.190,-82.838,-63.499,90.471,-47.174]
P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
eP2=[0.000,0.000,0.000,0.000]
dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
J3=[167.066,-95.700,-123.494,-42.493,90.466,-47.174]
P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
eP3=[0.000,0.000,0.000,0.000]
dP3=[0.000,0.000,0.000,0.000,0.000,0.000]

robot.MoveL(J1,P1,0,0,100.0,180.0,100.0,-1.0,eP1,0,1,dP1)   #Rectilinear motion in Cartesian space
robot.MoveL(J2,P2,0,0,100.0,180.0,100.0,-1.0,eP2,0,0,dP2)
robot.MoveL(J3,P3,0,0,100.0,180.0,100.0,-1.0,eP3,0,0,dP3)