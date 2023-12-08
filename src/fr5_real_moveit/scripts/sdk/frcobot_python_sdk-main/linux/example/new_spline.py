import frrpc

# A connection is established with the robot controller. A successful connection returns a robot object
robot = frrpc.RPC('192.168.33.211')

J1 = [114.578,-117.798,-97.745,-54.436,90.053,-45.216]
P1 = [-140.418,619.351,198.369,-179.948,0.023,69.793]
eP1 = [0.000,0.000,0.000,0.000]
dP1 = [0.000,0.000,0.000,0.000,0.000,0.000]
J2 = [115.401,-105.206,-117.959,-49.727,90.054,-45.222]
P2 = [-95.586,504.143,186.880,178.001,2.091,70.585]
J3 = [135.609,-103.249,-120.211,-49.715,90.058,-45.219]
P3 = [-252.429,428.903,188.492,177.804,2.294,90.782]
J4 = [154.766,-87.036,-135.672,-49.045,90.739,-45.223]
P4 = [-277.255,272.958,205.452,179.289,1.765,109.966]

robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
robot.NewSplineStart(1)    #Spline motion start
robot.NewSplinePoint(J1,P1,0,0,50.0,50.0,50.0,0.0,0)    #Spline control point
robot.NewSplinePoint(J2,P2,0,0,50.0,50.0,50.0,0.0,0)
robot.NewSplinePoint(J3,P3,0,0,50.0,50.0,50.0,0.0,0)
robot.NewSplinePoint(J4,P4,0,0,50.0,50.0,50.0,0.0,1)
robot.NewSplineEnd()       #Spline motion end