#git add mainPID.py && git commit -m 'commit' && git push origin main
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

matriculas = [2017003772]
freqMat = 0.0
timeMat = 0.0
estado = "busca"


kp = 1
ki = 0.04
kd = 0.02

lastError = 0
sumError = 0

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw
    
def mediaSomaMatriculas(arrayMat):
    avg = 0
    for matricula in arrayMat:
        sumMat=0
        for i in str(matricula):
            sumMat += int(i)
            
        avg+=sumMat
        
    avg = avg/len(arrayMat)
    return avg

freqMat = mediaSomaMatriculas(matriculas)
timeMat = 1.0/freqMat
print(freqMat, timeMat)

# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    global lastError, sumError, estado
    
    setpoint = 0.5
    scan_len = len(scan.ranges)
    msg = Twist()
   
    # POSICIONA DIRECAO ---------------------------------   
    if estado == 'busca':
        if scan_len > 0:
            print(min(scan.ranges[scan_len-10 : scan_len+10]))
            if min(scan.ranges[scan_len-10 : scan_len+10]) < 100:
                estado = 'avanca'
                msg.angular.z = 0
            
            else:
                msg.angular.z = 0.3 
        
        else:
            msg.angular.z = 0
        
    elif estado == 'avanca':
  
    # AVANCA --------------------------------
    
        if scan_len > 0:
            read = min(scan.ranges[scan_len-10 : scan_len+10])
    
            error = -(setpoint - read)
            varError = (error-lastError)/timeMat
            sumError+=error*timeMat
            
            P = kp*error
            I = ki*sumError
            D = kd*varError
            control = P+I+D
            #print(P, I, D, control)
            print(min(scan.ranges[scan_len-10 : scan_len+10]))
            
            if control > 1:
                control = 1
            elif control < -1:
                control = -1
        else:
            control = 0        
        
        msg.linear.x = control
        
        if abs(error) < 0.05 and abs(sumError) <0.1 and abs(varError) <0.01:
            estado = 'chegou'
            print ('Chegou ao destino')
    
    elif estado == 'chegou':
        msg.linear.x = 0
        msg.angular.z = 0
    
    
    
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(timeMat), timerCallBack)

rospy.spin()