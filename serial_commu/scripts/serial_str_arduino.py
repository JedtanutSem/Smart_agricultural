#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import String
from robot_msg.msg import Pwm_controller, Robot_state
import time

arduino_port_name       = '/dev/ttyUSB0'
psoc_port_name          = '/dev/ttyUSB0'
baud_rate               = 115200 #set baud for both controller
serial_timeout          = .1 #in second
data_refresh_rate       = 10 #hz unit

max_pwm                 = 255 #for check from controller node

arduino_name            = 'Arduino'
psoc_name               = 'PSoC'

class Serial_commu:
    def __init__(self,ser_port_arduino='/dev/ttyUSB0', ser_port_psoc ='/dev/ttyACM0', baud_rate=9600, timeout=.1):

        rospy.init_node('Serial_node', anonymous=True)
        print('\n\n------Serial node Start!!!------\n\n')
        rospy.Subscriber('pwm_to_controller', String, self.pwm_get_clbk)
        rospy.Subscriber('robot_state_data', Robot_state, self.robot_state_data_clbk)

        self.ser_port_arduino = ser_port_arduino
        self.ser_port_psoc = ser_port_psoc
        self.baud_rate = baud_rate
        self.timeout = timeout

        #Arduino init port
        try:
            self.arduino = serial.Serial(self.ser_port_arduino, self.baud_rate, timeout=self.timeout)
            time.sleep(0.5)
            print("******************\n")
            self.init_print_serial(arduino_name,self.ser_port_arduino,self.baud_rate)
            time.sleep(0.5)
        except Exception as e:
            self.fail_print_with_exit(e,'Arduino')

        #PSoC init port
        try:
            self.psoc = serial.Serial(self.ser_port_psoc, self.baud_rate, timeout=self.timeout)
            self.init_print_serial(psoc_name,self.ser_port_arduino,self.baud_rate)
            time.sleep(0.5)
        except Exception as e:
            self.fail_print_with_exit(e,'PSoC')

    def init_print_serial(self, device, serial_port, baud_rate):
        print("Serial "+ str(device)+" Init on port : "+str(serial_port)+'  '+'Baud rate : '+str(baud_rate)+'\n')

    def fail_print_with_exit(self,fail_state,fail_device):
        print("******************\n\n")
        print(str(fail_device)+" Serial Fail!!!!!")
        print('Error : '+ str(fail_state))
        print("\n\n******************\n\n")
        exit()

    def fail_print_without_exit(self,fail_state):
        print("******************\n\n")
        print(fail_state)
        print("\n\n******************\n\n")

    def robot_state_data_clbk(self, robot_state_data):
        robot_state = robot_state_data
        self.fail_state = robot_state.fail_state

    #GET DATA FROM PID CONTROLLER NODE
    def pwm_get_clbk(self, pwm_msg):
        pwm_get = pwm_msg
        self.pwmL = pwm_get.pwmL
        self.dirL = pwm_get.dirL
        self.pwmR = pwm_get.pwmR
        self.dirR = pwm_get.dirR
        #print(pwm_get)

    def serial_read(self):
        psoc_read = self.psoc.readline()[:-2]
        print(psoc_read)

    def serial_write(self):
        try:
            if(self.pwmL or self.pwmR <= max_pwm):
                if self.fail_state == 1:
                    str_write = '%s,%s,%s,%s\n' %(0,0,0,0)
                    self.fail_print_without_exit('Robot fail')
                else:
                    str_write = '%s,%s,%s,%s\n' %(self.pwmL,self.dirL,
                                            self.pwmL,self.dirR)
                    self.arduino.write(str_write)
            else:
                self.fail_print_with_exit('PWM value Fail. Please check value from PWM node!!!!!',
                                            'Arduino')

        except AttributeError:
            self.fail_print_with_exit('Please check all node required',
                                        'Arduino')

        except Exception as e_ser_write:
            self.fail_print_with_exit(e_ser_write
                                        ,'Arduino')

if __name__ == '__main__':
    Serial = Serial_commu(ser_port_arduino  = arduino_port_name,
                            ser_port_psoc   = psoc_port_name,
                            baud_rate       = baud_rate,
                            timeout         = serial_timeout)
    rate = rospy.Rate(data_refresh_rate) #data refresh in 10hz
    while not rospy.is_shutdown():
        Serial.serial_write()
        Serial.serial_read()
        rate.sleep()



"""
def vel_clbk(data):
    global vel_x
    vel_x = data.linear.x
def pwm_get_clbk(data):
    global pwmL, pwmR, dirL, dirR
    pwm_str = data.data
    pwm_list = pwm_str.split(',')
    pwmL = float(pwm_list[0])
    dirL = int(pwm_list[1])
    pwmR = float(pwm_list[2])
    dirR = int(pwm_list[3])
    print(pwm_str)

    #pass

if __name__ == "__main__":

    rospy.init_node('read_ser', anonymous=True)
    pub = rospy.Publisher('serial_read_arduino', String, queue_size=1)
    pub_vel = rospy.Publisher('cmd_vel',Twist,queue_size=10)
    rospy.Subscriber("cmd_vel",Twist, vel_clbk)
    rospy.Subscriber('pwm_to_controller', String, pwm_get_clbk)
    rate = rospy.Rate(150) # 10hz
    try:
        arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=.1)
        #rospy.Subscriber('chatter', String, callback)
        #rospy.loginfo("test")

        while not rospy.is_shutdown():
            #ospy.loginfo("loop")
            str = '%s,%s,%s,%s\n' %(pwmL,dirL,pwmR,dirR)#pwm_L, dir_L, pwm_R, dir_R
            #1--> forward , 0--> backward

            #rospy.loginfo(str)
            arduino.write(str)
            data = arduino.readline()[:-2]
            #ospy.loginfo(str)
    # left,right
            if data:
                serial_str = data
                serial_split = serial_str.split(',')
                #rospy.loginfo(len(serial_split))
                try:
                    if len(serial_split) == 2:
                        L_count_str = serial_split[0]
                        L_count_int = int(L_count_str)
                        #x_linear_vel =  max_linear_speed * (x_linear_val / 100)

                        R_count_str = serial_split[0]
                        R_count_int = int(R_count_str)
                        #z_angular_vel = max_angular_speed * (z_angular_val / 100)
#

                        msg = Twist()
                        msg.linear.x = x_linear_vel
                        msg.angular.z = z_angular_vel
                        #pub_vel.publish(msg)

                except:
                    pass

                rospy.loginfo(serial_str)
                pub.publish(serial_str)
                rate.sleep()
    except Exception as e:
        rospy.loginfo("Serial Fail")
        print e
"""
