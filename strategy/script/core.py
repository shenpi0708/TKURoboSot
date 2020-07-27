#!/usr/bin/env python
import rospy
import sys
import math
import time
from std_msgs.msg import String
from my_state_machine import MyStateMachine
import dynamic_reconfigure.client

class Strategy(object):
  def __init__(self, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(200)
    self.robot = MyStateMachine(sim)
    self.main()

  def main(self):
    ball_pass_finsh = True
    while not rospy.is_shutdown():
      print(self.robot.current_state)
      self.robot.RobotStatePub(self.robot.current_state.name)
      self.robot.Requestsignal()
      # self.robot.ShowRobotInfo()
      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()
      otherrobot = self.robot.GetRobotOther()
      shootcheck = self.robot.GetREInfo()
      # Can not find ball when starting
      if targets is None or targets['ball']['ang'] == 999 and self.robot.game_start:
        print("Can not find ball")
        self.robot.toIdle()
      else:
        if not self.robot.is_idle and not self.robot.game_start:
          self.robot.toIdle()

        if self.robot.is_idle:
          if self.robot.game_start:
            if not self.robot.test:
              self.robot.toChase()
            else:
              self.robot.toSupporter()


        if self.robot.is_chase:
          if self.robot.CheckBallHandle():
            self.robot.toAttack()
          elif otherrobot['state'] =="Chase":
            if not ball_pass_finsh:
              self.robot.toSupporter()
        if self.robot.is_supporter:
          if  shootcheck:
            if ball_pass_finsh:
              if not otherrobot['state'] =="Chase":
                if targets['ball']['dis']<=100:
                  self.robot.toChase()

        if self.robot.is_attack:
          if not self.robot.CheckBallHandle():
            #self.robot.toChase()
            self.robot.toSupporter()
          elif shootcheck:
            self.robot.toShoot(49) 
            ball_pass_finsh = False

          elif  abs(targets[self.robot.opp_side]['ang']) < self.robot.atk_shoot_ang and \
                abs(targets[self.robot.opp_side]['dis']) < self.robot.atk_shoot_dis:
            self.robot.toShoot(100)


        if self.robot.is_shoot:
          self.robot.toAttack()

        if rospy.is_shutdown():
          break
        

        ## Keep Current State Running
        keepState = 'to' + self.robot.current_state.name
        getattr(self.robot, keepState)()
        
        self.rate.sleep()

if __name__ == '__main__':
  try:
      s = Strategy(True) # True is simulated mode
  except rospy.ROSInterruptException:
    pass 
