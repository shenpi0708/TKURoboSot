#!/usr/bin/env python
import rospy
import sys
import math
import time
from std_msgs.msg import String
from my_state_machine import MyStateMachine
import dynamic_reconfigure.client
from robot.obstacle import Obstacle

class Strategy(object):
  def __init__(self, sim=False):
    rospy.init_node('core', anonymous=True)
    self.rate = rospy.Rate(200)
    self.robot = MyStateMachine(sim)
    self.main()

  def main(self):
    while not rospy.is_shutdown():
      print(self.robot.current_state)
      self.robot.RobotStatePub(self.robot.current_state.name,self.robot.PassRequestPass,self.robot.PassRequestCatch)
      self.robot.Requestsignal()
      #print(" a= ",self.robot.is_pass)
      #print(" b= ",self.robot.Other_PassRequestCatch)
      # self.robot.ShowRobotInfo()
      t = self.robot.GetObjectInfo()
      l = self.robot.GetObstacleInfo() 

      targets = self.robot.GetObjectInfo()
      position = self.robot.GetRobotInfo()
      otherrobot = self.robot.GetRobotOther()
      shootcheck = self.robot.GetREInfo()
      #print(position)
      # Can not find ball when starting
      if self.robot.defence:
        canpassball=True
      else :
        canpassball=False
      if self.robot.CheckBallHandle():  
        if self.robot.AC.ball_pass_check(t[self.robot.opp_side]['dis'],\
                                       t[self.robot.opp_side]['ang'],\
                                       l['ranges'],\
                                       l['angle']['increment']):
          self.robot.PassRequestPass = True
      if self.robot.Other_PassRequestPass:
        
        if self.robot.canpassball==True:
          if not "robot1" in rospy.get_namespace():
            self.robot.PassRequestCatch= True
  
      if targets is None or targets['ball']['ang'] == 999 and self.robot.game_start and not self.robot.PassRequestCatch == True:
        print("Can not find ball")
        self.robot.toIdle()
      else:
         
        if not self.robot.is_idle and not self.robot.game_start:
          self.robot.toIdle()
        if self.robot.PassRequestCatch == True and self.robot.game_start:
          self.robot.toSupporter()
          self.robot.ball_passingcatch = True
        if self.robot.is_idle:
          if self.robot.game_start:
            if self.robot.is_supporterrobot==True or self.robot.test:
              self.robot.toSupporter()
            else:  
              self.robot.toChase()
        if  self.robot.PassRequestPass == True:
          if self.robot.game_start:
            if self.robot.CheckBallHandle():
              if self.robot.R2_PassRequestCatch == True:
                if not "robot2" in rospy.get_namespace():
                  self.robot.ball_passingpass = True
                  self.robot.toAttack()
              elif self.robot.R3_PassRequestCatch == True:
                if not "robot3" in rospy.get_namespace():
                  self.robot.ball_passingpass = True
                  self.robot.toAttack()

        if self.robot.is_chase:
          if self.robot.CheckBallHandle():
            self.robot.toAttack()
          elif self.robot.ball_passingpass == True:
            if  self.robot.is_pass:
              self.robot.ball_passingpass = False
              self.robot.is_pass = False
              self.robot.PassRequestPass = False
              self.robot.toSupporter()
              self.robot.is_supporterrobot=True
        if self.robot.is_supporter:
          if self.robot.ball_passingcatch :
            if  shootcheck:
              self.robot.ball_passingcatch = False
              self.robot.PassRequestCatch = False
              self.robot.is_catch = True
              self.robot.toChase()
           

        if self.robot.is_attack:
          if not self.robot.CheckBallHandle():
            self.robot.toChase()
            #self.robot.toSupporter()
          else : 
            if self.robot.is_catch == True:
              self.robot.is_catch = False
              self.robot.PassRequestCatch = False
            if self.robot.ball_passingpass == True:
              if shootcheck:
                self.robot.toShoot(50) 
                self.robot.is_pass = True

            if abs(targets[self.robot.opp_side]['ang']) < self.robot.atk_shoot_ang and \
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
