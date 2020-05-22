from statemachine import StateMachine, State
from methods.chase import Chase
from methods.attack import Attack
from methods.defence import Defence
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from strategy.cfg import RobotConfig
from robot.robot import Robot
from strategy.msg import RobotState
class MyStateMachine(Robot, StateMachine):

  def __init__(self, sim = False):
    super(MyStateMachine, self).__init__(sim)
    StateMachine.__init__(self)
    self.CC  = Chase()
    self.DC  = Defence()
    self.AC  = Attack()
    dsrv = DynamicReconfigureServer(RobotConfig, self.Callback)
    

  def Callback(self, config, level):
    self.game_start = config['game_start']
    self.test = config['test']
    self.defence = config['defence']
    self.our_side   = config['our_side']
    self.opp_side   = 'Blue' if self.our_side == 'Yellow' else 'Yellow'
    self.maximum_v = config['maximum_v']
    self.minimum_v = config['minimum_v']
    self.atk_shoot_ang = config['atk_shoot_ang']
    self.atk_shoot_dis = config['atk_shoot_dis']

    self.ChangeVelocityRange(config['minimum_v'], config['maximum_v'])
    self.ChangeAngularVelocityRange(config['minimum_w'], config['maximum_w'])
    self.ChangeBallhandleCondition(config['ballhandle_dis'], config['ballhandle_ang'])

    return config

  idle   = State('Idle', initial = True)
  chase  = State('Chase')
  attack = State('Attack')
  defence = State('Defence')
  shoot  = State('Shoot')

  toIdle   = chase.to(idle) | attack.to(idle)  | shoot.to(idle) | defence.to(idle) | idle.to.itself()
  toChase  = idle.to(chase) | attack.to(chase) | chase.to.itself() | defence.to(chase)
  toAttack = attack.to.itself() | shoot.to(attack) | chase.to(attack)
  toDefence = defence.to.itself() | idle.to(defence) | chase.to(defence)
  toShoot  = attack.to(shoot)| idle.to(shoot)

  def on_toIdle(self):
    self.MotionCtrl(0,0,0)

  def on_toChase(self):
    method = "Classic"
    t = self.GetObjectInfo()
    side = self.opp_side
    
    if method == "Classic":
      x, y, yaw = self.CC.StraightForward(\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
    self.MotionCtrl(x, y, yaw)
    
  def on_toAttack(self, method = "Orbit"):
    t = self.GetObjectInfo()
    l = self.GetObstacleInfo()  
    r2 = self.GetRobot2()
    r3 = self.GetRobot3()
    side = self.opp_side
    ourside = self.our_side

      
    if not self.test:
      method = "Classic"
    elif self.test :
      method = "ball_pass"
    
    
    if method == "Classic":
      x, y, yaw = self.AC.ClassicAttacking(t[side]['dis'], t[side]['ang'])
    elif method == "Orbit":
      x, y, yaw = self.AC.Orbit(t[side]['ang'])
    elif method == "Twopoint":
      x, y, yaw  = self.AC.Twopoint(t[side]['dis'], t[side]['ang'] ,t[ourside]
      ['ang'])
    elif method == "Post_up":
      if t[side]['dis'] < 50 :
        t[side]['dis'] = 50
      x, y, yaw = self.AC.Post_up(t[side]['dis'],\
                                       t[side]['ang'],\
                                       l['ranges'],\
                                       l['angle']['increment'])
    elif method == "ball_pass":
      x, y, yaw = self.AC.ball_pass(r2['position']['x'],r2['position']['y'],r2['position']['yaw'],r3['position']['x'],r3['position']['y'],r3['position']['yaw'])
    self.MotionCtrl(x, y, yaw)

  def on_toDefence(self, method = "ball_pass"):
    t = self.GetObjectInfo()
    side = self.our_side

    r2 = self.GetRobot2()
    r3 = self.GetRobot3()
    if method == "ball_pass":
      x, y, yaw = self.AC.ball_pass(r3['position']['x'],r3['position']['y'],r3['position']['yaw'],r2['position']['x'],r2['position']['y'],r2['position']['yaw'])




    elif method == "Classic":
      x, y, yaw = self.DC.Toball(t['ball']['dis'], t['ball']['ang'], t[side]['dis'], t[side]['ang'])
    self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos = 1):
    self.RobotShoot(power, pos)

  def CheckBallHandle(self):
    if self.RobotBallHandle():
      ## Back to normal from Accelerator
      self.ChangeVelocityRange(self.minimum_v, self.maximum_v)
      self.last_ball_dis = 0

    return self.RobotBallHandle()
