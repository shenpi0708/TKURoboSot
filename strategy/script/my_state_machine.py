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
    #self.defence = config['defence']
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
  supporter = State('Supporter')
  toIdle   = chase.to(idle) | attack.to(idle)  | shoot.to(idle) | defence.to(idle) | idle.to.itself() |supporter.to(idle)
  toChase  = idle.to(chase) | attack.to(chase) | chase.to.itself() | defence.to(chase)|supporter.to(chase)
  toAttack = attack.to.itself() | shoot.to(attack) | chase.to(attack) |supporter.to(attack)
  toDefence = defence.to.itself() | idle.to(defence) | chase.to(defence)
  toShoot  = attack.to(shoot) | idle.to(shoot)
  toSupporter =  attack.to(supporter)  | idle.to(supporter ) | supporter.to.itself()| chase.to(supporter)
  def on_toIdle(self):
    self.MotionCtrl(0,0,0)

  def on_toChase(self,method="Classic"):
    check = self.GetChass()
    if not check:
      method == "ballpasschase"

    t = self.GetObjectInfo()
    side = self.opp_side
    if method == "Classic":
      x, y, yaw = self.CC.StraightForward(\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
    if method == "ballpasschase":
      x, y, yaw = self.CC.StraightForward(\
                                          t['ball']['dis'],\
                                          t['ball']['ang'])
    self.MotionCtrl(x, y, yaw)
    
  def on_toAttack(self, method = "ball_pass"):
    t = self.GetObjectInfo()
    l = self.GetObstacleInfo()  
    rs = self.GetRobotInfo()
    ro = self.GetRobotOther()
    side = self.opp_side
    ourside = self.our_side

      
    if  self.test:
      method = "ball_pass"
    else :
      method = "Classic"
    
    
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
      x, y, yaw = self.AC.ball_pass(rs['location']['x'],rs['location']['y'],rs['location']['yaw'],ro['position']['x'],ro['position']['y'],ro['position']['yaw'])

    self.MotionCtrl(x, y, yaw)

  def on_toDefence(self, method = "Classic"):
    t = self.GetObjectInfo()
    side = self.our_side

    rs = self.GetRobotInfo()
    ro = self.GetRobotOther()

    if method == "Classic":
      x, y, yaw = self.DC.Toball(t['ball']['dis'], t['ball']['ang'], t[side]['dis'], t[side]['ang'])
    self.MotionCtrl(x, y, yaw)

  def on_toShoot(self, power, pos = 1):
    self.RobotShoot(power, pos)

  def on_toSupporter(self,method = "ball_pass"):
    t = self.GetObjectInfo()
    side = self.our_side

    rs = self.GetRobotInfo()
    ro = self.GetRobotOther()

    if method == "ball_pass":
      x, y, yaw = self.AC.ball_pass(rs['location']['x'],rs['location']['y'],rs['location']['yaw'],ro['position']['x'],ro['position']['y'],ro['position']['yaw'])
    
    self.MotionCtrl(x, y, yaw)


  def CheckBallHandle(self):
    if self.RobotBallHandle():
      ## Back to normal from Accelerator
      self.ChangeVelocityRange(self.minimum_v, self.maximum_v)
      self.last_ball_dis = 0

    return self.RobotBallHandle()
