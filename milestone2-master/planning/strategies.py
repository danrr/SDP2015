from utilities import *
import math
from random import randint

class Strategy(object):

    PRECISE_BALL_ANGLE_THRESHOLD = math.pi / 15.0
    UP, DOWN = 'UP', 'DOWN'

    def __init__(self, world, states):
        self.world = world
        self.states = states
        self._current_state = states[0]

    @property
    def current_state(self):
        return self._current_state

    @current_state.setter
    def current_state(self, new_state):
        assert new_state in self.states
        self._current_state = new_state

    def reset_current_state(self):
        self.current_state = self.states[0]

    def is_last_state(self):
        return self._current_state == self.states[-1]

    def generate(self):
        return self.NEXT_ACTION_MAP[self.current_state]()



class DefenderDefence(Strategy):

    DEFEND_GOAL =  'DEFEND_GOAL'
    STATES = [DEFEND_GOAL]
    LEFT, RIGHT = 'left', 'right'
    SIDES = [LEFT, RIGHT]

    GOAL_ALIGN_OFFSET = 60

    def __init__(self, world):
        super(DefenderDefence, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.DEFEND_GOAL: self.defend_goal
        }

        self.our_goal = self.world.our_goal
        # Find the point we want to align to.
        self.goal_front_x = self.get_alignment_position(self.world._our_side)
        self.their_attacker = self.world.their_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball

    def align(self):
        """
        Align yourself with the center of our goal.
        """
        if has_matched(self.our_defender, x=self.goal_front_x, y=self.our_goal.y):
            # We're there. Advance the states and formulate next action.
            self.current_state = self.DEFEND_GOAL
            return do_nothing()
        else:
            displacement, angle = self.our_defender.get_direction_to_point(
                self.goal_front_x, self.our_goal.y)
            return move(displacement, angle, strafe_ok=True, backwards_ok=True)

    def defend_goal(self):
        """
        Run around, blocking shots.
        """
        # Predict where they are aiming.
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.ball, bounce=False)

        if self.ball.velocity <= BALL_VELOCITY or predicted_y is None: 
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.their_attacker, bounce=False)

        if self.our_defender.catcher == 'closed':
            self.our_defender.catcher = 'open'
            return open_catcher()
        
        if predicted_y is not None:
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x,
                                                                           predicted_y - 7*math.sin(self.our_defender.angle))
            return move(displacement, angle, strafe_ok=True, backwards_ok=True)
        #TODO What is this doing?
        else:
            y = self.ball.y
            y = max([y, 60])
            y = min([y, self.world._pitch.height - 60])
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x, y)
            return move(displacement, angle, backwards_ok=True)

    def get_alignment_position(self, side):
        """
        Given the side, find the x coordinate of where we need to align to initially.
        """
        assert side in self.SIDES
        if side == self.LEFT:
            return self.world.our_goal.x + self.GOAL_ALIGN_OFFSET
        else:
            return self.world.our_goal.x - self.GOAL_ALIGN_OFFSET


class AttackerGrab(Strategy):

    GO_TO_BALL, GRAB_BALL, GRABBED = 'GO_TO_BALL', 'GRAB_BALL', 'GRABBED'
    STATES = [GO_TO_BALL, GRAB_BALL, GRABBED]

    def __init__(self, world):
        super(AttackerGrab, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.GO_TO_BALL: self.position,
            #self.GRAB_BALL: self.grab,
            self.GRABBED: do_nothing
        }

        self.our_attacker = self.world.our_attacker
        self.ball = self.world.ball

    def position(self):
        displacement, angle = self.our_attacker.get_direction_to_point(self.ball.x, self.ball.y)

        if self.our_attacker.catcher == 'closed':
            self.our_attacker.catcher = 'open'
            return open_catcher()
        if self.our_attacker.can_catch_ball(self.ball):
            self.current_state = self.GRABBED
            return grab_ball_center()
        else:
            return move(displacement, angle, careful=True)

    # def grab(self):
    #     if self.our_attacker.has_ball(self.ball):
    #         # self.current_state = self.GRABBED
    #         return do_nothing()
    #     else:
    #         self.our_attacker.catcher = 'closed'
    #         # the angle by which the robot needs to rotate in order to achieve alignment with the ball
    #         angle = 180*self.our_attacker.get_rotation_to_point(self.ball.x, self.ball.y)/math.pi
    #         if angle > 30:
    #             return grab_ball_right()
    #         elif angle < -30:
    #             return grab_ball_left()
    #         else:
    #             return grab_ball_center()


class DefenderGrab(Strategy):

    DEFEND, GO_TO_BALL, GRAB_BALL, GRABBED = 'DEFEND', 'GO_TO_BALL', 'GRAB_BALL', 'GRABBED'
    STATES = [DEFEND, GO_TO_BALL, GRAB_BALL, GRABBED]

    def __init__(self, world):
        super(DefenderGrab, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.DEFEND: self.defend,
            self.GO_TO_BALL: self.position,
            self.GRAB_BALL: self.grab,
            self.GRABBED: do_nothing
        }

        self.our_defender = self.world.our_defender
        self.ball = self.world.ball

    def defend(self):
        '''
        If the ball is heading towards our goal at high velocity then don't go directly into
        grabbing mode once the ball enters our zone. Try to match it's y-coordinate as fast as possible.
        '''
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.ball, bounce=True)

            if predicted_y is not None:
                displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x,
                                                                               predicted_y - 7*math.sin(self.our_defender.angle))
                return move(displacement, angle, strafe_ok=True, backwards_ok=True)
        
        self.current_state = self.GO_TO_BALL
        return do_nothing()

    def position(self):
        displacement, angle = self.our_defender.get_direction_to_point(self.ball.x, self.ball.y)
        # print self.our_defender.catcher
        if self.our_defender.catcher == 'closed':
            self.our_defender.catcher = 'open'
            return open_catcher()
        if self.our_defender.can_catch_ball(self.ball):
            self.current_state = self.GRAB_BALL
            return do_nothing()
        else:
            return move(displacement, angle, careful=True)

    def grab(self):
        if self.our_defender.has_ball(self.ball):
            self.current_state = self.GRABBED
            return do_nothing()
        else:
            self.our_defender.catcher = 'closed'
            # the angle by which the robot needs to rotate in order to achieve alignment with the ball            
            angle = 180*self.our_defender.get_rotation_to_point(self.ball.x, self.ball.y)/math.pi
            if angle > 30:
                return grab_ball_right()
            elif angle < -30:
                return grab_ball_left()
            else:
                return grab_ball_center()


class AttackerShoot(Strategy):

    AIM, OPEN, SHOOT, = 'AIM', 'OPEN', 'SHOOT'
    STATES = [AIM, OPEN, SHOOT]

    def __init__(self, world):
        super(AttackerShoot, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.AIM: self.aim,
            self.OPEN: self.open,
            self.SHOOT: self.shoot,
        }

        self.our_attacker = self.world.our_attacker
        self.ball = self.world.ball

    def aim(self): 
        '''
        Aim towards the centre of the enemy goal (will change this later)
        '''

        self.current_state = self.OPEN
        # Angle to turn in order to aim at the centre of the enemy goal
        angle_to_turn = self.our_attacker.get_rotation_to_point(20, 100)

        # Rotate at the given angle
        return move(0, angle_to_turn)

    def open(self):
        self.current_state = self.SHOOT
        return open_catcher()

    def shoot(self):
        '''
        Shoot at the enemy goal
        '''

        self.current_state = 'FINISHED'
        return kick_ball(100)

class DefenderPass(Strategy):

    AIM, PASS_BALL = 'AIM', 'PASS_BALL'
    STATES = [AIM, PASS_BALL]

    def __init__(self, world):
        super(DefenderPass, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.AIM: self.aim,
            self.PASS_BALL: self.pass_ball,
        }

        self.our_defender = self.world.our_defender
        self.ball = self.world.ball
        
    def aim(self): 
        '''
        Aim towards our attacker zone
        '''

        self.current_state = self.AIM
        # Angle to turn in order to aim at our attacker zone
        angle_to_turn = self.our_defender.get_rotation_to_point(60, 100)

        # Rotate at the given angle
        return move(0, angle_to_turn)

    def pass_ball(self):
        '''
        Pass the ball
        '''
        print "PASS THE FREAKIN BALL"
        self.current_state = self.PASS_BALL
        return kick_ball(70)
