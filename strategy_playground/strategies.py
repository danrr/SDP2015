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

    UNALIGNED, DEFEND_GOAL = 'UNALIGNED', 'DEFEND_GOAL'
    STATES = [UNALIGNED, DEFEND_GOAL]
    LEFT, RIGHT = 'left', 'right'
    SIDES = [LEFT, RIGHT]

    GOAL_ALIGN_OFFSET = 60

    def __init__(self, world):
        super(DefenderDefence, self).__init__(world, self.STATES)

        #Each action involved in the strategy
        self.NEXT_ACTION_MAP = {
            self.UNALIGNED: self.align,
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
            #Calculates which wheels need to move and how fast
            return calculate_motor_speed(displacement, angle, backwards_ok=True)

    def defend_goal(self):
        """
        Run around, blocking shots.
        """
        # Predict where they are aiming.
        if self.ball.velocity > BALL_VELOCITY:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.ball, bounce=False)

        if self.ball.velocity <= BALL_VELOCITY or predicted_y is None:
            predicted_y = predict_y_intersection(self.world, self.our_defender.x, self.their_attacker, bounce=False)
        # If the attacking robot is facing towards the goal
        if predicted_y is not None:
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x,
                                                                           predicted_y - 7*math.sin(self.our_defender.angle))
            #Calculates which wheels need to move and how fast
            return calculate_motor_speed(displacement, angle, backwards_ok=True)
        #If the attacking robot is facing the wrong way
        else:
            y = self.ball.y
            y = max([y, 60])
            y = min([y, self.world._pitch.height - 60])
            displacement, angle = self.our_defender.get_direction_to_point(self.our_defender.x, y)
            #Calculates which wheels need to move and how fast
            return calculate_motor_speed(displacement, angle, backwards_ok=True)

    def get_alignment_position(self, side):
        """
        Given the side, find the x coordinate of where we need to align to initially.
        """
        assert side in self.SIDES
        if side == self.LEFT:
            return self.world.our_goal.x + self.GOAL_ALIGN_OFFSET
        else:
            return self.world.our_goal.x - self.GOAL_ALIGN_OFFSET


class AttackerDefend(Strategy):

    UNALIGNED, BLOCK_PASS = 'UNALIGNED', 'BLOCK_PASS'
    STATES = [UNALIGNED, BLOCK_PASS]

    def __init__(self, world):
        super(AttackerDefend, self).__init__(world, self.STATES)

        #Each action involved in the strategy
        self.NEXT_ACTION_MAP = {
            self.UNALIGNED: self.align,
            self.BLOCK_PASS: self.block_pass
        }

        zone = self.world._pitch._zones[self.world.our_attacker.zone]
        min_x, max_x, min_y, max_y  = zone.boundingBox()
        self.center_x = (min_x + max_x)/2
        self.center_y = (min_y + max_y)/2
        self.our_attacker = self.world.our_attacker
        self.our_defender = self.world.our_defender
        self.their_attacker = self.world.their_attacker
        self.their_defender = self.world.their_defender

    def align(self):
        """
        Align yourself with the middle of our zone.
        """
        #If we're in the center of our zone
        if has_matched(self.our_attacker, x=self.center_x, y=self.our_attacker.y):
            # We're there. Advance the states and formulate next action.
            self.current_state = self.BLOCK_PASS
            return do_nothing()
        else:
            displacement, angle = self.our_attacker.get_direction_to_point(
                self.center_x, self.our_attacker.y)
            #Calculates which wheels need to move and how fast
            return calculate_motor_speed(displacement, angle, backwards_ok=True)

    def block_pass(self):
        predicted_y = predict_y_intersection(self.world, self.our_attacker.x, self.their_defender, full_width=True, bounce=True)
        #If defending robot is facing the wrong way
        if predicted_y is None:
            ideal_x = self.our_attacker.x
            ideal_y = (self.their_attacker.y + self.their_defender.y) / 2
        else:
            ideal_x = self.our_attacker.x
            ideal_y = predicted_y - 7  *math.sin(self.our_attacker.angle)

        displacement, angle = self.our_attacker.get_direction_to_point(ideal_x, ideal_y)
        if not has_matched(self.our_attacker, ideal_x, ideal_y):
            #Calculates which wheels need to move and how fast
            return calculate_motor_speed(displacement, angle, backwards_ok=True)
        else:
            return do_nothing()


class AttackerPositionCatch(Strategy):
    '''
    This catching strategy positions the robot in the middle of the zone
    so that (ideally) it does not need to do anything
    '''
    PREPARE, ALIGN, ROTATE = 'PREPARE', 'ALIGN', 'ROTATE'
    STATES = [PREPARE, ALIGN, ROTATE]

    def __init__(self, world):
        super(AttackerPositionCatch, self).__init__(world, self.STATES)

        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.ALIGN: self.align,
            self.ROTATE: self.rotate
        }

        self.our_attacker = self.world.our_attacker
        self.our_defender = self.world.our_defender
        self.ball = self.world.ball
        zone = self.world._pitch._zones[self.our_attacker.zone]
        min_x, max_x, min_y, max_y  = zone.boundingBox()
        self.center_x = (min_x + max_x)/2
        self.center_y = (min_y + max_y)/2

    def prepare(self):
        self.current_state = self.ALIGN
        if self.our_attacker.catcher == 'closed':
            self.our_attacker.catcher = 'open'
            return open_catcher()
        else:
            return do_nothing()

    def align(self):
        if has_matched(self.our_attacker, x=self.center_x, y=self.center_y):
            self.current_state = self.ROTATE
            return do_nothing()
        else:
            displacement, angle = self.our_attacker.get_direction_to_point(
                self.center_x, self.center_y)
            return calculate_motor_speed(displacement, angle, backwards_ok=True)

    def rotate(self):
        '''
        Rotate in the center of the zone in order to intercept the pass of the defender.
        Tries to match the correct angle given the angle of the defender.
        '''
        defender_angle = self.our_defender.angle
        attacker_angle = None
        our_side = self.world._our_side
        if our_side == 'left':
            if defender_angle > 0 and defender_angle < pi / 2:
                attacker_angle = 3 * pi / 4
            elif defender_angle > 3 * pi / 2:
                attacker_angle = 5 * pi / 4
        else:
            if defender_angle > pi / 2 and defender_angle < pi:
                attacker_angle = pi / 4
            elif defender_angle > pi and defender_angle < 3 * pi / 2:
                attacker_angle = 7 * pi / 4

        if attacker_angle:
            # Offsets the attacker's position in the direction of the desired angled in order to calculate the
            # required rotation.
            displacement, angle = self.our_attacker.get_direction_to_point(self.our_attacker.x + 10 * math.cos(attacker_angle),
                                                                           self.our_attacker.y + 10 * math.sin(attacker_angle))
            return calculate_motor_speed(None, angle, careful=True)

        return do_nothing()




class DefenderPass(Strategy):

    POSITION, ROTATE, SHOOT, FINISHED = 'POSITION', 'ROTATE', 'SHOOT', 'FINISHED'
    STATES = [POSITION, ROTATE, SHOOT, FINISHED]


    def __init__(self, world):
        super(DefenderPass, self).__init__(world, self.STATES)

        # Each action involved in the strategy
        self.NEXT_ACTION_MAP = {
            self.POSITION: self.position,
            self.ROTATE: self.rotate,
            self.SHOOT: self.shoot,
            self.FINISHED: do_nothing
        }

        self.our_defender = self.world.our_defender

    def position(self):

        """
        Position the robot in the middle close to the goal. Angle does not matter.
        Executed initially when we've grabbed the ball and want to move.
        """
        ideal_x, ideal_y = self.shooting_pos
        distance, angle = self.our_defender.get_direction_to_point(ideal_x, ideal_y)

        if has_matched(self.our_defender, x=ideal_x, y=ideal_y):
            self.current_state = self.ROTATE
            return do_nothing()
        else:
            return calculate_motor_speed(distance, angle, careful=True)

    def rotate(self):
        """
        Calculate the angle of where the robots facing to the centre
        of enemy goal and turn to that position
        """
        goal_x= self.world._their_goal.x
        goal_y= self.world._their_goal.y
        angle= self.our_defender.get_rotation_to_point(goal_x,goal_y)
        self.current_state= self.shoot
        return calculate_motor_speed(None, angle, careful=True)


    def shoot(self):
        """
        Kick.
        """
        self.current_state = self.FINISHED
        self.our_defender.catcher = 'open'
        return kick_ball()



class AttackerGrab(Strategy):

    PREPARE, GO_TO_BALL, GRAB_BALL, GRABBED = 'PREPARE', 'GO_TO_BALL', 'GRAB_BALL', 'GRABBED'
    STATES = [PREPARE, GO_TO_BALL, GRAB_BALL, GRABBED]

    def __init__(self, world):
        super(AttackerGrab, self).__init__(world, self.STATES)

        #Each action involved in the stratgey
        self.NEXT_ACTION_MAP = {
            self.PREPARE: self.prepare,
            self.GO_TO_BALL: self.position,
            self.GRAB_BALL: self.grab,
            self.GRABBED: do_nothing
        }

        self.our_attacker = self.world.our_attacker
        self.ball = self.world.ball

    def prepare(self):
        self.current_state = self.GO_TO_BALL
        if self.our_attacker.catcher == 'closed':
            self.our_attacker.catcher = 'open'
            return open_catcher()
        else:
            return do_nothing()

    def position(self):
        displacement, angle = self.our_attacker.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_attacker.can_catch_ball(self.ball):
            self.current_state = self.GRAB_BALL
            return do_nothing()
        else:
            return calculate_motor_speed(displacement, angle, careful=True)

    def grab(self):
        if self.our_attacker.has_ball(self.ball):
            self.current_state = self.GRABBED
            return do_nothing()
        else:
            self.our_attacker.catcher = 'closed'
            return grab_ball()


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
                return calculate_motor_speed(displacement, angle, backwards_ok=True)

        self.current_state = self.GO_TO_BALL
        return do_nothing()

    def position(self):
        displacement, angle = self.our_defender.get_direction_to_point(self.ball.x, self.ball.y)
        if self.our_defender.can_catch_ball(self.ball):
            self.current_state = self.GRAB_BALL
            return do_nothing()
        else:
            return calculate_motor_speed(displacement, angle, careful=True)

    def grab(self):
        if self.our_defender.has_ball(self.ball):
            self.current_state = self.GRABBED
            return do_nothing()
        else:
            self.our_defender.catcher = 'closed'
            return grab_ball()



class AttackerShoot(Strategy):

    POSITION, ROTATE, SHOOT, FINISHED = 'POSITION', 'ROTATE', 'SHOOT', 'FINISHED'
    STATES = [POSITION, ROTATE, SHOOT, FINISHED]

    def __init__(self, world):
        super(AttackerShoot, self).__init__(world, self.STATES)

        # Each action involved in the strategy
        self.NEXT_ACTION_MAP = {
            self.POSITION: self.position,
            self.ROTATE: self.rotate,
            self.SHOOT: self.shoot,
            self.FINISHED: do_nothing
        }

        self.our_attacker = self.world.our_attacker

    def position(self):

        """
        Position the robot in the middle close to the goal. Angle does not matter.
        Executed initially when we've grabbed the ball and want to move.
        """
        ideal_x, ideal_y = self.shooting_pos
        distance, angle = self.our_attacker.get_direction_to_point(ideal_x, ideal_y)

        if has_matched(self.our_attacker, x=ideal_x, y=ideal_y):
            self.current_state = self.ROTATE
            return do_nothing()
        else:
            return calculate_motor_speed(distance, angle, careful=True)

    def rotate(self):
        """
        Calculate the angle of where the robots facing to the centre
        of enemy goal and turn to that position
        """
        goal_x= self.world._their_goal.x
        goal_y= self.world._their_goal.y
        angle= self.our_attacker.get_rotation_to_point(goal_x,goal_y)
        self.current_state= self.shoot
        return calculate_motor_speed(None, angle, careful=True)


    def shoot(self):
        """
        Kick.
        """
        self.current_state = self.FINISHED
        self.our_attacker.catcher = 'open'
        return kick_ball()
