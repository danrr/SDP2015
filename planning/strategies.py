from math import pi, log10
from planning.utilities import predict_y_intersection

GOAL_ALIGN_OFFSET = 60
BALL_VELOCITY = 3
DISTANCE_THRESHOLD = 15
CAREFUL_THRESHOLD = 50
STRAFING_THRESHOLD = pi / 5
TURNING_THRESHOLD = pi / 10


class BaseStrategy(object):
    def __init__(self, world, comms_manager):
        self.world = world
        self.comms_manager = comms_manager
        self.goal_line = self.world.our_goal.x + GOAL_ALIGN_OFFSET * (1 if self.world._our_side == "left" else -1)

    def execute(self):
        raise NotImplementedError

    def update_world(self, position_dictionary):
        self.world.update_positions(position_dictionary)

    def send_correct_strafe(self, distance):
        if abs(distance) < DISTANCE_THRESHOLD:
            return False

        if distance > 0 and self.world._our_side == "left" or \
                distance < 0 and self.world._our_side == "right":
            self.comms_manager.strafe_left(100)
        else:
            self.comms_manager.strafe_right(100)

        return True

    def send_correct_turn(self, angle):
        angle = int(((angle / pi) * 180))
        if angle > 0:
            self.comms_manager.turn_left(angle)
        else:
            self.comms_manager.turn_right(abs(angle))


class GoToBall(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(GoToBall, self).__init__(world, comms_manager)
        self.state = "go to ball"

    def execute(self):
        if not self.world.pitch.zones[self.world.our_defender.zone].isInside(self.world.ball.x, self.world.ball.y) :
            return Intercept(self.world, self.comms_manager)

        if self.world.our_defender.catcher == "closed":
            # move back if the ball is in the catcher area
            if self.world.our_defender.can_catch_ball(self.world.ball):
                self.comms_manager.move_backward(40)
            else:
                self.world.our_defender.catcher = "open"
                self.comms_manager.open_grabber()
            return self

        if self.world.our_defender.can_catch_ball(self.world.ball) and self.world.our_defender.catcher == "open":
            # TODO: make grabbers close depending on ball position
            self.world.our_defender.catcher = "closed"
            self.comms_manager.close_grabber_center()
            return AimAndPass(self.world, self.comms_manager)

        angle = self.world.our_defender.get_rotation_to_point(self.world.ball.x, self.world.ball.y)
        if abs(angle) > TURNING_THRESHOLD:
            self.send_correct_turn(angle)
        else:
            distance_to_ball = self.world.our_defender.get_displacement_to_point(self.world.ball.x, self.world.ball.y)
<<<<<<< HEAD
            speed = int(50 + log10(distance_to_ball)*10) if distance_to_ball <= 100 else 100
            self.comms_manager.move_forward(speed)
=======
            if distance_to_ball > CAREFUL_THRESHOLD:
                self.comms_manager.move_forward(100)
            elif distance_to_ball > DISTANCE_THRESHOLD:
                self.comms_manager.move_forward(50)
            else:
                self.comms_manager.stop()
>>>>>>> strategy-2-reveangeance
        return self


class Intercept(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(Intercept, self).__init__(world, comms_manager)
        self.state = "turning"
        self.our_goal = self.world.our_goal
        # Find the point we want to align to.
        self.goal_front_x = self.get_alignment_position()

    def get_alignment_position(self):
        """
        Find the x coordinate of where we need to align to initially.
        """
        if self.world._our_side == "left":
            return self.world.our_goal.x + GOAL_ALIGN_OFFSET
        else:
            return self.world.our_goal.x - GOAL_ALIGN_OFFSET

    def execute(self):
        if self.world.pitch.zones[self.world.our_defender.zone].isInside(self.world.ball.x, self.world.ball.y) and \
                self.world.ball.velocity < BALL_VELOCITY:
            # ball is in our zone, and it's moving slowly, go and catch
            return GoToBall(self.world, self.comms_manager)

        # if can catch, catch and go to aim and shoot
        if self.world.our_defender.can_catch_ball(self.world.ball) and self.world.our_defender.catcher == "open":
            self.comms_manager.close_grabber_center()
            self.world.our_defender.catcher = "closed"
            return AimAndPass(self.world, self.comms_manager)

        # turn to face enemy attacker
        angle = self.world.our_defender.get_rotation_to_point(self.world.pitch.width / 2, self.world.our_defender.y)
        angle_threshold = STRAFING_THRESHOLD if self.state == "strafing" else TURNING_THRESHOLD
        if abs(angle) > angle_threshold:
            self.state = "turning"
            self.send_correct_turn(angle)
        else:
            # TODO: move away from edges, self.state = "aligning", add collision detection, use goal_front_x

            predicted_y = None
            # if the ball is moving fast move to intercept
            if self.world.ball.velocity > BALL_VELOCITY:
                predicted_y = predict_y_intersection(self.world,
                                                     self.world.our_defender.x,
                                                     self.world.ball,
                                                     bounce=True)
                if predicted_y and self.world.our_defender.catcher == "closed":
                    self.comms_manager.open_grabber()
                    self.world.our_defender.catcher = "open"

            # if the ball is moving slowly or not at all, attempt to
            if self.world.ball.velocity <= BALL_VELOCITY or predicted_y is None:
                predicted_y = predict_y_intersection(self.world,
                                                     self.world.our_defender.x,
                                                     self.world.their_attacker,
                                                     bounce=True)

                if self.world.our_defender.catcher == "closed":
                    self.comms_manager.close_grabber_center()
                    self.world.our_defender.catcher = "closed"

            if not predicted_y:
                predicted_y = self.world.ball.y
                predicted_y = max([predicted_y, 60])
                predicted_y = min([predicted_y, self.world._pitch.height - 60])

            distance_to_move = self.world.our_defender.y - predicted_y

            if self.send_correct_strafe(distance_to_move):
                self.state = "strafing"
            else:
                # TODO: move to be closer to an ideal distance from goal self.state = "aligning", use goal_front_x
                self.comms_manager.stop()

        return self


class AimAndPass(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(AimAndPass, self).__init__(world, comms_manager)
        self.state = "aiming"

    def execute(self):
        if not self.world.our_defender.can_catch_ball(self.world.ball):
            return GoToBall(self.world, self.comms_manager)

        if self.state == "kicking":
            self.comms_manager.kick()
            return Intercept(self.world, self.comms_manager)

        # TODO: be a lot more clever about passing: obstacle avoidance(utilities.is_shot_blocked), bounce passing
        angle = self.world.our_defender.get_rotation_to_point(self.world.our_attacker.x,
                                                              self.world.our_attacker.y
                                                              )
        if abs(angle) > TURNING_THRESHOLD:
            self.send_correct_turn(angle)
        else:
            self.comms_manager.open_grabber()
            self.world.our_defender.catcher = "open"
            self.state = "kicking"

        return self
