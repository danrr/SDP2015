from math import pi, log
from planning.utilities import predict_y_intersection, is_shot_blocked, is_wall_in_front, centre_of_zone
import time

GOAL_ALIGN_OFFSET = 50
BALL_VELOCITY = 3
DISTANCE_THRESHOLD = 15
AIMING_THRESHOLD = pi / 27
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
        speed = self.calculate_speed(distance)

        if distance < 0 and self.world._our_side == "left" \
                or distance > 0 and self.world._our_side == "right":
            self.comms_manager.strafe_left(speed)
        else:
            self.comms_manager.strafe_right(speed)

        return True

    def send_correct_turn(self, angle):
        angle = int(((angle / pi) * 180))
        if angle > 0:
            self.comms_manager.turn_left(angle)
        else:
            self.comms_manager.turn_right(abs(angle))

    def in_our_half(self):
        return (self.world.pitch.zones[self.world.our_defender.zone].isInside(self.world.ball.x,
                                                                              self.world.ball.y)
                or self.world.pitch.zones[self.world.their_attacker.zone].isInside(self.world.ball.x,
                                                                                   self.world.ball.y))

    @staticmethod
    def calculate_speed(distance):
        speed = 25 + log(abs(distance)) * 5 if distance <= 150 else 80
        return max(int(10 * round(speed / 10)), 40)

    def get_bounded_ball_y(self, full_width=False):
        top_y = self.world._pitch.height - 60 if full_width \
            else self.world.our_goal.y + (self.world.our_goal.width / 2) - 30
        bottom_y = 60 if full_width else \
            self.world.our_goal.y - (self.world.our_goal.width / 2) + 30
        y = self.world.ball.y
        y = max([y, bottom_y])
        y = min([y, top_y])
        return y

    def send_correct_catch(self):
        can_catch = self.world.our_defender.can_catch_ball(self.world.ball)
        if can_catch and self.world.our_defender.catcher == "open":
            if can_catch == "both":
                self.comms_manager.close_grabber_center()
            elif can_catch == "right":
                self.comms_manager.close_grabber_right()
            elif can_catch == "left":
                self.comms_manager.close_grabber_left()
            self.world.our_defender.catcher = "closed"
            return True
        return False

    def move_away_from_wall(self):
        wall = is_wall_in_front(self.world)
        if wall == "both":
            self.comms_manager.move_backward(40)
            return True
        elif wall == "left":
            self.comms_manager.strafe_right(100)
            return True
        elif wall == "right":
            self.comms_manager.strafe_left(100)
            return True
        return False


class Init(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(Init, self).__init__(world, comms_manager)

    def __repr__(self):
        return "Initialize"

    def execute(self):
        self.comms_manager.close_grabber_center()
        if self.world.pitch.zones[self.world.our_defender.zone].isInside(self.world.ball.x, self.world.ball.y):
            if self.world.our_defender.has_ball:
                return BouncePass(self.world, self.comms_manager)
            else:
                return GoToBall(self.world, self.comms_manager)
        else:
            return Intercept(self.world, self.comms_manager)


class GoToBall(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(GoToBall, self).__init__(world, comms_manager)
        self.state = "go to ball"

    def __repr__(self):
        return "Go To Ball"

    def execute(self):
        if not self.world.pitch.zones[self.world.our_defender.zone].isInside(self.world.ball.x, self.world.ball.y):
            return Intercept(self.world, self.comms_manager)

        if self.world.our_defender.has_ball(self.world.ball):
            return BouncePass(self.world, self.comms_manager)

        if self.world.our_defender.catcher == "closed":
            # move back if the ball is in the catcher area
            if self.world.our_defender.can_catch_ball(self.world.ball):
                self.comms_manager.move_backward(40)
            else:
                self.world.our_defender.catcher = "open"
                self.comms_manager.open_grabber()
            return self

        # if self.move_away_from_wall():
        #     return self

        if self.send_correct_catch():
            return BouncePass(self.world, self.comms_manager)

        angle = self.world.our_defender.get_rotation_to_point(self.world.ball.x, self.world.ball.y)
        if abs(angle) > TURNING_THRESHOLD:
            self.send_correct_turn(angle)
        else:
            distance_to_ball = self.world.our_defender.get_displacement_to_point(self.world.ball.x, self.world.ball.y)
            speed = self.calculate_speed(distance_to_ball)
            if distance_to_ball > DISTANCE_THRESHOLD:
                self.comms_manager.move_forward(speed)
            else:
                self.comms_manager.stop()
        return self


class Intercept(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(Intercept, self).__init__(world, comms_manager)
        self.state = "turning"
        self.our_goal = self.world.our_goal
        # Find the point we want to align to.

    def __repr__(self):
        return "Intercept"

    def execute(self):
        if self.world.pitch.zones[self.world.our_defender.zone].isInside(self.world.ball.x, self.world.ball.y) and \
                self.world.ball.velocity < BALL_VELOCITY:
            # ball is in our zone, and it's moving slowly, go and catch
            return GoToBall(self.world, self.comms_manager)

        # if can catch, catch and go to aim and shoot
        if self.send_correct_catch():
            return BouncePass(self.world, self.comms_manager)

        if self.world.our_defender.has_ball(self.world.ball):
            return BouncePass(self.world, self.comms_manager)

        # turn to face enemy attacker
        angle = self.world.our_defender.get_rotation_to_point(self.world.pitch.width / 2, self.world.our_defender.y)
        angle_threshold = STRAFING_THRESHOLD if self.state == "strafing" else TURNING_THRESHOLD
        if abs(angle) > angle_threshold:
            self.state = "turning"
            self.send_correct_turn(angle)
        else:
            # TODO: move away from edges, self.state = "aligning", add collision detection, use goal_front_x
            disp = self.world.our_defender.get_displacement_to_point(self.goal_line, self.world.our_defender.y)
            if disp > DISTANCE_THRESHOLD:
                speed = self.calculate_speed(disp)
                if disp < 0:
                    self.comms_manager.move_forward(speed)
                else:
                    self.comms_manager.move_backward(speed)
                return self
            predicted_y = None
            # if the ball is moving fast move to intercept
            if self.world.ball.velocity > BALL_VELOCITY:
                predicted_y = predict_y_intersection(self.world,
                                                     self.world.our_defender.x,
                                                     self.world.ball,
                                                     bounce=True)
                if predicted_y and self.world.our_defender.catcher == "closed" and self.in_our_half():
                    self.comms_manager.open_grabber()
                    self.world.our_defender.catcher = "open"

            # if the ball is moving slowly or not at all, attempt to
            if self.world.ball.velocity <= BALL_VELOCITY or predicted_y is None:
                predicted_y = predict_y_intersection(self.world,
                                                     self.world.our_defender.x,
                                                     self.world.their_attacker,
                                                     bounce=True)

                if self.world.our_defender.catcher == "open":
                    self.comms_manager.close_grabber_center()
                    self.world.our_defender.catcher = "closed"

            if not predicted_y:
                predicted_y = self.get_bounded_ball_y()

            distance_to_move = self.world.our_defender.y - predicted_y

            if self.send_correct_strafe(distance_to_move):
                self.state = "strafing"
            else:
                # TODO: move to be closer to an ideal distance from goal self.state = "aligning", use goal_front_x
                if abs(disp) > DISTANCE_THRESHOLD * 2:
                    speed = self.calculate_speed(disp)
                    if disp < 0:
                        self.comms_manager.move_forward(speed)
                    else:
                        self.comms_manager.move_backward(speed)
                else:
                    self.comms_manager.stop()

        return self


class AimAndPass(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(AimAndPass, self).__init__(world, comms_manager)
        self.state = "aiming"
        self.time = None

    def __repr__(self):
        return "Aim and Pass"

    def execute(self):

        if self.world.our_defender.caught_area.isInside(self.world.ball.x, self.world.ball.y):
            if self.state == "kicking":
                self.comms_manager.kick()
                self.state = "passed"
                self.time = time.clock()
                return self

        if self.state == "passed":
            if self.time + 0.5 < time.clock():
                return Intercept(self.world, self.comms_manager)
            else:
                return self

        if not self.world.our_defender.has_ball(self.world.ball):
            return GoToBall(self.world, self.comms_manager)

        angle = self.world.our_defender.get_rotation_to_point(self.world.our_attacker.x,
                                                              self.world.our_attacker.y)

        # If the robot has a clear pass after turning by angle then pass
        # If not then move somewhere else and pass
        if not is_shot_blocked(self.world, angle):
            if abs(angle) > AIMING_THRESHOLD:
                self.send_correct_turn(angle)
            else:
                self.comms_manager.stop()
                self.comms_manager.open_grabber()
                self.world.our_defender.catcher = "open"
                self.state = "kicking"
        else:
            # Align before strafing
            angle_to_align = self.world.our_defender.get_rotation_to_point(self.world._pitch.width / 2,
                                                                           self.world.our_defender.y)
            if abs(angle_to_align) > TURNING_THRESHOLD:
                self.send_correct_turn(angle_to_align)
            else:
                if self.world.their_attacker.y < self.world.pitch.height / 3:
                    distance = 8 * self.world.pitch.height / 9 - self.world.our_defender.y
                elif self.world.their_attacker.y > 2 * self.world.pitch.height / 3:
                    distance = self.world.pitch.height / 9 - self.world.our_defender.y
                elif self.world.our_attacker.y > 2 * self.world.pitch.height / 3 or \
                                self.world.our_attacker.y < 1 * self.world.pitch.height / 3:
                    distance = self.world.our_attacker.y - self.world.our_defender.y
                else:
                    distance = self.world.pitch.height / 9 - self.world.our_defender.y
                self.send_correct_strafe(-distance)
        return self


class BouncePass(BaseStrategy):
    def __init__(self, world, comms_manager):
        super(BouncePass, self).__init__(world, comms_manager)
        self.state = "aligning"
        self.time = None

    def __repr__(self):
        return "Bounce Pass"

    def execute(self):
        if self.world.our_defender.caught_area.isInside(self.world.ball.x, self.world.ball.y):
            if self.state == "kicking":
                self.comms_manager.kick()
                self.state = "passed"
                self.time = time.clock()
                return self

        if self.state == "passed":
            if self.time + 0.5 < time.clock():
                return Intercept(self.world, self.comms_manager)
            else:
                return self

        if not self.world.our_defender.has_ball(self.world.ball):
            return GoToBall(self.world, self.comms_manager)

        if self.state == "aligning":
            centre = centre_of_zone(self.world, self.world.our_defender)
            angle = self.world.our_defender.get_rotation_to_point(*centre)
            disp = self.world.our_defender.get_displacement_to_point(*centre)

            if disp > DISTANCE_THRESHOLD:
                if abs(angle) > AIMING_THRESHOLD:
                    self.send_correct_turn(angle)
                else:
                    speed = self.calculate_speed(disp)
                    self.comms_manager.move_forward(speed)
            else:
                self.comms_manager.stop()
                self.state = "aiming"
                
        # If the robot has a clear pass at the top of the pitch pass
        # If not then turn 90 and pass
        if self.state == "aiming":
            if self.world.their_attacker.y > self.world.pitch.height / 2:
                angle = self.world.our_defender.get_rotation_to_point(
                    centre_of_zone(self.world, self.world.their_attacker)[0], 30
                )
            else:
                angle = self.world.our_defender.get_rotation_to_point(
                    centre_of_zone(self.world, self.world.their_attacker)[0], self.world.pitch.height - 30
                )

            if abs(angle) > AIMING_THRESHOLD:
                self.send_correct_turn(angle)
            else:
                self.comms_manager.stop()
                self.comms_manager.open_grabber()
                self.world.our_defender.catcher = "open"
                self.state = "kicking"

        return self
