from models import *
from collisions import *
from strategies import *
from utilities import *

GOAL_ALIGN_OFFSET = 80


class Planner:
    def __init__(self, our_side, pitch_num):
        self._world = World(our_side, pitch_num)
        # measurement used 1cm = 2.28px
        # TODO: tinker with these to get the proper grabber areas
        self._world.our_defender.catcher_area = {'width': 50, 'height': 35, 'front_offset': 0}  # 10
        self._world.our_attacker.catcher_area = {'width': 35, 'height': 20, 'front_offset': 18}

        self.is_kicking = False
        self.just_kicked = False
        self.is_catching = None
        self.time = None
        self.old_action = stop()
        self.current_action = stop()
        self.goal_line = self._world.our_goal.x + GOAL_ALIGN_OFFSET * (1 if self._world._our_side == "left" else -1)

    def reset_time(self, command):
        if command == "W" and self.old_action["move"] > 0:
            self.time = time.clock()
        elif command == "S" and self.old_action["move"] < 0:
            self.time = time.clock()
        elif command == "A" and self.old_action["angle"] > 0:
            self.time = time.clock()
        elif command == "D" and self.old_action["angle"] < 0:
            self.time = time.clock()
        elif command == "V" and self.old_action["strafe"] < 0:
            self.time = time.clock()
        elif command == "C" and self.old_action["strafe"] > 0:
            self.time = time.clock()
        elif command == " " and self.old_action == stop():
            self.time = time.clock()

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    def get_strafe_angle(self, distance):
        angle = pi / 2
        angle *= -1 if distance < 0 else 1
        angle *= -1 if self._world._our_side == "right" else 1
        return angle

    def plan(self):
        # TODO: take into account where ball was seen last when it's hidden, now it just assumes we have it
        if (self._world.ball.x, self._world.ball.y) == (0, 0) and self._world.our_defender.catcher == "closed":
            self.aim_and_pass()

        # if ball is in our zone:
        elif self._world.pitch.zones[self._world.our_defender.zone].isInside(self._world.ball.x, self._world.ball.y):
            self.current_action = stop()

            if self.is_kicking:
                self.is_kicking = False
                self.current_action = kick_ball(100)
                self.just_kicked = True

            elif self._world.our_defender.has_ball(self._world.ball):
                self.aim_and_pass()

            elif self._world.our_defender.catcher == "closed" and not self._world.our_defender.can_catch_ball(self._world.ball):
                self._world.our_defender.catcher = "open"
                self.current_action = open_catcher()
            elif self._world.our_defender.can_catch_ball(self._world.ball) and not self.just_kicked:
                # TODO: make grabbers close depending on ball position
                self._world.our_defender.catcher = "closed"
                self.current_action = grab_ball_center()
                self.is_catching = time.clock()
            # if ball moving fast
            elif self._world.ball.velocity > BALL_VELOCITY:
                # try and intercept
                predicted_y = predict_y_intersection(self._world,
                                                     self._world.our_defender.x,
                                                     self._world.ball,
                                                     bounce=False)
                if predicted_y:
                    distance_to_move = self._world.our_defender.y - predicted_y
                    angle = self.get_strafe_angle(distance_to_move)
                    self.current_action = move(distance_to_move, angle, strafe_ok=True, backwards_ok=False)
                if not predicted_y:
                    self.go_to_ball()
            elif not self.just_kicked:
                self.go_to_ball()

        # if ball is in enemy attacker zone, panic:
        elif self._world.pitch.zones[self._world.their_attacker.zone].isInside(self._world.ball.x, self._world.ball.y):

            self.close_and_turn()

            if self.current_action == stop():
                predicted_y = None
                # if the ball is moving fast, attempt to move to intercept ball
                if self._world.ball.velocity > BALL_VELOCITY:
                    predicted_y = predict_y_intersection(self._world,
                                                         self._world.our_defender.x,
                                                         self._world.ball,
                                                         bounce=True)

                # TODO: better way of determining if attacker has ball (is ball invisible or in front of attacker maybe)
                # if ball is moving slowly (or not at all) or not towards us attempt to intercept shot
                if self._world.ball.velocity <= BALL_VELOCITY or predicted_y is None:
                    predicted_y = predict_y_intersection(self._world,
                                                         self._world.our_defender.x,
                                                         self._world.their_attacker,
                                                         bounce=True)
                # if we can intercept
                if predicted_y:
                    distance_to_move = self._world.our_defender.y - predicted_y
                    angle = self.get_strafe_angle(distance_to_move)
                    self.current_action = move(abs(distance_to_move), angle, strafe_ok=True, backwards_ok=False)
            if self.current_action == stop():
                # move to center of goal if there is nothing to do
                self.align_to_goal()
        else:
            self.reset_to_goal()

        if self.just_kicked and not self._world.our_defender.can_catch_ball(self._world.ball):
            self.just_kicked = False
        if self.time and self.time + 0.5 < time.clock():
            self.time = None
            self.old_action = {}
        if self.is_catching and self.is_catching + 0.5 < time.clock():
            self.is_catching = None
        if self.current_action == self.old_action:
            return False
        if self.current_action.get("angle") and self.old_action.get("angle"):
            return False

        self.time = time.clock()
        self.old_action = self.current_action
        return self.current_action

    def go_to_ball(self):
        """
        Makes robot head towards the ball
        """
        angle = self._world.our_defender.get_rotation_to_point(self._world.ball.x, self._world.ball.y)
        self.current_action = move(0, angle)
        if self.current_action == stop():
            distance_to_ball = self._world.our_defender.get_displacement_to_point(self._world.ball.x, self._world.ball.y)
            self.current_action = move(distance_to_ball, 0, careful=True)

    def reset_to_goal(self):
        """
        Returns actions that make robot face opponent attacker and and move to the center of the goal
        """
        self.close_and_turn()
        # if already facing the opponent
        if self.current_action == stop():
            self.align_to_goal()

    def close_and_turn(self):
        """
        Returns actions to close the grabber and turns the robot to face the opponent
        """
        self.current_action = stop()
        if self._world.our_defender.catcher == "open":
            self.current_action = grab_ball_center()
            self._world.our_defender.catcher = "closed"

        # if not facing forward - turn to face forward
        angle = self._world.our_defender.get_rotation_to_point(self._world.pitch.width / 2, self._world.our_defender.y)
        self.current_action = move(0, angle, grabber=self.current_action["grabber"])

    def align_to_goal(self):
        """
        Returns actions that move a robot facing the opponent to the goal line
        """
        # if not aligned with the center of the goal - strafe to center
        distance_to_move = self._world.our_defender.y - self._world.pitch.height / 2
        if abs(distance_to_move) > DISTANCE_GOAL_THRESHOLD:
            angle = self.get_strafe_angle(distance_to_move)
            self.current_action = move(abs(distance_to_move), angle, strafe_ok=True, backwards_ok=False)
        # are we in the centre?
        if self.current_action == stop():
            # move back to goal line
            distance_to_move = abs(self._world.our_defender.x - self.goal_line)
            angle = -pi
            self.current_action = move(distance_to_move, angle, strafe_ok=False, backwards_ok=True, careful=True)

    def aim_and_pass(self):
        self.current_action = stop()
        angle = self._world.our_defender.get_rotation_to_point(self._world.our_attacker.x,
                                                               self._world.our_attacker.y
                                                               )
        self.current_action = move(0, angle, strafe_ok=False, backwards_ok=False, careful=True)
        if self.current_action == stop() and not self.is_catching:
            self.current_action = open_catcher()
            self._world.our_defender.catcher = "open"
            self.is_kicking = True