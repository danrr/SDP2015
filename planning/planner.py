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
        self._world.our_defender.catcher_area = {'width': 35, 'height': 20, 'front_offset': 23}  # 10
        self._world.our_attacker.catcher_area = {'width': 35, 'height': 20, 'front_offset': 18}

        self.is_kicking = False
        self.time = None
        self.old_action = stop()
        self.current_action = stop()
        self.goal_line = self._world.our_goal.x + GOAL_ALIGN_OFFSET * 1 if self._world._our_side == "left" else -1

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    def plan(self):
        # TODO: maybe move flood stuff into here using self.state
        # TODO: take into account where ball was seen last when it's hidden?

        # if ball is in our zone:
        if self._world.pitch.zones[self._world.our_defender.zone].isInside(self._world.ball.x, self._world.ball.y):
            self.current_action = stop()

            # if self.is_kicking:
            #     self.is_kicking = False
            #     self.current_action = kick_ball(100)
            # else:
            #
            #     # if grabber closed - open grabber
            #     if self._world.our_defender.catcher == "closed" and not self._world.our_defender.has_ball(self._world.ball):
            #         self._world.our_defender.catcher = "open"
            #         self.current_action = open_catcher()
            #
            #     if self._world.our_defender.has_ball(self._world.ball):
            #         # TODO: implement passing
            #         self.current_action = open_catcher()
            #         self._world.our_defender.catcher = "open"
            #         self.is_kicking = True
            #     else:
            #         # if ball in our grabber area - stop and grab
            #         if self._world.our_defender.can_catch_ball(self._world.ball):
            #             # TODO: make grabbers close depending on ball position
            #             self._world.our_defender.catcher = "closed"
            #             self.current_action = grab_ball_center()
                #
                #     # if ball moving fast
                #     elif self._world.ball.velocity > BALL_VELOCITY:
                #         # try and intercept
                #         predicted_y = predict_y_intersection(self._world,
                #                                              self._world.our_defender.x,
                #                                              self._world.ball,
                #                                              bounce=False)
                #         if predicted_y:
                #             distance_to_move = self._world.our_defender.y - predicted_y
                #             angle = pi/2 if distance_to_move > 0 else -pi/2
                #             self.current_action = move(distance_to_move, angle, strafe_ok=True, backwards_ok=False)
                #         if not predicted_y:
                #             self.go_to_ball()
                #     else:
                #         self.go_to_ball()

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
                                                         bounce=False)

                # TODO: better way of determining if attacker has ball (is ball invisible or in front of attacker maybe)
                # if ball is moving slowly or not towards us attempt to intercept shot
                if self._world.ball.velocity <= BALL_VELOCITY or predicted_y is None:
                    predicted_y = predict_y_intersection(self._world,
                                                         self._world.our_defender.x,
                                                         self._world.their_attacker,
                                                         bounce=False)
                # if we can intercept
                if predicted_y:
                    distance_to_move = self._world.our_defender.y - predicted_y
                    angle = pi/2 if distance_to_move > 0 else -pi/2
                    self.current_action = move(abs(distance_to_move), angle, strafe_ok=True, backwards_ok=False)
            if self.current_action == stop():
                # move to center of goal if there is nothing to do
                self.align_to_goal()
        else:
            self.reset_to_goal()

        if self.time and self.time + 0.5 < time.clock():
            self.time = None
            self.old_action = {}
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
            # if ball far away move fast, else move slow
            self.current_action = move(distance_to_ball, 0, careful=distance_to_ball > 5 * DISTANCE_MATCH_THRESHOLD)

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
        self.current_action = move(0, angle, grabber=self.current_action["grabber"], careful=True)

    def align_to_goal(self):
        """
        Returns actions that move a robot facing the opponent to the goal line
        """
        # if not aligned with the center of the goal - strafe to center
        distance_to_move = self._world.our_defender.y - self._world.pitch.height / 2
        if abs(distance_to_move) > DISTANCE_GOAL_THRESHOLD:
            angle = pi/2 if distance_to_move > 0 else -pi/2
            self.current_action = move(abs(distance_to_move), angle, strafe_ok=True, backwards_ok=False)
        # are we in the centre?
        if self.current_action == stop():
            # move back to goal line
            distance_to_move = self._world.our_defender.x - self.goal_line
            angle = -pi
            self.current_action = move(distance_to_move, angle, strafe_ok=False, backwards_ok=True, careful=True)
