from models import *
from collisions import *
from strategies import *
from utilities import *

GOAL_ALIGN_OFFSET = 60


class Planner:
    def __init__(self, our_side, pitch_num):
        self._world = World(our_side, pitch_num)
        # measurement used 1cm = 2.28px
        self._world.our_defender.catcher_area = {'width': 35, 'height': 20, 'front_offset': 18}  # 10
        self._world.our_attacker.catcher_area = {'width': 35, 'height': 20, 'front_offset': 18}

        self.state = None
        self.goal_line = self._world.our_goal.x + GOAL_ALIGN_OFFSET * -1 if self._world._our_side == "left" else 1

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    def plan(self):
        # move flood stuff into here using self.state

        # if ball is in our zone:
        if self._world.pitch.zones[self._world.our_defender.zone].isInside(self._world.ball.x, self._world.ball.y):
            # if grabber closed - open grabber
            # if we don't have ball
                # if ball in our grabber area - stop and grab
                # if ball not moving
                    # if close to ball - forward slow
                    # if far from ball - increase speed
                # if ball moving
                    # try and intercept?
            # else pass?
            return stop()

        # if ball is in enemy attacker zone, panic:
        elif self._world.pitch.zones[self._world.their_attacker.zone].isInside(self._world.ball.x, self._world.ball.y):
            # TODO: extract into method (2)
            # if not facing forward - turn to face forward
            angle = 180 * self._world.our_defender.get_rotation_to_point(
                self._world.pitch.width / 2, self._world.our_defender.y
                ) / (2 * math.pi)
            if abs(angle) > 2:
                return move(0, angle)

            predicted_y = None
            # if the ball is moving fast, attempt to move to intercept ball
            if self._world.ball.velocity > BALL_VELOCITY:
                predicted_y = predict_y_intersection(self._world,
                                                     self._world.our_defender.x,
                                                     self._world.ball,
                                                     bounce=False)

            # TODO: better way of determining if attacker has ball (is ball invisible or in front of attacker maybe)
            # if ball is moving slowly or not towards us attempt to intercept shot
            elif predicted_y is None:
                predicted_y = predict_y_intersection(self._world,
                                                     self._world.our_defender.x,
                                                     self._world.their_attacker,
                                                     bounce=False)
            # if we can intercept
            if predicted_y:
                distance_to_move = self._world.our_defender.y - predicted_y
                # TODO: extract into method (1)
                if distance_to_move > DISTANCE_MATCH_THRESHOLD:
                    angle = pi/2 if distance_to_move > 0 else -pi/2
                    return move(distance_to_move, angle, strafe_ok=True, backwards_ok=False)
            # move to center of goal
            return self.align_to_goal()
        else:
            return self.reset_to_goal()

    def reset_to_goal(self):
        """
        Resets robot to face opponent attacker and and move to the center of the goal
        """
        action = stop()

        # if grabber open - close grabber
        if self._world.our_defender.catcher == "open":
            action = grab_ball_center()
            self._world.our_defender.catcher = "close"

        # TODO: extract into method (2)
        # if not facing forward - turn to face forward
        angle = 180 * self._world.our_defender.get_rotation_to_point(
            self._world.pitch.width / 2, self._world.our_defender.y
            ) / (2 * math.pi)
        if abs(angle) > 2:
            action = move(0, angle, grabber=action["grabber"])

        # do both previous actions at once
        # if neither of those actions need to be done
        if action == stop():
            action = self.align_to_goal()
        return action

    def align_to_goal(self):
        """
        Does the actual moving to the goal line
        """
        action = stop()
        # if not aligned with the center of the goal - strafe to center
        distance_to_move = self._world.our_defender.y - self._world.pitch.height / 2

        # are we in the centre?#
        # TODO: extract into method (1)
        if distance_to_move > DISTANCE_MATCH_THRESHOLD:
            angle = pi/2 if distance_to_move > 0 else -pi/2
            action = move(distance_to_move, angle, strafe_ok=True, backwards_ok=False)
        else:
            # move back to goal line
            distance_to_move = self._world.our_defender - self.goal_line
            if distance_to_move > DISTANCE_MATCH_THRESHOLD:
                angle = pi
                action = move(distance_to_move, angle, strafe_ok=False, backwards_ok=True)
        return action