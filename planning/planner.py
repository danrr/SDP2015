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
            # if opponent have ball i.e is the ball not visible or in front of opponent - strafe to intercept shot
            # else
                # if ball moving towards us - move to y intercept
                # else move to y coor
            return stop()
        else:
            action = stop()

            # if grabber open - close grabber
            if self._world.our_defender.catcher == "open":
                action = grab_ball_center()
                self._world.our_defender.catcher = "close"

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
        action = stop()
        # if not aligned with the center of the goal - strafe to center
        distance_to_move = self._world.our_defender.y - self._world.pitch.height / 2

        # are we in the centre?
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