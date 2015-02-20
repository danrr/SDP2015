from models import *
from collisions import *
from strategies import *
from utilities import *


class Planner:
    def __init__(self, our_side, pitch_num):
        self._world = World(our_side, pitch_num)
        # measurement used 1cm = 2.28px
        self._world.our_defender.catcher_area = {'width': 35, 'height': 20, 'front_offset': 18}  #10
        self._world.our_attacker.catcher_area = {'width': 35, 'height': 20, 'front_offset': 18}

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    def plan(self):
        our_defender = self._world.our_defender
        our_attacker = self._world.our_attacker
        their_defender = self._world.their_defender
        their_attacker = self._world.their_attacker
        ball = self._world.ball

        # take states into account
        # if ball is in our zone:
        if self._world.pitch.zones[our_defender.zone].isInside(ball.x, ball.y):
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
        elif self._world.pitch.zones[their_attacker.zone].isInside(ball.x, ball.y):
            # if opponent have ball i.e is the ball not visible or in front of opponent - strafe to intercept shot
            # else
                # if ball moving towards us - move to y intercept
                # else move to y coor
            return stop()
        else:
            # do both at once
                # if grabber open - close grabber
                # if not facing forward - turn to face forward
            # if not aligned with the center of the goal - strafe to center
            # if too far forward - move back
            return stop()