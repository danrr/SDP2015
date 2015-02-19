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

        self._attacker_strategies = {'grab': [AttackerGrab],
                                     'shoot': [AttackerShoot]}

        self._defender_strategies = {'defence': [DefenderDefence],
                                     'grab': [DefenderGrab],
                                     'pass': [DefenderPass]}

        self._defender_state = 'defence'
        self._defender_current_strategy = self.choose_defender_strategy(self._world)

        self._attacker_state = 'grab'
        self._attacker_current_strategy = self.choose_attacker_strategy(self._world)

    # Provisional. Choose the first strategy in the applicable list.
    def choose_attacker_strategy(self, world):
        next_strategy = self._attacker_strategies[self._attacker_state][0]
        return next_strategy(world)

    # Provisional. Choose the first strategy in the applicable list.
    def choose_defender_strategy(self, world):
        next_strategy = self._defender_strategies[self._defender_state][0]
        return next_strategy(world)

    @property
    def attacker_strat_state(self):
        return self._attacker_current_strategy.current_state

    @property
    def defender_strat_state(self):
        return self._defender_current_strategy.current_state

    @property
    def attacker_state(self):
        return self._attacker_state

    @attacker_state.setter
    def attacker_state(self, new_state):
        assert new_state in ['defence', 'attack']
        self._attacker_state = new_state

    @property
    def defender_state(self):
        return self._defender_state

    @defender_state.setter
    def defender_state(self, new_state):
        assert new_state in ['defence', 'attack']
        self._defender_state = new_state

    def update_world(self, position_dictionary):
        self._world.update_positions(position_dictionary)

    def plan(self):
        our_defender = self._world.our_defender
        our_attacker = self._world.our_attacker
        their_defender = self._world.their_defender
        their_attacker = self._world.their_attacker
        ball = self._world.ball

        ## If ball is in our attacker zone, then grab the ball and shoot:
        if self._world.pitch.zones[our_attacker.zone].isInside(ball.x, ball.y) or \
                        self._attacker_current_strategy.current_state in ['AIM', 'OPEN', 'SHOOT']:
            print self._attacker_current_strategy.current_state

            # Check if we should switch from a grabbing to a scoring strategy.
            if self._attacker_state == 'grab' and self._attacker_current_strategy.current_state == 'GRABBED':
                print "[ATTACKER]: we grabbed the ball so go ahead and shoot"
                self._attacker_state = 'shoot'
                self._attacker_current_strategy = AttackerShoot(self._world)

            # If we're not ready to change to scoring stay in 'grab'
            elif self._attacker_state == 'grab':
                # Switch to careful mode if the ball is too close to the wall.

                # print "[ATTACKER]: grab"
                self._attacker_current_strategy = AttackerGrab(self._world)

            #If we've finished shooting
            elif self._attacker_state == 'shoot' and self._attacker_current_strategy.current_state == 'FINISHED':
                self._attacker_state = 'grab'
            #     self._attacker_current_strategy = AttackerGrab(self._world)
            #     print "[ATTACKER]: we finished kicking the ball"

            # elif self._attacker_state == 'shoot' and\
            #         not our_attacker.can_catch_ball(self._world.ball):
            #     self._attacker_state = 'grab'
            #     self._attacker_current_strategy = AttackerGrab(self._world)

            return self._attacker_current_strategy.generate()

        else:
            # print "[ATTACKER]: motor speed on 0"
            # print our_attacker.catcher
            if our_attacker.catcher == 'open':
                our_attacker.catcher = 'closed'
                return grab_ball_center()
            else:
                return do_nothing()
