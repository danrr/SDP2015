from math import tan, pi, hypot


def is_shot_blocked(world):
    """
    Checks if our robot could shoot past their robot
    """
    predicted_y = predict_y_intersection(
        world,  world.their_attacker.x, world.our_defender, full_width=False, bounce=True)
    if predicted_y is None:
        return True
    return abs(predicted_y - world.their_attacker.y) < world.their_attacker.length


def predict_y_intersection(world, predict_for_x, robot, full_width=False, bounce=False):
    """
    Predicts the (x, y) coordinates of the ball shot by the robot
    Corrects them if it's out of the bottom_y - top_y range.
    If bounce is set to True, predicts for a bounced shot
    Returns None if the robot is facing the wrong direction.
    """
    x = robot.x
    y = robot.y
    top_y = world._pitch.height - 60 if full_width else world.our_goal.y + (world.our_goal.width / 2) - 30
    bottom_y = 60 if full_width else world.our_goal.y - (world.our_goal.width / 2) + 30
    angle = robot.angle
    if (robot.x < predict_for_x and not (pi / 2 < angle < 3 * pi / 2)) or \
            (robot.x > predict_for_x and (3 * pi / 2 > angle > pi / 2)):
        if bounce:
            if not (0 <= (y + tan(angle) * (predict_for_x - x)) <= world._pitch.height):
                bounce_pos = 'top' if (y + tan(angle) * (predict_for_x - x)) > world._pitch.height else 'bottom'
                x += (world._pitch.height - y) / tan(angle) if bounce_pos == 'top' else (0 - y) / tan(angle)
                y = world._pitch.height if bounce_pos == 'top' else 0
                angle = (-angle) % (2 * pi)
        predicted_y = (y + tan(angle) * (predict_for_x - x))
        # Correcting the y coordinate to the closest y coordinate on the goal line:
        if predicted_y > top_y:
            return top_y
        elif predicted_y < bottom_y:
            return bottom_y
        return predicted_y
    else:
        return None
