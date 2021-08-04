def make_simple_profile(current, target, slop):
    """
    current : control_vel
    target : target_vel
    ??? : For more subtle control?
    """
    if target > current:
        # target is bigger than current
        current = min(target, current + slop)

    elif target < current:
        # target is smaller than current
        current = max(target, current - slop)
    else:
        current = target

    return current


def constrain(origin, low, high):
    """Set all the origin values stays in low - high boundary"""
    if origin < low:
        return low
    elif origin > high:
        return high
    return origin
