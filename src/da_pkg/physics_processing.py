def make_simple_profile(current, target, slop):
    """
    target 값에 서서히 다가가기 위한 전처리 함수

    :param current: 현재 값
    :param target: 목표 값
    :param slop: '서서히'의 최소 범위
    :return: 목표 값에 서서히 다가간 변화값
    """
    if target > current:
        # 목표 값: 증가
        # 변화량이 slop보다 크면 slop만큼만 더해서 리턴, slop보다 작으면 타겟을 그대로 리턴
        return min(target, current + slop)

    elif target < current:
        # 목표 값: 감소
        return max(target, current - slop)
    else:
        return target


def constrain(origin, low, high):
    """Set all the origin values stays in low - high boundary"""
    if origin < low:
        return low
    elif origin > high:
        return high
    return origin


def map_number(origin, min1, max1, min2, max2):
    return ((origin - min1) / (max1 - min1)) * (max2 - min2) + min2
