import math

def fast_atan2(y: float, x: float) -> float:
    """
    Approximation polynomiale rapide de atan2(y, x) en radians.
    Erreur maximale < 0.005 rad (~0.28°).
    """
    if x == 0.0 and y == 0.0:
        return 0.0

    abs_y = abs(y) + 1e-10  # Évite la division par zéro

    if x >= 0.0:
        r = (x - abs_y) / (x + abs_y)
        angle = 0.1963 * (r ** 3) - 0.9817 * r + 0.7853981633974483  # pi / 4
    else:
        r = (x + abs_y) / (abs_y - x)
        angle = 0.1963 * (r ** 3) - 0.9817 * r + 2.356194490192345  # 3 * pi / 4

    return -angle if y < 0.0 else angle


for i in range(0,10):
    a = fast_atan2(1,i)
    b = math.atan2(1,i)
    print(abs(a-b))