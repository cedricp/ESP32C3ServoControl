import math
import struct

def fast_inv_sqrtf(number: float) -> float:
    if number <= 0.0:
        return 0.0

    threehalfs = 1.5
    x2 = number * 0.5
    
    # 1. Cast "reinterpret" float32 vers uint32
    # 'f' = float 32 bits, 'I' = unsigned int 32 bits
    i = struct.unpack('I', struct.pack('f', number))[0]
    
    # 2. Le bit-hack magique (Quake III 0x5f3759df)
    i = 0x5f3759df - (i >> 1)
    
    # 3. Cast "reinterpret" uint32 vers float32
    y = struct.unpack('f', struct.pack('I', i))[0]
    
    # 4. 1ère itération de Newton-Raphson (précision ~0.1%)
    y = y * (threehalfs - (x2 * y * y))
    
    # 5. 2ème itération optionnelle pour plus de précision (erreur < 0.001%)
    # y = y * (threehalfs - (x2 * y * y))
    
    return y

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


# for i in range(0,10):
#     a = fast_atan2(1,i)
#     b = math.atan2(1,i)
#     print(abs(a-b))

for i in range (1, 100):
    a = fast_inv_sqrtf(i)
    b = 1.0 / math.sqrt(i)
    print(a,b)