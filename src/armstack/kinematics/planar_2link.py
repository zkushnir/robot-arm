import math

class IKResult:
    def __init__(self, ok: bool, theta1_rad: float = 0.0, theta2_rad: float = 0.0, message: str = ""):
        self.ok = ok
        self.theta1_rad = theta1_rad
        self.theta2_rad = theta2_rad
        self.message = message

def ik_2link_planar(x: float, y: float, L1: float, L2: float, elbow_up: bool = True) -> IKResult:
    r2 = x*x + y*y
    r = math.sqrt(r2)

    if r > (L1 + L2) + 1e-9:
        return IKResult(False, message="Target out of reach (too far).")
    if r < abs(L1 - L2) - 1e-9:
        return IKResult(False, message="Target out of reach (too close).")

    c2 = (r2 - L1*L1 - L2*L2) / (2.0 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    s2 = math.sqrt(max(0.0, 1.0 - c2*c2))
    if not elbow_up:
        s2 = -s2

    theta2 = math.atan2(s2, c2)

    k1 = L1 + L2 * c2
    k2 = L2 * s2
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    return IKResult(True, theta1, theta2, "OK")
