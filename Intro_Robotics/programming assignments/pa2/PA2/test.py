import math
def getPos(t1, t2, l1, l2):
    x = l1 * math.cos(t1) + l2 * math.cos(t1+t2)
    y = l1 * math.sin(t1) + l2 * math.sin(t1+t2)
    return x, y

def getAngle(x, y, l1, l2):
        t1 = (l1**2 + l2**2 - x**2 - y**2) / (2 * l1 * l2)
        print(t1)
        t2 = (x**2 + y**2 + l1**2 - l2**2) / (2 * l1 * math.sqrt(x**2 + y**2))

        alpha = math.acos( t1 )
        beta = math.acos( t2 )

        print(alpha)

        theta11 = math.atan2(y, x) + beta
        theta12 = math.atan2(y, x) - beta
        theta21 = math.pi + alpha
        theta22 = math.pi - alpha

        return theta11, theta12, theta21, theta22


theta11, theta12, theta21, theta22 = getAngle(18.0, 8.0, 14.0, 10.0)
print("%f %f %f %f" % (math.degrees(theta11), math.degrees(theta12), math.degrees(theta21), math.degrees(theta22)))
print(getPos(theta11, theta21, 14.0, 10.0))
print(getPos(theta12, theta22, 14.0, 10.0))
print(getPos(theta11, theta22, 14.0, 10.0))
print(getPos(theta12, theta21, 14.0, 10.0))