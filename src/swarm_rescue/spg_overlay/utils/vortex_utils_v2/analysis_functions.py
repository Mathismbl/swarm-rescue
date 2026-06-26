import math
import numpy

from spg_overlay.utils.utils import normalize_angle

#Calcul raie-angle
def FromRayToAngle(ray, sensor_angle, mirror = False):
    if mirror:
        return(sensor_angle[ray - 1] + 2*math.pi)
    else:
        return(sensor_angle[ray - 1])

def FromAngleToRay(alpha, sensor_angle):
    frame1 = None
    frame2 = None
    for i, angle in enumerate(sensor_angle):
        if alpha == angle:
            return(i + 1)
        if i == len(sensor_angle) - 1:
            return(i + 1)
        elif alpha > angle and alpha < sensor_angle[i+1] :
            frame1 = (i + 1, angle)
            frame2 = (i + 2, sensor_angle[i+1])
            delta1 = abs(alpha - frame1[1])
            delta2 = abs(alpha - frame2[1])
            if delta1 < delta2:
                return(frame1[0])
            else:
                return(frame2[0])

def AngleBetweenRay(ray_1, ray_2, ray_angles, mirror = False):

    if not mirror:
        angle_ray_1 = FromRayToAngle(ray_1, ray_angles, False)
        angle_ray_2 = FromRayToAngle(ray_2, ray_angles, False)
        return abs(normalize_angle(angle_ray_1 - angle_ray_2))
    else:
        angle_ray_1 = FromRayToAngle(ray_1, ray_angles, False)
        angle_ray_2 = FromRayToAngle(ray_2, ray_angles, True)
        return abs(angle_ray_1 - angle_ray_2)
    
def AngleBetweenDir(dir_1, dir_2):
    return abs(normalize_angle(dir_1 - dir_2))