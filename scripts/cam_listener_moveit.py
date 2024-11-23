#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import sqrt
import rospy  # Импортируем библиотеку ROS для работы с узлами и сообщениями
import moveit_commander  # Импортируем библиотеку MoveIt для управления манипуляторами
import geometry_msgs.msg  # Импортируем сообщения для работы с геометрией (например, позы)
import sys  # Импортируем библиотеку sys для работы с аргументами командной строки
from apriltag_ros.msg import AprilTagDetectionArray

err_pos_max    = 0.03
err_orient_max = 0.03

poses = [
    geometry_msgs.msg.Pose(),
    geometry_msgs.msg.Pose(),
    geometry_msgs.msg.Pose(),
    geometry_msgs.msg.Pose(),
    geometry_msgs.msg.Pose(),
    geometry_msgs.msg.Pose(),
]

def robot_planner():
    # Создание объекта MoveGroupCommander для управления манипулятором (в данном случае "panda_arm")
    group_arm = moveit_commander.MoveGroupCommander("panda_arm")

    # Цикл для выполнения заранее определенных поз
    for pose_target in poses:

        # Установка целевой позы для манипулятора
        group_arm.set_pose_target(pose_target)

        # Планирование движения к целевой позе
        plan = group_arm.plan()  # Получаем план

        # Проверка на успешность планирования
        if isinstance(plan, tuple):  # Если план является кортежем
            success = plan[0]  # Получаем статус (успех/неуспех)
            trajectory = plan[1]  # Получаем сам план (траекторию)
        else:
            success = True  # Если не кортеж, считаем, что планирование прошло успешно
            trajectory = plan  # В противном случае просто берем план

        # Проверяем, есть ли точки в плане
        if success and len(trajectory.joint_trajectory.points) > 0:  # Если планирование успешно и есть точки
            # Выполнение движения манипулятора по запланированной траектории
            #group_arm.execute(trajectory, wait=True)  # Выполняем движение и ждем его завершения
            rospy.logwarn(" Планирование удалось")
        else:
            # Если планирование не удалось, выводим предупреждение
            rospy.logwarn(" Планирование не удалось для данной позы.")

        #group_arm.clear_pose_target()

        # Опционально: добавление задержки для визуализации
        rospy.sleep(1)  # Задержка в 1 секунду перед следующей итерацией

def check_pose_changed(poseid, pose_new):
    if (not poses[poseid]):
        return True
    pose_old   = poses[poseid]
    err_pos    = map(lambda x: abs(getattr(pose_old.position,x)-getattr(pose_new.position,x)),      ['x','y','z'])
    err_orient = map(lambda x: abs(getattr(pose_old.orientation,x)-getattr(pose_new.orientation,x)), ['x','y','z','w'])
    if max(err_pos) >= err_pos_max:
        return True
    if max(err_orient) >= err_orient_max:
        return True
    return False

def tag_callback(detection_msg):
    print(poses)
    for det in detection_msg.detections:
        print(f"pose id {det.id[0]} found pose {det.pose.pose}")
        pose_id = det.id[0]
        if check_pose_changed(pose_id, det.pose.pose.pose):
            poses[pose_id] = det.pose.pose.pose
            robot_planner()

#rospy.init_node('tag_listener')
rospy.init_node('moveit_random_pose', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback, queue_size=1)
rospy.spin()
