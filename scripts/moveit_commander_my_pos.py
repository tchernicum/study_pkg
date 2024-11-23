#!/usr/bin/env python3

import rospy  # Импортируем библиотеку ROS для работы с узлами и сообщениями
import moveit_commander  # Импортируем библиотеку MoveIt для управления манипуляторами
import geometry_msgs.msg  # Импортируем сообщения для работы с геометрией (например, позы)
import sys  # Импортируем библиотеку sys для работы с аргументами командной строки
from apriltag_ros.msg import AprilTagDetectionArray

_INITED = None

poses_hand = [
    # Каждая поза состоит из позиции (x, y, z) и ориентации (x, y, z, w)
    #([0, 0, 0], [0, 0, 0, 0]),
]

def init():
    global _INITED
    if (_INITED is not None):
        return
    _INITED = 1
    rospy.init_node('tag_listener')
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback, queue_size=1)
    # Инициализация ROS-узла с именем 'moveit_random_pose'
    rospy.init_node('moveit_random_pose', anonymous=True)
    # Инициализация moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

#def predefined_poses_hand():
#    positions = [
#        ({"panda_finger_joint1": 0}),     # Поза 1
#        ({"panda_finger_joint1": 0.035}), # Поза 2
#    ]
#    return positions # Возвращаем список поз

def predefined_poses_arm():
    positions = [
        ({"panda_joint2": -1.57}),  # Поза 1
        ({"panda_joint2": 0}),      # Поза 2
    ]
    return positions  # Возвращаем список поз

def tag_callback(data):
    print(data)
    #if (type(data.detections) == 'list' and len(data.detections) > 0):
    #    for (pose in data.detections):
    #        if (!poses_hand(pose.id)):
    #            poses_hand(pose.id) = pose

def main():
    # Основная функция, выполняющаяся при запуске программы

    # Создание объекта RobotCommander для управления роботом
    #robot = moveit_commander.RobotCommander()

    # Создание объекта PlanningSceneInterface для работы с окружением
    #scene = moveit_commander.PlanningSceneInterface()

    # Создание объекта MoveGroupCommander для управления манипулятором (в данном случае "panda_arm")
    group_arm = moveit_commander.MoveGroupCommander("panda_arm")

    # Получаем заранее определенные позы
    poses_arm = predefined_poses()

    # Цикл для выполнения заранее определенных поз
    for position, orientation in poses:

        # Создание объекта Pose для установки целевой позы
        pose_target = geometry_msgs.msg.Pose()

        pose_target.position.x = position[0]  # Установка x-координаты
        pose_target.position.y = position[1]  # Установка y-координаты
        pose_target.position.z = position[2]  # Установка z-координаты
        pose_target.orientation.x = orientation[0]  # Установка x компонента ориентации
        pose_target.orientation.y = orientation[1]  # Установка y компонента ориентации
        pose_target.orientation.z = orientation[2]  # Установка z компонента ориентации
        pose_target.orientation.w = orientation[3]  # Установка w компонента ориентации

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
            group_arm.execute(trajectory, wait=True)  # Выполняем движение и ждем его завершения
        else:
            # Если планирование не удалось, выводим предупреждение
            rospy.logwarn(" Планирование не удалось для данной позы.")

        group_arm.clear_pose_target()

        # Опционально: добавление задержки для визуализации
        rospy.sleep(1)  # Задержка в 1 секунду перед следующей итерацией

    # Создание объекта MoveGroupCommander для управления захватом (в данном случае "panda_hand")
    #group_hand = moveit_commander.MoveGroupCommander("panda_hand")

    # Получаем заранее определенные позы
    #poses_hand = predefined_poses_hand()

    # Цикл для выполнения заранее определенных поз
    #for position in poses_hand:
        # Установка целевой позы для манипулятора
        #group_hand.set_joint_value_target(position)

        # Планирование движения к целевой позе
        #plan = group_hand.plan()  # Получаем план

        # Проверка на успешность планирования
        #if isinstance(plan, tuple):  # Если план является кортежем
        #    success = plan[0]  # Получаем статус (успех/неуспех)
        #    trajectory = plan[1]  # Получаем сам план (траекторию)
        #else:
        #    success = True  # Если не кортеж, считаем, что планирование прошло успешно
        #    trajectory = plan  # В противном случае просто берем план

        # Проверяем, есть ли точки в плане
        #if success and len(trajectory.joint_trajectory.points) > 0:  # Если планирование успешно и есть точки
            # Выполнение движения манипулятора по запланированной траектории
        #    group_hand.execute(trajectory, wait=True)  # Выполняем движение и ждем его завершения
        #else:
            # Если планирование не удалось, выводим предупреждение
        #    rospy.logwarn(" Планирование не удалось для данной позы.")

        # Опционально: добавление задержки для визуализации
        #rospy.sleep(1)  # Задержка в 1 секунду перед следующей итерацией

    rospy.spin()

    # Завершение работы moveit_commander
    #moveit_commander.roscpp_shutdown()  # Остановка работы MoveIt

# Проверяем, является ли данный файл основным модулем
if __name__ == "__main__":
    #if (not __INITED):
    init()

    main()  # Запускаем основную функцию
