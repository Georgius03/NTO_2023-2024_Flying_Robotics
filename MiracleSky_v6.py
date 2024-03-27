import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import math
import pytesseract


class COEX:
    def __init__(self):
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)
        self.SetLEDEffect = rospy.ServiceProxy('SetLEDEffect', srv.SetLEDEffect)

    def navigate_wait(self, x=0, y=0, z=0, yaw=float('nan'), speed=0.1, frame_id='', tolerance=0.1, auto_arm=False):
        res = self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        if not res.success:
            return res

        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                return res
            rospy.sleep(0.2)

    def land_wait(self):
        self.land()
        while self.get_telemetry().armed:
            rospy.sleep(0.2)


class CameraStream:
    def __init__(self):
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.stream_sub = rospy.Subscriber('main_camera/image_raw',
                                           Image,
                                           self.image_callback,
                                           queue_size=1)
        self.bridge = CvBridge()
        self.car_numbers = {}
        self.car_count = 1
        self.car_zone = ''

        self.hor = ((60, 180, 40, 200),
                    (60, 180, 80, 240),
                    (60, 180, 120, 280),
                    (30, 150, 80, 240),
                    (90, 210, 80, 240))

        self.ver = ((40, 200, 70, 190),
                    (40, 200, 100, 220),
                    (40, 200, 130, 250))

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.zone_definition()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        for window in self.hor:
            cropped_image = img[window[0]:window[1], window[2]:window[3]]
            text = pytesseract.image_to_string(cropped_image)  # получение текста из картинки
            text = self.text_replace(text)  # фильтр
            self.check_and_add(text)

        for window in self.ver:
            cropped_image = img[window[0]:window[1], window[2]:window[3]]
            text = pytesseract.image_to_string(cropped_image)  # получение текста из картинки
            text = self.text_replace(text)  # фильтр
            self.check_and_add(text)

        # print(text)

    def text_replace(self, text):
        new_text = text.replace(" ", "")
        new_text = new_text.replace("\\", "")
        new_text = new_text.replace("/", "")
        new_text = new_text.replace("\n", "")
        return new_text

    def zone_definition(self):
        telem = self.get_telemetry(frame_id='aruco_map')
        if (0 < telem.x < 1.2) and (1 < telem.y < 4.25):
            self.car_zone = 'Zone 1'
        elif (telem.x > 4.9) and (0 < telem.y < 5.2):
            self.car_zone = 'Zone 2'
        else:
            self.car_zone = 'Road'

    def check_and_add(self, text):
        if 3 <= len(text) <= 9:
            if self.car_numbers != {}:
                for key, value in list(self.car_numbers.items()):
                    count = 0
                    for symbol in text:
                        if symbol in value:
                            count += 1
                    if count <= 2:
                        self.car_numbers[f'{text}'] = self.car_zone
                        self.car_count += 1
            else:
                self.car_numbers[f'Car{text}'] = self.car_zone
                self.car_count += 1


def main():
    rospy.init_node('flight', disable_signals=True)
    coex = COEX()
    cam = CameraStream()

    points = [[0, 0.9], [0, 1.8], [0, 2.7], [0, 3.6], [0, 4.5], [0, 5.2],
              [0.9, 5.2], [0.9, 4.5], [0.9, 3.6], [0.9, 2.7], [0.9, 1.8], [0.9, 0.9], [0.9, 0],
              [1.8, 0], [1.8, 0.9], [1.8, 1.8], [1.8, 2.7], [1.8, 3.6], [1.8, 4.5], [1.8, 5.2],
              [2.7, 5.2], [2.7, 4.5], [2.7, 3.6], [2.7, 2.7], [2.7, 1.8], [2.7, 0.9], [2.7, 0],
              [3.4, 0], [3.4, 0.9], [3.4, 1.8], [3.4, 2.7], [3.4, 3.6], [3.4, 4.5], [3.4, 5.2], ]

    altitude = 1.5
    velocity = 0.15

    coex.navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)

    for point in points:
        # print(f'Moving to {point[0]}, {point[1]}, {altitude}')
        coex.navigate_wait(x=point[0], y=point[1], z=altitude, speed=velocity, frame_id='aruco_map')

    # print("Moving to zero point...")
    coex.navigate_wait(x=0, y=0, z=altitude, speed=0.3, frame_id='aruco_map')

    # print("Landing...")
    coex.land_wait()

    for key, value in list(cam.car_numbers.items()):
        print(f'{key}, car on {value}', end=', ')


if __name__ == '__main__':
    main()
