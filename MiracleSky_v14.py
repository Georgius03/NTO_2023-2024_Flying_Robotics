import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from socket import *
import numpy as np

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
        self.ticket = ''

        # For text recognition
        self.replacer = {"l": '1', "s": '5', "S": '5', "u": '0', "U": '0', "z": '2', "Z": '2', "/": "A", " ": "",
                         "\\": "A", "\n": "",
                         "|": '1'}
        self.sharp_filter = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])

        # server connection params
        host = '192.168.11.2'
        port = 12345
        addr = (host, port)

        self.tcp_socket = socket(AF_INET, SOCK_STREAM)
        self.tcp_socket.connect(addr)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = np.zeros((240, 320), dtype=np.uint8)
        mask[30:210, 40:280] = 255
        ret, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
        rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
        dilate = cv2.dilate(thresh, kernel=rect_kernel, iterations=2)
        dilate = cv2.bitwise_and(dilate, mask)

        contours1, hierarchy1 = cv2.findContours(dilate,
                                                 cv2.RETR_EXTERNAL,
                                                 cv2.CHAIN_APPROX_NONE)

        # img2 = img.copy()
        for cnt1 in contours1:
            x1, y1, w1, h1 = cv2.boundingRect(cnt1)
            cropped_thresh = thresh[y1:y1 + h1, x1:x1 + w1]
            cropped_image = img[y1:y1 + h1, x1:x1 + w1]
            if 400 <= cv2.contourArea(cnt1) <= 16000:
                # cv2.rectangle(img2, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 2)

                cropped_thresh = cv2.bitwise_not(cropped_thresh)
                cropped_thresh = cv2.dilate(cropped_thresh, kernel=rect_kernel, iterations=2)
                contours2, hierarchy2 = cv2.findContours(cropped_thresh,
                                                         cv2.RETR_TREE,
                                                         cv2.CHAIN_APPROX_NONE)

                for cnt2 in contours2:
                    if 100 <= cv2.contourArea(cnt2) < 1600:
                        x2, y2, w2, h2 = cv2.boundingRect(cnt2)
                        if ((w2 / h2) >= 1.5) or ((h2 / w2) >= 1.5):
                            cropped_image2 = cropped_image[y2:y2 + h2, x2:x2 + w2]
                            cropped_image2 = cv2.filter2D(cropped_image2, ddepth=-1, kernel=self.sharp_filter)
                            text = pytesseract.image_to_string(cropped_image2)  # получение текста из картинки
                            self.check_and_add(self.text_replace(text))
                            # print(text_replace(text))

    def text_replace(self, text):
        new_text = text[::]
        for symbol in self.replacer:
            new_text = new_text.replace(symbol, self.replacer[symbol])
        return new_text

    def zone_definition(self):
        telem = self.get_telemetry(frame_id='aruco_map')
        if (0 < telem.x < 1.2) and (1 < telem.y < 4.25):
            self.car_zone = 'Zone 1'
        elif (telem.x > 3) and (0 < telem.y < 5.2):
            self.car_zone = 'Zone 2'
        else:
            self.car_zone = 'Road'

    def ticket_check(self, data):
        if data == 'car not found':
            self.ticket = ''
            return
        data_splits = data.split(';')
        print(data_splits)
        time1 = data_splits[0].split(':')
        time_seconds1 = int(time1[0]) * 60 * 60 + int(time1[1]) * 60 + int(time1[0])
        time2 = data_splits[1].split(':')
        time_seconds2 = int(time2[0]) * 60 * 60 + int(time2[1]) * 60 + int(time2[0])
        time_dif = abs(time_seconds2 - time_seconds1)
        if data_splits[2] == 'not paid' and time_dif > 900:
            self.ticket = 'ticket'
        else:
            self.ticket = 'not ticket'

    def check_and_add(self, text):
        if 3 <= len(text) <= 9:
            if self.car_numbers != {}:
                for key, value in list(self.car_numbers.items()):
                    count = 0
                    for symbol in text:
                        if symbol in value:
                            count += 1
                    if count <= 2:
                        self.zone_definition()
                        data = text
                        data = str.encode(data)
                        self.tcp_socket.send(data)
                        data = self.tcp_socket.recv(1024)
                        data = bytes.decode(data)
                        self.ticket_check(data)
                        self.car_numbers[f'{text}'] = self.car_zone + ', ' + self.ticket
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
    velocity = 0.2

    coex.navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    coex.navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    for point in points:
        # print(f'Moving to {point[0]}, {point[1]}, {altitude}')
        coex.navigate_wait(x=point[0], y=point[1], z=altitude, speed=velocity, frame_id='aruco_map')

    # print("Moving to zero point...")
    coex.navigate_wait(x=0, y=0, z=altitude, speed=0.3, frame_id='aruco_map')

    # print("Landing...")
    coex.land_wait()

    for key, value in list(cam.car_numbers.items()):
        print(f'{key}, car on {value}')


if __name__ == '__main__':
    main()
