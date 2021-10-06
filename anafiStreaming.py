#!/usr/bin/env python

# NOTE: Line numbers of this example are referenced in the user guide.
# Don't forget to update the user guide after every modification of this example.
# source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell

import csv
import cv2
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import traceback
from anafiRequestPost import Anafi_Request_Post
from anafiScanning import Anafi_Scanning
import time
from pyzbar import pyzbar
import keyboard 

import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingEvent import moveByEnd
from olympe.messages.ardrone3.Piloting import moveBy, CancelMoveBy
from olympe.messages.ardrone3.Piloting import moveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AltitudeChanged, GpsLocationChanged, PositionChanged, moveByChanged, AltitudeAboveGroundChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.move import extended_move_by
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

DRONE_IP = "192.168.42.1"
# DRONE_IP = "10.202.0.1"
CONTROLLER_IP = "192.168.53.1"


class AnafiConnection(threading.Thread):

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            DRONE_IP, drone_type=od.ARSDK_DEVICE_TYPE_ANAFI4K)

        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()

         # initialize the known distance from the camera to the object (referring to picture)
        self.KNOWN_DISTANCE = 25.1

        # initialize the known object width, which in this case, the barcode(referring to QR code in picture)
        self.KNOWN_WIDTH = 2.16535

        # change path to your own path
        self.image = cv2.imread("/home/dragonfly/aina/physical_parrot/images/barcode.jpg")

        # decode the QR code in the picture
        self.foundBarcode = pyzbar.decode(self.image)

        # loop the found barcode and get the width (in pixels) for the QR code in the picture, then calculate the focal length
        for QRCode in self.foundBarcode:
            (x, y, w, h) = QRCode.rect
        self.focalLength = (w * self.KNOWN_DISTANCE) / self.KNOWN_WIDTH

        self.request_post = Anafi_Request_Post()
        self.scanning_decode = Anafi_Scanning()

         # comment/uncomment this part if want to read location from txt file
        self.listOfLocation = self.request_post.readLocation()
        # print(self.listOfLocation)
        
        # flag for the current location and status
        self.currentLocation = None
        self.currentLocationStatus = False

         # initializing the barcode data list for storing the list of barcode that will be scanned later
        self.barcodeDataList = []

        # initializing for forward and backward movement to avoid crash
        self.fbRange = [70000, 90000]
        self.fb = 0
        # self.inst = ["UP", "DOWN", "LEFT", "RIGHT", "TAKEOFF"]
        self.barcodeData = ""
        self.moveup = 0
        self.i = 0
        self.inst = "NONE"
        self.rackInst = ["UP", "DOWN", "RIGHT", "LEFT"]
        self.inst1 = ""
        

        #for distance x and distance y for scanning area
        self.final_dX = 0.0
        self.final_dY = 0.0
        
        # self.dY_1 = 100
        # self.dX_2 = 100
        # self.dY_2 = 100
        # self.dX_3 = 100
        # self.dY_3 = 100
        # self.dX_4 = 100
        # self.dY_4 = 100
        # self.dX_5 = 100
        # self.dY_5 = 100
        self.limitDistance = 1

        self.dY_1 = 0
        self.dX_2 = 0
        self.dY_2 = 1.5
        self.dX_3 = 1.5
        self.dY_3 = 1.5
        self.dX_4 = 0
        self.dY_4 = 0
        self.dX_5 = -1.5
        self.dY_5 = 1.5
        self.droneOperation = 1
        self.itemlocation = ""
        

        super().__init__()
        print("Initialization succesfull, Drone is ready to FLY")
        super().start()

    # connect olympe with the drone and setup callback functions for the olympe SDK
    def start(self):
        # Connect the the drone
        self.drone.connect()
        print("**----------> Drone Connected")
        print("**----------> Battery percentage before takeoff:", self.drone.get_state(BatteryStateChanged)["percent"])
        print("**---------->Altitude before takeoff(should be 0):", self.drone.get_state(AltitudeChanged)["altitude"])

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()

    # Properly stop the video stream and disconnect
    def stop(self):
        self.drone.stop_video_streaming()
        self.drone.disconnect()
        self.h264_stats_file.close()

    # This function will be called by Olympe for each decoded YUV frame.
    def yuv_frame_cb(self, yuv_frame):
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    # This function will be called by Olympe to flush the callback
    def flush_cb(self):
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    # This function is necessary for Olympe SDK
    def start_cb(self):
        pass

    # This function is necessary for Olympe SDK
    def end_cb(self):
        pass

    # This function will be called by Olympe for each new h264 frame.
    def h264_frame_cb(self, h264_frame):

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

    def show_yuv_frame(self, window_name, yuv_frame):
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(
            yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        # scan the barcode, draw box and data in the frame
        # self.barcodeData = self.scanning_decode.startScanning(self.cv2frame, self.focalLength, self.KNOWN_WIDTH)
        self.barcodeInfo  = self.scanning_decode.startScanning(cv2frame, self.focalLength, self.KNOWN_WIDTH)
 
        if not self.barcodeInfo:
            pass

        else:
        #  str(self.barcodeInfo) != "None":
            # print("Area", self.barcodeInfo[1][1])
            # forward backward to avoid crash
            self.fb = self.avoidCrash(self.barcodeInfo[1])
            # print(self.fb)
            
            self.barcodeData1 = str(self.barcodeInfo[0])
            # print(self.barcodeData)

            # to read the tags only once at a time 
            if self.barcodeData1 != self.barcodeData:
                self.barcodeData = str(self.barcodeInfo[0])
                codeInfo= str(self.barcodeData).split()
                if len(codeInfo) > 1: 
                    self.storeData(codeInfo[0])
                    self.inst = codeInfo[1]
                    # self.automove(codeInfo[1], self.fb)
                else:
                    self.storeData(codeInfo[0])

        # Use OpenCV to show this frame
        cv2.imshow(window_name, cv2frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...


    def scanningItemMovement(self, count, y, x):
        # upmovement, up, self.final_dX
        # count = upmovement
        count2 = count
        # count = count
        print("**----------> distance x that drone should travel: ", x)
        print("**----------> distance y that drone shoud travel: ", y)

        if count2%2 == 0:
            countType = "Even Number"
            print(countType)

            while count > 0:
                # print(count)
                print("count is: ", count)
                self.cancelmove()
                self.move_Up2(y)

                if count%2 == 0:
                    self.cancelmove()
                    # print("masuk sini")
                    self.move_Right2(x)
                    # count = count-1
                    # print("tolak satu")

                elif count%2 == 1:
                    self.cancelmove()
                    self.move_Left2(x)
                    # count = count-1

                count = count-1

                # else:
                #     print("asal tak turun")

            else:
                print("done")
                self.final_dY = -10

        elif count2 %2 == 1:
            countType = "Odd Number"
            print(countType)

            while count > 0:
                print("count is: ", count)
                self.cancelmove()
                self.move_Up2(y)

                if count%2 == 0:
                    self.cancelmove()
                    # print("masuk sini")
                    self.move_Left2(x)
                    # count = count-1

                elif count%2 == 1:
                    self.cancelmove()
                    self.move_Right2(x)
                    # count = count-1
                
                count = count-1
                # else:
                    # print("asal tak turun")
                    
            else:
                self.final_dY = -10

            # else:
            #     print("errorrrr")
        
        # return
    
    def scanItem(self):

        if (self.dX_2 != 100) and (self.dX_3 != 100) and (self.dX_4 != 100) and (self.dX_5 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100):

            if self.i == 0:
                print("******Enter scanning items operation******")
                
                dXX = (self.dX_3 - self.dX_5)/2
                dYY = ((self.dY_2 + self.dY_3)/2) - ((self.dY_1 + self.dY_4)/2)
                # dX = math.ceil((dXX)*100)/100
                # dY = math.ceil((dYY)*100)/100
                dX = round(dXX, 1)
                dY = round(dYY, 1)
                
                self.final_dX = dX
                self.final_dY = dY
                print("**----------> Final dX: ",self.final_dX)
                print("**----------> Final dY: ", self.final_dY)

                x = self.final_dY/0.5
                upmovement = round(x)
                print("**----------> Movement drone should move up to: ", upmovement)
                # up = math.ceil((self.final_dY/upmovement)*100)/100
                up = round((self.final_dY/upmovement), 1)

                self.scanningItemMovement(upmovement, up, self.final_dX)
 

            # else:
                # print("lebih satu dah")

            self.i = self.i + 1

        if (self.final_dY == -10):
            print("***************End scanning*******************")
            self.land()
            self.stop()

    def avoidCrash(self, info):
        area = info[1]

        if area > self.fbRange[0] and area > self.fbRange[1]:
            fb = 0
        elif area > self.fbRange[1]:
            fb = -0.2
        elif area < self.fbRange[0] and area != 0:
            fb = 0.2

        return fb

    def storeData(self, barcodeData):
        # condition to check the barcode that have been scanned is an item ID or location ID
        # because of the library keep scanning and decode the frame, we need to set a condition if there is no QR code in the frame, just pass
        # if not self.barcodeData:
        #     pass
        # elif
        # print("barcode: " + barcodeData)
        if (barcodeData in self.listOfLocation):
            self.currentLocation = barcodeData
            self.currentLocationStatus = True
            print("*******Location status true detected******")
            print(self.barcodeDataList)

        elif (barcodeData not in self.barcodeDataList) and (self.currentLocationStatus == True):
            self.barcodeDataList.append(barcodeData)
            # item_dY = str(math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100)
            # item_dX = str(math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100)
            # item_dX = str(self.drone.get_state(moveByEnd)["dY"])
            # self.itemlocation = "(" + item_dX + "," + item_dY + ")"

            # self.request_post.sendData(barcodeData, self.currentLocation, self.itemlocation)
            self.request_post.sendData(barcodeData, self.currentLocation)
            print("*******Send data to sendData() in request post*******")


    def automove(self, movement, fb):
 
        if movement == "UP":
            print("*******MOVE UP********")
            
            if self.moveup<1:
                # time.sleep(1)
                self.moveup +=1
                # self.cancelmove()
                self.dY_1 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
                # self.dY_1 = self.drone.get_state(AltitudeChanged)["altitude"]
                # print("Altitude takeoff :", self.drone.get_state(AltitudeChanged)["altitude"])
                print("**----------> dY_1 (there is no dx_1): ", self.dY_1)
                # print("Position (latitude, longitude, altitude) :", self.drone.get_state(PositionChanged))
                dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
                print("**----------> Altitude Above Ground: ", dY_aboveGround)
                print("**----------> Position Changed: ",self.drone.get_state(PositionChanged))
                # print("moveByEnd (d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed) :", self.drone.get_state(moveByEnd))
                
                #stopmove() is to make sure the drone stop for a while before move to the next direction.. this is to avoid too much inertia
                self.stopmove()
                self.move_Up(self.fb, self.limitDistance)
                
            else:
                self.cancelmove()
                # self.land() # will change to start scanning operation
                
                # print("Altitude takeoff :", self.drone.get_state(AltitudeChanged)["altitude"])
                # self.dY_5 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
                # self.dX_5 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
                self.dY_5 = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
                self.dX_5 = round((self.drone.get_state(moveByEnd)["dY"]), 1)
                
                print("**----------> dX_5(corner 1): ", self.dX_5)
                print("**----------> dY_5(corner 1): ", self.dY_5)
                # print("Position (latitude, longitude, altitude) :", self.drone.get_state(PositionChanged))
                dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
                print("**----------> Altitude Above Ground: ", dY_aboveGround)
                
                # self.move_Up(self.fb, 5)
                print("*********End scanning rack tags***********")
                self.takeoff()
                # self.scanItem()
                # if (self.dX_2 != 0.0) and (self.dX_3 != 0.0) and (self.dX_4 != 0.0) and (self.dX_5 != 0.0) and (self.dY_2 != 0.0) and (self.dY_2 != 0.0) and (self.dY_2 != 0.0) and (self.dY_2 != 0.0):
                #     print("all dx and dy has values")

            
        elif movement == "DOWN":
            print("*********MOVE DOWN**********")
            self.cancelmove()
            
            # self.dY_3 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
            # self.dX_3 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
            self.dY_3 = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_3 = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            dY_aboveGround = round((self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]), 1)
            print("**----------> Altitude Above Ground: ", dY_aboveGround)
            print("**----------> dX_3(corner 3): ", self.dX_3)
            print("**----------> dY_3(corner 3): ", self.dY_3)
            print("**----------> Position Changed: ", self.drone.get_state(PositionChanged))

            self.stopmove()
            self.move_Down(self.fb, self.limitDistance)

        elif movement == "RIGHT":
            print("**********MOVE RIGHT************")
            self.cancelmove()

            # self.dY_2 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
            # self.dX_2 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
            # dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
            self.dY_2 = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_2 = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            dY_aboveGround = round((self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]), 1)
            print("**----------> Altitude Above Ground: ", dY_aboveGround)
            print("**----------> dY_2(corner 2): ", self.dY_2)
            print("**----------> dX_2(corner 2): ", self.dX_2)

            self.stopmove()
            self.move_Right(self.fb, self.limitDistance)

        elif movement == "LEFT":
            print("***********MOVE LEFT**************")
            self.cancelmove()

            # self.dY_4 = math.ceil((self.drone.get_state(AltitudeChanged)["altitude"])*100)/100
            # self.dX_4 = math.ceil((self.drone.get_state(moveByEnd)["dY"])*100)/100
            # dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
            self.dY_4 = round((self.drone.get_state(AltitudeChanged)["altitude"]), 1)
            self.dX_4 = round((self.drone.get_state(moveByEnd)["dY"]), 1)
            dY_aboveGround = self.drone.get_state(AltitudeAboveGroundChanged)["altitude"]
            print("**----------> Altitude Above Ground: ", dY_aboveGround)
            print("**----------> dY_4(corner 4): ", self.dY_4)
            print("**----------> dX_4:(corner 4): ", self.dX_4)
          
            self.stopmove()
            self.move_Left(self.fb, self.limitDistance)

        else:
            self.stopmove()
            print("*********[WARNING] No such direction [WARNING]**********")
            # print("Altitude takeoff :", self.drone.get_state(AltitudeChanged)["altitude"])

        return




    # This function is necessary for the Olympe SDK. It will be called by Olympe
    def run(self):
        window_name = "Olympe Parrot Camera Live Streaming"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.show_yuv_frame(window_name, yuv_frame)
                    
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                    
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                anafi_connection.stop()
            
            if cv2.waitKey(1) & 0xFF == ord('l'):
                anafi_connection.land()

            if cv2.waitKey(1) & 0xFF == ord('t'):
                anafi_connection.takeoff()
                print("taking off")
                       
        cv2.destroyWindow(window_name)

    def takeoff(self):
        assert self.drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait().success()
        print("---------------------------------------------------SUCCESSFUL TAKEOFF---------------------------------------------------------------------")

        return
    
    def land(self):
        assert self.drone(Landing()).wait().success()
        print("---------------------------------------------------SUCCESSFUL LAND---------------------------------------------------------------------")

    def cancelmove(self):
        self.drone(CancelMoveBy()).wait(1).success()
        self.fb = 0
        print("---------------------------------------------------SUCCESSFUL CANCEL---------------------------------------------------------------------")

        return

    def stopmove(self):
        self.drone(extended_move_by(0, 0, 0, 0, 0, 0, 0)).wait(1).success()
        print("---------------------------------------------------SUCCESSFUL REST---------------------------------------------------------------------")
       
        return

    # def move_Forward(self, range):
    #     distance = float(range)
    #     assert self.drone(
    #     moveBy(distance, 0, 0, 0)
    #     >> FlyingStateChanged(state="hovering", _timeout=5)
    #     ).wait().success()
    #     print("---------------------------------------------SUCCESSFUL FORWARD---------------------------------------------------------------------")

    #     return
    
    # def move_Backward(self, range):
    #     distance = -float(range)
    #     assert self.drone(
    #     moveBy(distance, 0, 0, 0)
    #     >> FlyingStateChanged(state="hovering", _timeout=5)
    #     ).wait().success()
    #     print("---------------------------------------------SUCCESSFUL BACKWARD---------------------------------------------------------------------")

    #     return
    
    def move_Right(self, fb, range):
        distance = float(range)
        #extended_move_by(d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed, _timeout=10, _no_expect=False, _float_tol=(1e-07, 1e-09))
        self.drone(extended_move_by(fb, distance, 0, 0, 2, 2, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=5)
        # )
        # print("---------------------------------------------------SUCCESSFUL RIGHT---------------------------------------------------------------------")
        return

    def move_Right2(self, range):
        distance = float(range)
        #extended_move_by(d_x, d_y, d_z, d_psi, max_horizontal_speed, max_vertical_speed, max_yaw_rotation_speed, _timeout=10, _no_expect=False, _float_tol=(1e-07, 1e-09))
        self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0)).wait().success()

        print("---------------------------------------------------SUCCESSFUL RIGHT---------------------------------------------------------------------")
        return

    def move_Left2(self, range):
        distance = -float(range)
        self.drone(extended_move_by(0, distance, 0, 0, 1, 1, 0)).wait().success()
        print("---------------------------------------------------SUCCESSFUL LEFT---------------------------------------------------------------------")
        return

    def move_Up2(self, range):
        distance = -float(range)
        self.drone(extended_move_by(0, 0, distance, 0, 1, 1, 0)).wait().success()
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        print("---------------------------------------------------SUCCESSFUL UP---------------------------------------------------------------------")
        return

    def move_Left(self, fb, range):
        distance = -float(range)
        self.drone(extended_move_by(fb, distance, 0, 0, 2, 2, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=5)
        # )
        # print("---------------------------------------------------SUCCESSFUL LEFT---------------------------------------------------------------------")
        return
    
    def move_Up(self, fb, range):
        distance = -float(range)
        self.drone(extended_move_by(fb, 0, distance, 0, 1, 1, 0)).wait()
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )
        # print("---------------------------------------------------SUCCESSFUL UP---------------------------------------------------------------------")
        return

    def move_Down(self, fb, range):
        distance = float(range)
        self.drone(extended_move_by(fb, 0, distance, 0, 1, 1, 0))
            # >> FlyingStateChanged(state="hovering", _timeout=10)
        # )ss
        # print("---------------------------------------------------SUCCESSFUL DOWN---------------------------------------------------------------------")
        return


    def fullyAutonomous(self):

        takeoffStatus = self.drone.get_state(FlyingStateChanged)["state"]
        # if self.takeoff() == False:
        if takeoffStatus == FlyingStateChanged_State.landed :
            print(takeoffStatus)
            self.takeoff()

        # while self.drone.get_state(FlyingStateChanged)["state"] == FlyingStateChanged_State.hovering :
        
        while self.droneOperation == 1 :
            pass
            # if takeoffStatus != FlyingStateChanged_State.landed:
            # if (self.dX_2 != 100) and (self.dX_3 != 100) and (self.dX_4 != 100) and (self.dX_5 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100) and (self.dY_2 != 100):
            #     self.scanItem()

            # elif self.inst in self.rackInst :
            #     if self.inst1 != self.inst:
            #         self.inst1 = self.inst
            #         self.automove(self.inst, self.fb)
            #         # inst1 = self.inst
            #     else:
            #         pass

            # else:
            #     pass
            # self.stopmove()
            # while self.drone.connect():
            # print("no movement")
            # time.wait(5)
            # print("No Movement")

# Main function
if __name__ == "__main__":
    anafi_connection = AnafiConnection()
    # Start the video stream
    anafi_connection.start()
    # anafi_connection.run()
    # Perform some live video processing while the drone is flying
    anafi_connection.fullyAutonomous()
    # Stop the video stream
    anafi_connection.stop()
