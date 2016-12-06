#! /usr/bin/python
"""
Tool that listens to js demand messages and publishes adjusted message back on the CAN bus.

Copyright 2016 Dynamic Controls
"""
import os
import sys
import time
import select
import socket
from can_tool import *
import importutils
importutils.addImportDirectory('/home/nuc/projects/cv_assistive_control')
import lib_arucoTracker
import cv2
import math
import collections

# class socketConnection():
#     # sock
#     # server_address
#     # client_address
#     # connection


#     def init(self, address):
#         # Create a TCP/IP socket
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.sock.setblocking(0)
#         self.server_address = address
#         print >> sys.stderr, 'starting up on %s' % (self.server_address,)
#         self.sock.bind(self.server_address)

#         #Listen for incoming connections 
#         self.sock.listen(1)

#     def waitForConnection(self):
#         try:
#             self.connection, self.client_address = self.sock.accept()
#             return True
#         except:
#             return False

#     def getCameraData(self):
#         data = connection.recv(1024)
#         if data:
#             print >> sys.stderr, 'recieved "%s"' % data
#             return data

#     def closeConnection(self):
#         # Clean up the connection
#         self.connection.close()

def moving_average(iterable, n=3):
    # moving_average([40, 30, 50, 46, 39, 44]) --> 40.0 42.0 45.0 43.0
    # http://en.wikipedia.org/wiki/Moving_average
    it = iter(iterable)
    d = deque(itertools.islice(it, n-1))
    d.appendleft(0)
    s = sum(d)
    for elem in it:
        s += elem - d.popleft()
        d.append(elem)
        yield s / n


class CanCameraInteraface(CanTool):
    jsXdemand = 0
    jsYdemand = 0
    adjXdemand = 0
    adjYdemand = 0
    x_restrict = 0
    y_restrict = 0
    cameraConnected = False
    camX = 0
    camY = 0
    camZ = collections.deque(maxlen=10)

    def getJSDemand(self):
        messages = self.incoming_messages.get_filtered_messages(100,PGN.IS_2D_16BIT_USER_INPUT_DEMAND)
        for message in reversed(messages):
            if message.function_id == 27 :
                self.jsXdemand = message.x_demand
                self.jsYdemand = message.y_demand
                break



    def sendAdjustedDemand(self):
        self.send(IS2D16BITUserInputDemand(x_demand=self.adjXdemand,y_demand=self.adjYdemand,function_id=28))

    def calculateAdjDemand(self, x_restrict=None, y_restrict=None):
        if x_restrict != None and y_restrict != None:
            if self.jsYdemand > 0 and y_restrict >0:
                self.adjYdemand = self.jsYdemand * (100-y_restrict)/100 
            else:
                self.adjYdemand = self.jsYdemand

            if self.jsXdemand < 0 and x_restrict < 0:
                self.adjXdemand = self.jsXdemand * (100-abs(x_restrict))/100
            elif self.jsXdemand > 0 and x_restrict > 0:
                self.adjXdemand = self.jsXdemand * (100-abs(x_restrict))/100       
            else:
                self.adjXdemand = self.jsXdemand


    def pollKeyboard(self):
        i,o,e = select.select([sys.stdin],[],[],0.0001)
        for s in i:
            if s == sys.stdin:
                input = sys.stdin.readline()
                return input
        return False

    def main(self):

        # Send message to tell remote to start listening
        self.sendAdjustedDemand()


        self.cameraConnected = True
        
        if self.cameraConnected == True:

            # camera = lib_arucoTracker.arucoTracker(10, 0.198)
            camera = lib_arucoTracker.arucoTracker(244, 0.032)
            id = camera.getTrackedMarkerID()
            print('Tracked Marker ID: {}'.format(id))

            camera.openCaptureDevice(0)
            camera.setDetectorParams()
            camera.calibrateFromFile("/home/nuc/src/aruco-2.0.14/build/utils_calibration/cameraCalibration.yml")


        while True:
            try:
                # Receive the current joystick demand from the can bus
                self.getJSDemand()

                # Get the x,y restriction to apply
                if self.cameraConnected :
                    camera.getNextFrame();
                    camera.detectMarkers();
                    # camera.displayMarker();

                    if(camera.getMarkerPoseZ() > 0):
                        self.camX = camera.getMarkerPoseX()
                        self.camY = camera.getMarkerPoseY()
                        self.camZ.append(camera.getMarkerPoseZ())
                    else:
                        self.camX = 0
                        self.camY = 0
                        self.camZ.append(1)

                    angle = math.atan2(self.camX, self.camY)

                    if (sum(self.camZ)/10 < 1 and sum(self.camZ)/10 > 0 ):
                        self.y_restrict = (1-sum(self.camZ)/10)*100
                    else:
                        self.y_restrict = 0

                    # if (math.fabs(angle) < 45):
                    #     self.x_restrict = cmp(angle,0)*50   


                    if(cv2.waitKey(1000/30)==27):    # tab key
                        break


                else:
                    # Get keyboard input
                    line = self.pollKeyboard()
                    if line != False:
                        axis, percent = line.split()
                        if 'y' in axis:
                            self.y_restrict = int(percent)
                        if 'x' in axis:
                            self.x_restrict = int(percent)
                
                # Apply the x,y restrction to the js demand and send back on can bus
                self.calculateAdjDemand(self.x_restrict, self.y_restrict)
                self.sendAdjustedDemand()

                self.incoming_messages.clear_messages()
                # print("JS(X): {0} JS(Y): {1}".format(self.jsXdemand, self.jsYdemand))
                print("adj(X): {: 7.2f} adj(Y): {: 7.2f} | x: {: 3f} y: {: 3f} z: {: 3f}".format(self.adjXdemand, self.adjYdemand, self.camX,self.camY,self.y_restrict))

            except KeyboardInterrupt:
                break
            except Exception as e:
                print('Error: {}'.format(e))


if __name__ == "__main__":
    CanCameraInteraface()
