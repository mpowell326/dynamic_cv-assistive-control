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


class CanCameraInteraface(CanTool):
    jsXdemand = 0
    jsYdemand = 0
    adjXdemand = 0
    adjYdemand = 0
    x_restrict = 0
    y_restrict = 0
    cameraConnected = False

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

        # # Connect to camera service
        # camSocket = socketConnection()      
        # camSocket.init(('localhost', 10000))
        # print >> sys.stderr, 'Checking for a connection to camera service'
        # if camSocket.waitForConnection():
        #     self.cameraConnected = True


        while True:
            try:
                # Receive the current joystick demand from the can bus
                self.getJSDemand()
                

                # Get the x,y restriction to apply
                if self.cameraConnected :
                    self.x_restrict, self.y_restrict = camSocket.getCameraData()
                # elif camSocket.waitForConnection():
                #     self.cameraConnected = True
                #     self.x_restrict, self.y_restrict = camSocket.getCameraData()
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
                print("adj(X): {: 7.2f} adj(Y): {: 7.2f} | x: {: 3d} y: {: 3d}".format(self.adjXdemand, self.adjYdemand, self.x_restrict,self.y_restrict))

            except KeyboardInterrupt:
                break
            except Exception as e:
                print('Error: {}'.format(e))


if __name__ == "__main__":
    CanCameraInteraface()