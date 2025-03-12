# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json
import time
import threading

from src.hardware.serialhandler.threads.messageconverter import MessageConverter
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    Klem,
    Control,
    SteerMotor,
    SpeedMotor,
    Brake,
    AEB,
    HighwaySignal,
    ToggleBatteryLvl,
    ToggleImuData,
    ToggleInstant,
    ToggleResourceMonitor,
    LaneKeeping,
    LaneSpeed,
    DrivingMode
)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender


class threadWrite(ThreadWithStop):
    """This thread write the data that Raspberry PI send to NUCLEO.\n

    Args:
        queues (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        serialCom (serial.Serial): Serial connection between the two boards.
        logFile (FileHandler): The path to the history file where you can find the logs from the connection.
        example (bool, optional): Flag for exmaple activation. Defaults to False.
    """

    # ===================================== INIT =========================================
    def __init__(self, queues, serialCom, logFile, logger, debugger = False, example=False):
        super(threadWrite, self).__init__()
        self.queuesList = queues
        self.serialCom = serialCom
        self.logFile = logFile
        self.exampleFlag = example
        self.logger = logger
        self.debugger = debugger

        self.running = False
        self.engineEnabled = False
        self.messageConverter = MessageConverter()
        self.steerMotorSender = messageHandlerSender(self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(self.queuesList, SpeedMotor)
        self.configPath = "src/utils/table_state.json"

        self.loadConfig("init")
        self.subscribe()

        if example:
            self.i = 0.0
            self.j = -1.0
            self.s = 0.0
            self.example()
        

        self.lastspeed = 0 # For Finishing AEB System
        self.highwayMode = False # For Highway Mode

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""

        self.klSubscriber = messageHandlerSubscriber(self.queuesList, Klem, "lastOnly", True)
        self.controlSubscriber = messageHandlerSubscriber(self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(self.queuesList, Brake, "lastOnly", True)
        self.AEBSubscriber = messageHandlerSubscriber(self.queuesList, AEB, "lastOnly", True)
        self.highway_signalSubscriber = messageHandlerSubscriber(self.queuesList, HighwaySignal, "lastOnly", True)

        self.instantSubscriber = messageHandlerSubscriber(self.queuesList, ToggleInstant, "lastOnly", True)
        self.batterySubscriber = messageHandlerSubscriber(self.queuesList, ToggleBatteryLvl, "lastOnly", True)
        self.resourceMonitorSubscriber = messageHandlerSubscriber(self.queuesList, ToggleResourceMonitor, "lastOnly", True)
        self.imuSubscriber = messageHandlerSubscriber(self.queuesList, ToggleImuData, "lastOnly", True)
        self.lanekeepingSubscriber = messageHandlerSubscriber(self.queuesList, LaneKeeping, "lastOnly", True)
        self.lanespeedSubscriber = messageHandlerSubscriber(self.queuesList, LaneSpeed, "lastOnly", True)
        self.modeSubscriber = messageHandlerSubscriber(self.queuesList, DrivingMode, "lastOnly", True)

    # ==================================== SENDING =======================================

    def sendToSerial(self, msg):
        command_msg = self.messageConverter.get_command(**msg)
        if command_msg != "error":
            self.serialCom.write(command_msg.encode("ascii"))
            self.logFile.write(command_msg)
            #print("\t\t\t\t\t\tserial:", command_msg)


    def loadConfig(self, configType):
        with open(self.configPath, "r") as file:
            data = json.load(file)

        if configType == "init":
            data = data[len(data)-1]
            command = {"action": "batteryCapacity", "capacity": data["batteryCapacity"]["capacity"]}
            self.sendToSerial(command)
            time.sleep(0.05)
        else:
            for e in range(4):
                if data[e]["value"] == "False":
                    value = 0
                else:
                    value = 1 
                command = {"action": data[e]['command'], "activate": value}
                self.sendToSerial(command)
                time.sleep(0.05)

    def convertFc(self,instantRecv):
        if instantRecv =="True":
            return 1
        else :
            return 0
        
    # ===================================== RUN ==========================================
    def run(self):
        """In this function we check if we got the enable engine signal. After we got it we will start getting messages from raspberry PI. It will transform them into NUCLEO commands and send them."""

        while self._running:
            try:
                drivingmode = self.modeSubscriber.receive()
                if drivingmode is not None:
                    currnetMode = drivingmode
                    print(f"Driving Mode switched to: {currnetMode}")

                klRecv = self.klSubscriber.receive()
                if klRecv is not None:
                    if self.debugger:
                        self.logger.info(klRecv)
                    if klRecv == "30":
                        self.running = True
                        self.engineEnabled = True
                        command = {"action": "kl", "mode": 30}
                        self.sendToSerial(command)
                        self.loadConfig("sensors")
                    elif klRecv == "15":
                        self.running = True
                        self.engineEnabled = False
                        command = {"action": "kl", "mode": 15}
                        self.sendToSerial(command)
                        self.loadConfig("sensors")
                    elif klRecv == "0":
                        self.running = False
                        self.engineEnabled = False
                        command = {"action": "kl", "mode": 0}
                        self.sendToSerial(command)

                if self.running:
                    if self.engineEnabled:
                        if currnetMode == "auto":
                            ########################### Highway ###########################
                            highway_signal = self.highway_signalSubscriber.receive() # 1: highway, 0: non-highway
                            if highway_signal is not None:
                                if highway_signal == 1.0:
                                    self.highwayMode = True
                                    if self.debugger:
                                        self.logger.info("Highway Mode activated")
                                elif highway_signal == 0.0:
                                    self.highwayMode = False
                            
                            ########################### YOLO ###########################
                            aeb_signal = self.AEBSubscriber.receive()
                            if aeb_signal is not None:
                                if self.debugger:
                                    self.logger.info(f"AEB signal received: {aeb_signal}")
                                if aeb_signal == 0.0:
                                    if self.lastspeed > 0:
                                        command = {"action": "speed", "speed": self.lastspeed}
                                        self.sendToSerial(command)

                                elif aeb_signal == 1.0:
                                    command = {"action": "speed", "speed": 0}
                                    self.sendToSerial(command)
                                

                            ########################### LaneKeeping ###########################
                            if aeb_signal != 1.0:
                                laneRecv = self.lanekeepingSubscriber.receive()
                                if laneRecv is not None:
                                    if self.debugger:
                                        self.logger.info(laneRecv)
                                    command = {"action": "steer", "steerAngle": int(laneRecv-10)}
                                    #print("serial Write : ", int(laneRecv - 10))
                                    self.sendToSerial(command)
                                    #print("\t\t\t\t\tserial Write : ",int(laneRecv - 10))

                                laneRecv_speed = self.lanespeedSubscriber.receive()
                                if laneRecv_speed is not None:
                                    laneRecv_speed = int(laneRecv_speed) + (100 if self.highwayMode else 0) # Highway Mode Speed
                                    if self.debugger:
                                        self.logger.info(laneRecv_speed)
                                    command = {"action": "speed", "speed":int(laneRecv_speed)}
                                    #print("#####speed: ",int(laneRecv_speed))
                                    self.sendToSerial(command)

                        else:
                            brakeRecv = self.brakeSubscriber.receive()# other brake
                            if brakeRecv is not None:
                                if self.debugger:
                                    self.logger.info(brakeRecv)
                                command = {"action": "brake", "steerAngle": int(brakeRecv)}
                                self.sendToSerial(command)

                            speedRecv = self.speedMotorSubscriber.receive()
                            if speedRecv is not None: 
                                if self.debugger:
                                    self.logger.info(speedRecv)
                                command = {"action": "speed", "speed": int(speedRecv)}
                                self.lastspeed = int(speedRecv)
                                self.sendToSerial(command)

                            steerRecv = self.steerMotorSubscriber.receive()
                            if steerRecv is not None:
                                if self.debugger:
                                    self.logger.info(steerRecv) 
                                command = {"action": "steer", "steerAngle": int(steerRecv)}
                                self.sendToSerial(command)

                            controlRecv = self.controlSubscriber.receive()
                            if controlRecv is not None:
                                if self.debugger:
                                    self.logger.info(controlRecv) 
                                command = {
                                    "action": "vcd",
                                    "time": int(controlRecv["Time"]),
                                    "speed": int(controlRecv["Speed"]),
                                    "steer": int(controlRecv["Steer"]),
                                }
                                self.sendToSerial(command)

                    instantRecv = self.instantSubscriber.receive()
                    if instantRecv is not None: 
                        if self.debugger:
                            self.logger.info(instantRecv) 
                        command = {"action": "instant", "activate": int(instantRecv)}
                        self.sendToSerial(command)

                    batteryRecv = self.batterySubscriber.receive()
                    if batteryRecv is not None: 
                        if self.debugger:
                            self.logger.info(batteryRecv)
                        command = {"action": "battery", "activate": int(batteryRecv)}
                        self.sendToSerial(command)

                    resourceMonitorRecv = self.resourceMonitorSubscriber.receive()
                    if resourceMonitorRecv is not None: 
                        if self.debugger:
                            self.logger.info(resourceMonitorRecv)
                        command = {"action": "resourceMonitor", "activate": int(resourceMonitorRecv)}
                        self.sendToSerial(command)

                    imuRecv = self.imuSubscriber.receive()
                    if imuRecv is not None: 
                        if self.debugger:
                            self.logger.info(imuRecv)
                        command = {"action": "imu", "activate": int(imuRecv)}
                        self.sendToSerial(command)

            except Exception as e:
                print(e)

    # def _send_brake_command(self, value):
    #     """Send a brake command to the NUCLEO."""
    #     try:
    #         command = f"#BRAKE:{value};\r\n"
    #         self.serialCom.write(command.encode("ascii"))
    #         self.logFile.write(command)
    #         if self.debugger:
    #             self.logger.info(f"Sent brake command: {command}")
    #     except Exception as e:
    #         if self.debugger:
    #             self.logger.error(f"Failed to send brake command: {e}")

    # ==================================== START =========================================
    def start(self):
        super(threadWrite, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        """This function will close the thread and will stop the car."""

        self.exampleFlag = False
        command = {"action": "kl", "mode": 0.0}
        self.sendToSerial(command)
        time.sleep(2)
        super(threadWrite, self).stop()

    # ================================== EXAMPLE =========================================
    def example(self):
        """This function simulte the movement of the car."""

        if self.exampleFlag:
            self.signalRunningSender.send({"Type": "Run", "value": True})
            self.speedMotorSender.send({"Type": "Speed", "value": self.s})
            self.steerMotorSender.send({"Type": "Steer", "value": self.i})
            self.i += self.j
            if self.i >= 21.0:
                self.i = 21.0
                self.s = self.i / 7
                self.j *= -1
            if self.i <= -21.0:
                self.i = -21.0
                self.s = self.i / 7
                self.j *= -1.0
            threading.Timer(0.01, self.example).start()
