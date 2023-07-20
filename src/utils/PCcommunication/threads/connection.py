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
from twisted.internet import protocol
import json
from src.utils.messages.allMessages import EngineRun,SpeedMotor, SteerMotor, Brake
# One class is generated for each new connection
class SingleConnection(protocol.Protocol):

    # ================================= CONNECTION MADE =====+================================
    def connectionMade(self):
        peer = self.transport.getPeer()
        self.factory.connectiondata = peer.host + ":" + str(peer.port)
        self.factory.connection = self
        self.connected=False
        print("Attempting connection by :", self.factory.connectiondata)
    
    # ================================= CONNECTION LOST ======================================
    def connectionLost(self, reason):
        print("Connection lost with ", self.factory.connectiondata, " due to: ", reason)
        self.factory.isConnected = False
        self.factory.isConnected = False
        self.factory.connectiondata = None
        self.factory.connection = None

    # ================================== DATA RECEIVED =======================================
    def dataReceived(self, data):
        if self.factory.isConnected == False:
            pswd = data.decode()
            if pswd == "Ala-Bala":
                self.factory.isConnected = True
                print("Connected with ", self.factory.connectiondata, " : ", data.decode())
            else:
                print("Connection attempted failed with incorrect password ", self.factory.connectiondata)
                self.factory.isConnected = False
                self.factory.connectiondata = None
                self.transport.loseConnection()
                self.factory.connection = None
        else:
            dataJSON = json.loads(data.decode())
            print(dataJSON)
            if dataJSON["action"] == "startEngine":
                self.factory.queues[EngineRun.Queue.value].put({ "Owner" : EngineRun.Owner.value , "msgID": EngineRun.msgID.value, "msgType" :EngineRun.msgType.value,"msgValue":dataJSON["value"] })
            elif dataJSON["action"] == "brake":
                self.factory.queues[Brake.Queue.value].put({ "Owner" : Brake.Owner.value , "msgID": Brake.msgID.value, "msgType" :Brake.msgType.value,"msgValue":dataJSON["value"] })
            elif dataJSON["action"] == "speed":
                self.factory.queues[SpeedMotor.Queue.value].put({ "Owner" : SpeedMotor.Owner.value , "msgID": SpeedMotor.msgID.value, "msgType" : SpeedMotor.msgType.value,"msgValue":dataJSON["value"] })
            elif dataJSON["action"] == "steer":
             self.factory.queues[SteerMotor.Queue.value].put({ "Owner" : SteerMotor.Owner.value , "msgID": SteerMotor.msgID.value, "msgType" : SteerMotor.msgType.value,"msgValue":dataJSON["value"] })   
            print("Received from", self.factory.connectiondata, " : ", data.decode())
            
    # ===================================== SEND DATA ==========================================
    def send_data(self, message,message2):
        self.transport.write(self.factory.encoder[message2].to_bytes(1,byteorder="big"))
        self.transport.write(len(message.encode("utf-8")).to_bytes(4, byteorder='big'))  # send size of image
        self.transport.write(message.encode("utf-8"))  # send image data


# The server itself. Creates a new Protocol for each new connection and has the info for all of them.
class FactoryDealer(protocol.Factory):
    # ======================================= INIT =============================================
    def __init__(self,queues):
        self.connection = None
        self.isConnected = False
        self.connectiondata = None
        self.queues = queues
        self.encoder = {'base64':1,'table':2,"Position":3,"State&Position":4, "Boolean":5, "Boolean2":6}
       
    def send_data_to_client(self, message,message2):
        if self.isConnected == True:
            self.connection.send_data(message,message2)
        else:
            print("Client not connected")

    # ================================== BUILD PROTOCOL ========================================
    def buildProtocol(self, addr):
        conn = SingleConnection()
        conn.factory = self
        return conn
    
    # =================================== AUXILIARY ============================================
    def doStart(self):
        print("Start factory")
    
    def doStop(self):
        print("Stop factory")
        