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
import serial
from src.templates.workerprocess import WorkerProcess
from src.hardware.serialhandler.threads.filehandler import FileHandler
from src.hardware.serialhandler.threads.threadRead import threadRead
from src.hardware.serialhandler.threads.threadWrite import threadWrite

class processSerialHandler(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, queueList, logging, debugging=False):
        devFile = "/dev/ttyACM0"
        logFile = "historyFile.txt"

        # comm init
        self.serialCom = serial.Serial(devFile, 19200, timeout=0.1)
        self.serialCom.flushInput()
        self.serialCom.flushOutput()

        # log file init
        self.historyFile = FileHandler(logFile)
        self.queuesList = queueList
        self.logger = logging
        self.debugging = debugging
        
        super(processSerialHandler, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def _stop(self):
        for thread in self.threads:
            thread.stop()
            thread.join()
        super(processSerialHandler, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        super(processSerialHandler, self).run()
        self.historyFile.close()

    # ===================================== INIT TH =================================
    def _init_threads(self):
        """Initializes the read and the write thread."""
        readTh  = threadRead(self.serialCom,self.historyFile,self.queuesList)
        self.threads.append(readTh)
        writeTh = threadWrite(self.queuesList, self.serialCom, self.historyFile)
        self.threads.append(writeTh)


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processSerialHandler.py
if __name__ == "__main__":
    from multiprocessing import Queue, Event, Pipe
    import logging
    import time

    allProcesses = list()
    debugg = False
    # We have a list of multiprocessing.Queue() which individualy represent a priority for processes.
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    logger = logging.getLogger()
    pipeRecv, pipeSend = Pipe(duplex=False)
    process = processSerialHandler(queueList, logger, debugg)
    allProcesses.append(process)

    for process in allProcesses:
        process.daemon = True
        process.start()

    i = 0.0
    j = -1.0
    s=0.0
    while True:
        time.sleep(0.05)
        pipeSend.send({"msgType": "dict", "value": {"action": "2", "steerAngle": i}})
        pipeSend.send({"msgType": "dict", "value": {"action": "1", "speed": s}})
        i += j
        if i >= 21.0:
            i = 21.0
            s = i/7
            j *= -1
        if i <= -21.0:
            i = -21.0
            s = i /7
            j *= -1.0
    # ===================================== STAYING ALIVE ====================================
    blocker = Event()
    try:
        blocker.wait()
    except KeyboardInterrupt:
        print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
        for proc in allProcesses:
            if hasattr(proc, "stop") and callable(getattr(proc, "stop")):
                print("Process with stop", proc)
                proc.stop()
                proc.join()
            else:
                print("Process witouth stop", proc)
                proc.terminate()
                proc.join()
