#!/usr/bin/env python3

class SerialMessageConverter:
    commands = {
        "speed": [["speed"], [3], [False]],
        "steer": [["steerAngle"], [3], [False]],
        "brake": [["steerAngle"], [3], [False]],
        "batteryCapacity": [["capacity"], [5], [False]],
        "battery": [["activate"], [1], [False]],
        "instant": [["activate"], [1], [False]],
        "resourceMonitor": [["activate"], [1], [False]],
        "imu": [["activate"], [1], [False]],
        "vcd": [["speed", "steer", "time"], [3, 3, 3], [False]],
        "kl": [["mode"], [2], [False]]
    }

    def verify_command(self, action, commandDict):
        """명령어와 매개변수가 올바른지 검증"""
        if action not in self.commands:
            print(0)
            return False

        expected_params = self.commands[action][0]
        expected_digits = self.commands[action][1]

        if len(commandDict.keys()) != len(expected_params):
            print(00)
            return False

        for i, (key, value) in enumerate(commandDict.items()):
            if key not in expected_params:
                print(key)
                print(1)
                print(expected_params)
                return False
            elif type(value) != int:
                print(2)
                return False
            elif value < 0 and len(str(value)) > (expected_digits[i] + 1):  # 음수일 경우 "-" 포함
                print(3)
                return False
            elif value > 0 and len(str(value)) > expected_digits[i]:
                print(4)
                return False

        return True

    def get_command(self, action, **kwargs):
        """NUCLEO가 요구하는 포맷으로 변환"""
        valid = self.verify_command(action, kwargs)
        if valid:
            listKwargs = self.commands[action][0]
            command = "#" + action + ":"

            for key in listKwargs:
                value = kwargs[key]
                command += str(value) + ";"

            command += ";\r\n"
            return command
        else:
            return "error"

