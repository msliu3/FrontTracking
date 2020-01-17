"""
@File    :   DriverMonitor.py
@Contact :   mspzhuz@gmail.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/11 19:02   zzhu      1.0

@Description
------------
发送监控指令[0x80, 0x00, 0x80]后，将电机驱动器返回的bytes型监控信息
(包含电机状态、故障信息、母线电压、输出电流、转速、位置给定和位置反馈）
处理并记录在一个dictionary中返回

"""


class DriverMonitor:

    def __init__(self):
        self.receivedByte = bytes()
        self.monitorData = {
            "ONorOFF": bool(),  # 'ON'-True, 'OFF'-False
            "Malfunction": '',
            "InputVoltage": 0,
            "OutputCurrent": 0.0,
            "RPM": 0.0,
            "GivenPosition": 0,
            "FeedbackPosition": 0
        }

    def processData(self, receivedBytes):

        self.receivedByte = receivedBytes
        data = list(self.receivedByte)

        # 电机状态
        self.monitorData["ONorOFF"] = bool(data[2])

        # 故障信息
        self.monitorData["Malfunction"] = ''
        if len(data) > 32:
            malfunction = data[5:8]
            if malfunction == [0x80, 0x00, 0x02, 0x82]:
                self.monitorData["Malfunction"] = 'Over-current!'
            elif malfunction == [0x80, 0x00, 0x04, 0x84]:
                self.monitorData["Malfunction"] = 'Over-voltage!'
            elif malfunction == [0x80, 0x00, 0x08, 0x88]:
                self.monitorData["Malfunction"] = 'Encoder malfunction!'
            elif malfunction == [0x80, 0x00, 0x10, 0x90]:
                self.monitorData["Malfunction"] = 'Over-heat!'
            elif malfunction == [0x80, 0x00, 0x20, 0x90]:
                self.monitorData["Malfunction"] = 'Undervoltage!'
            elif malfunction == [0x80, 0x00, 0x40, 0xC0]:
                self.monitorData["Malfunction"] = 'Overload!'
            else:
                self.monitorData["Malfunction"] = None

        # 母线电压
        inputVoltage = data[-28:-24]
        self.monitorData["InputVoltage"] = (inputVoltage[1] << 8) + inputVoltage[2]

        # 输出电流
        outputCurrent = data[-24:-20]
        temp = (outputCurrent[1] << 8) + outputCurrent[2]
        self.monitorData["OutputCurrent"] = temp / 100

        # 转速
        RPM = data[-20:-16]
        temp = (RPM[1] << 8) + RPM[2]
        temp = self.hex2int_8(temp)
        self.monitorData["RPM"] = temp / 6000 * 16384

        # 位置给定
        GivenPosH = data[-16:-12]
        GivenPosL = data[-12:-8]
        temp_H = (GivenPosH[1] << 8) + GivenPosH[2]
        temp_L = (GivenPosL[1] << 8) + GivenPosL[2]
        temp = (temp_H << 16) + temp_L
        self.monitorData["GivenPosition"] = self.hex2int(temp)

        # 位置反馈
        FeedbackPosH = data[-8:-4]
        FeedbackPosL = data[-4:]
        temp_H = (FeedbackPosH[1] << 8) + FeedbackPosH[2]
        temp_L = (FeedbackPosL[1] << 8) + FeedbackPosL[2]
        temp = (temp_H << 16) + temp_L
        self.monitorData["FeedbackPosition"] = self.hex2int(temp)

        return self.monitorData

    def getFeedbackPos(self):
        return self.monitorData["FeedbackPosition"]

        # 8位16进制 -> 有符号整型数

    def hex2int(self, data):
        sign_bit = (data & 0x80000000) >> 31
        if sign_bit == 1:
            result = (data - 1) ^ 0xFFFFFFFF
            return -result
        else:
            return data

    def hex2int_8(self, data):
        sign_bit = (data & 0x8000) >> 15
        if sign_bit == 1:
            result = (data - 1) ^ 0xFFFF
            return -result
        else:
            return data

    pass
