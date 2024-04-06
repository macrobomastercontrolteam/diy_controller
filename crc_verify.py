import struct
from crc_table import CRC16_INIT, CRC16_table, CRC8_INIT, CRC8_table

class CRC:
    
    def verify_CRC8_check_sum(self, pch_message, dw_length):
        if pch_message == 0 or dw_length <= 2:
            return False
        ucExpected =  self.get_CRC8_check_sum(pch_message, dw_length - 1, CRC8_INIT)
        return ucExpected == pch_message[dw_length - 1]

    def append_CRC8_check_sum(self, pch_message, dw_length):
        if pch_message == 0 or dw_length <= 2:
                return
        
        ucCRC = self.get_CRC8_check_sum(pch_message, dw_length - 1, CRC8_INIT)

        pch_message += struct.pack("<B",ucCRC)
        return pch_message

    def get_CRC16_check_sum(self, pch_message, dw_length, wCRC):
        if pch_message is None:
            return 0xFFFF
        
        # while(dw_length):
        for i in range(dw_length):
            # dw_length-=1
            chData = pch_message[i]
            wCRC = ((wCRC) >> 8) ^ CRC16_table[((wCRC) ^ (chData)) & 0x00ff]
            # print(wCRC)
        return wCRC

    def verify_CRC16_check_sum(self, pchMessage, dwLength):
        if pchMessage is None or dwLength <= 2:
            return False
        wExpected = self.get_CRC16_check_sum(pchMessage, dwLength - 2, CRC16_INIT)
        return (wExpected & 0xff) == pchMessage[dwLength - 2] and ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]

    def append_CRC16_check_sum(self, pchMessage, dwLength):
        if pchMessage is None or dwLength <= 2:
            return
        wCRC = self.get_CRC16_check_sum(pchMessage, dwLength-2, CRC16_INIT)
        pchMessage = pchMessage +  struct.pack("<H", wCRC)
        
        # print(pchMessage)
        return pchMessage
    
    def get_CRC8_check_sum(self,pch_message, dw_length, ucCRC8):
        for i in range(dw_length):
            uc_index = ucCRC8 ^ pch_message[i]
            ucCRC8 = CRC8_table[uc_index]
            # dw_length-=1
            
            # print(ucCRC8)
        return ucCRC8