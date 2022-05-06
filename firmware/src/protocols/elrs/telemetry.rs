#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adapted from the official ELRS example here:
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/TelemetryProtocol/telemetry_protocol.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/Telemetry/telemetry.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/Telemetry/telemetry.cpp


// todo: #define - types?
pub const ELRS_TELEMETRY_TYPE_LINK: u8 = 0x01;
pub const ELRS_TELEMETRY_TYPE_DATA: u8 = 0x02;
pub const ELRS_TELEMETRY_TYPE_MASK: u8 = 0x03;
pub const ELRS_TELEMETRY_SHIFT: u8 = 2;
pub const ELRS_TELEMETRY_BYTES_PER_CALL: u8 = 5;
pub const ELRS_TELEMETRY_MAX_PACKAGES: u8 = (255 >> ELRS_TELEMETRY_SHIFT);

pub const ELRS_MSP_BYTES_PER_CALL: u8 = 5;
pub const ELRS_MSP_BUFFER: u8 = 65;
pub const ELRS_MSP_MAX_PACKAGES: u8 = ((ELRS_MSP_BUFFER/ELRS_MSP_BYTES_PER_CALL)+1);


#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum CustomTelemSubTypeID {
    SINGLE_PACKET_PASSTHROUGH = 0xF0,
    STATUS_TEXT = 0xF1,
    MULTI_PACKET_PASSTHROUGH = 0xF2,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum TelemetryState {
    TELEMETRY_IDLE = 0,
    RECEIVING_LENGTH,
    RECEIVING_DATA
}

pub struct CrsfTelemetryPackage {
    type_ : u8,
    size: u8,
    locked: bool,
    updated: bool,
    data: [u8; 69],
}

// #define PAYLOAD_DATA(type0, type1, type2, type3, type4, type5)\
//     uint8_t PayloadData[\
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type0##_PAYLOAD_SIZE) + \
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type1##_PAYLOAD_SIZE) + \
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type2##_PAYLOAD_SIZE) + \
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type3##_PAYLOAD_SIZE) + \
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type4##_PAYLOAD_SIZE) + \
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type5##_PAYLOAD_SIZE) + \
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_GENERAL_RESP_PAYLOAD_SIZE) + \
//         CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_GENERAL_RESP_PAYLOAD_SIZE)]; \
//     crsf_telemetry_package_t payloadTypes[] = {\
//     {CRSF_FRAMETYPE_##type0, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type0##_PAYLOAD_SIZE), false, false, 0},\
//     {CRSF_FRAMETYPE_##type1, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type1##_PAYLOAD_SIZE), false, false, 0},\
//     {CRSF_FRAMETYPE_##type2, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type2##_PAYLOAD_SIZE), false, false, 0},\
//     {CRSF_FRAMETYPE_##type3, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type3##_PAYLOAD_SIZE), false, false, 0},\
//     {CRSF_FRAMETYPE_##type4, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type4##_PAYLOAD_SIZE), false, false, 0},\
//     {CRSF_FRAMETYPE_##type5, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_##type5##_PAYLOAD_SIZE), false, false, 0},\
//     {0, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_GENERAL_RESP_PAYLOAD_SIZE), false, false, 0},\
//     {0, CRSF_TELEMETRY_TOTAL_SIZE(CRSF_FRAME_GENERAL_RESP_PAYLOAD_SIZE), false, false, 0}};\
//     const uint8_t payloadTypesCount = (sizeof(payloadTypes)/sizeof(crsf_telemetry_package_t))
//
// class Telemetry
// {
// public:
//     Telemetry();
//     bool RXhandleUARTin(uint8_t data);
//     void ResetState();
//     bool ShouldCallBootloader();
//     bool ShouldCallEnterBind();
//     bool ShouldCallUpdateModelMatch();
//     bool ShouldSendDeviceFrame();
//     uint8_t GetUpdatedModelMatch() { return modelMatchId; }
//     bool GetNextPayload(uint8_t* nextPayloadSize, uint8_t **payloadData);
//     uint8_t UpdatedPayloadCount();
//     uint8_t ReceivedPackagesCount();
//     bool AppendTelemetryPackage(uint8_t *package);
// private:
//     void AppendToPackage(volatile crsf_telemetry_package_t *current);
//     uint8_t CRSFinBuffer[CRSF_MAX_PACKET_LEN];
//     telemetry_state_s telemetry_state;
//     uint8_t currentTelemetryByte;
//     uint8_t currentPayloadIndex;
//     volatile crsf_telemetry_package_t *telemetryPackageHead;
//     uint8_t receivedPackages;
//     bool callBootloader;
//     bool callEnterBind;
//     bool callUpdateModelMatch;
//     bool sendDeviceFrame;
//     uint8_t modelMatchId;
// };


// todo: Below  might be CRSF only?
//
//
// Telemetry::Telemetry()
// {
//     ResetState();
// }
//
// bool Telemetry::ShouldCallBootloader()
// {
//     bool bootloader = callBootloader;
//     callBootloader = false;
//     return bootloader;
// }
//
// bool Telemetry::ShouldCallEnterBind()
// {
//     bool enterBind = callEnterBind;
//     callEnterBind = false;
//     return enterBind;
// }
//
// bool Telemetry::ShouldCallUpdateModelMatch()
// {
//     bool updateModelMatch = callUpdateModelMatch;
//     callUpdateModelMatch = false;
//     return updateModelMatch;
// }
//
// bool Telemetry::ShouldSendDeviceFrame()
// {
//     bool deviceFrame = sendDeviceFrame;
//     sendDeviceFrame = false;
//     return deviceFrame;
// }
//
//
// PAYLOAD_DATA(GPS, BATTERY_SENSOR, ATTITUDE, DEVICE_INFO, FLIGHT_MODE, VARIO);
//
// bool Telemetry::GetNextPayload(uint8_t* nextPayloadSize, uint8_t **payloadData)
// {
//     uint8_t checks = 0;
//     uint8_t oldPayloadIndex = currentPayloadIndex;
//     uint8_t realLength = 0;
//
//     if (payloadTypes[currentPayloadIndex].locked)
//     {
//         payloadTypes[currentPayloadIndex].locked = false;
//         payloadTypes[currentPayloadIndex].updated = false;
//     }
//
//     do
//     {
//         currentPayloadIndex = (currentPayloadIndex + 1) % payloadTypesCount;
//         checks++;
//     } while(!payloadTypes[currentPayloadIndex].updated && checks < payloadTypesCount);
//
//     if (payloadTypes[currentPayloadIndex].updated)
//     {
//         payloadTypes[currentPayloadIndex].locked = true;
//
//         realLength = CRSF_FRAME_SIZE(payloadTypes[currentPayloadIndex].data[CRSF_TELEMETRY_LENGTH_INDEX]);
//         // search for non zero data from the end
//         while (realLength > 0 && payloadTypes[currentPayloadIndex].data[realLength - 1] == 0)
//         {
//             realLength--;
//         }
//
//         if (realLength > 0)
//         {
//             // store real length in frame
//             payloadTypes[currentPayloadIndex].data[CRSF_TELEMETRY_LENGTH_INDEX] = realLength - CRSF_FRAME_NOT_COUNTED_BYTES;
//             *nextPayloadSize = realLength;
//             *payloadData = payloadTypes[currentPayloadIndex].data;
//             return true;
//         }
//     }
//
//     currentPayloadIndex = oldPayloadIndex;
//     *nextPayloadSize = 0;
//     *payloadData = 0;
//     return false;
// }
//
// uint8_t Telemetry::UpdatedPayloadCount()
// {
//     uint8_t count = 0;
//     for (int8_t i = 0; i < payloadTypesCount; i++)
//     {
//         if (payloadTypes[i].updated)
//         {
//             count++;
//         }
//     }
//
//     return count;
// }
//
// uint8_t Telemetry::ReceivedPackagesCount()
// {
//     return receivedPackages;
// }
//
// fn ResetState(&self) {
//     telemetry_state = TELEMETRY_IDLE;
//     currentTelemetryByte = 0;
//     currentPayloadIndex = 0;
//     receivedPackages = 0;
//
//     uint8_t offset = 0;
//
//     for (int8_t i = 0; i < payloadTypesCount; i++)
//     {
//         payloadTypes[i].locked = false;
//         payloadTypes[i].updated = false;
//         payloadTypes[i].data = PayloadData + offset;
//         offset += payloadTypes[i].size;
//
//         #if defined(UNIT_TEST)
//         if (offset > sizeof(PayloadData)) {
//             cout << "data not large enough\n";
//         }
//         #endif
//     }
// }
//
// fn RXhandleUARTin(&self, data: u8) -> bool {
//     switch(telemetry_state) {
//         case TELEMETRY_IDLE:
//             if (data == CRSF_ADDRESS_CRSF_RECEIVER || data == CRSF_SYNC_BYTE)
//             {
//                 currentTelemetryByte = 0;
//                 telemetry_state = RECEIVING_LENGTH;
//                 CRSFinBuffer[0] = data;
//             }
//             else {
//                 return false;
//             }
//
//             break;
//         case RECEIVING_LENGTH:
//             if (data >= CRSF_MAX_PACKET_LEN)
//             {
//                 telemetry_state = TELEMETRY_IDLE;
//                 return false;
//             }
//             else
//             {
//                 telemetry_state = RECEIVING_DATA;
//                 CRSFinBuffer[CRSF_TELEMETRY_LENGTH_INDEX] = data;
//             }
//
//             break;
//         case RECEIVING_DATA:
//             CRSFinBuffer[currentTelemetryByte + CRSF_FRAME_NOT_COUNTED_BYTES] = data;
//             currentTelemetryByte++;
//             if (CRSFinBuffer[CRSF_TELEMETRY_LENGTH_INDEX] == currentTelemetryByte)
//             {
//                 // exclude first bytes (sync byte + length), skip last byte (submitted crc)
//                 uint8_t crc = crsf_crc.calc(CRSFinBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, CRSFinBuffer[CRSF_TELEMETRY_LENGTH_INDEX] - CRSF_TELEMETRY_CRC_LENGTH);
//                 telemetry_state = TELEMETRY_IDLE;
//
//                 if (data == crc)
//                 {
//                     AppendTelemetryPackage(CRSFinBuffer);
//                     receivedPackages++;
//                     return true;
//                 }
//                 #if defined(UNIT_TEST)
//                 if (data != crc)
//                 {
//                     cout << "invalid " << (int)crc  << '\n';
//                 }
//                 #endif
//
//                 return false;
//             }
//
//             break;
//     }
//
//     return true;
// }
//
// fn AppendTelemetryPackage(&self, package: &[u8]) -> bool {
//     const crsf_header_t *header = (crsf_header_t *) package;
//
//     if (header->type == CRSF_FRAMETYPE_COMMAND && package[3] == 'b' && package[4] == 'l')
//     {
//         callBootloader = true;
//         return true;
//     }
//     if (header->type == CRSF_FRAMETYPE_COMMAND && package[3] == 'b' && package[4] == 'd')
//     {
//         callEnterBind = true;
//         return true;
//     }
//     if (header->type == CRSF_FRAMETYPE_COMMAND && package[3] == 'm' && package[4] == 'm')
//     {
//         callUpdateModelMatch = true;
//         modelMatchId = package[5];
//         return true;
//     }
//     if (header->type == CRSF_FRAMETYPE_DEVICE_PING && package[CRSF_TELEMETRY_TYPE_INDEX + 1] == CRSF_ADDRESS_CRSF_RECEIVER)
//     {
//         sendDeviceFrame = true;
//         return true;
//     }
//
//     uint8_t targetIndex = 0;
//     bool targetFound = false;
//
//
//     if (header->type >= CRSF_FRAMETYPE_DEVICE_PING)
//     {
//         const crsf_ext_header_t *extHeader = (crsf_ext_header_t *) package;
//
//         if (header->type == CRSF_FRAMETYPE_ARDUPILOT_RESP)
//         {
//             // reserve last slot for adrupilot custom frame with the sub type status text: this is needed to make sure the important status messages are not lost
//             if (package[CRSF_TELEMETRY_TYPE_INDEX + 1] == CRSF_AP_CUSTOM_TELEM_STATUS_TEXT)
//             {
//                 targetIndex = payloadTypesCount - 1;
//             }
//             else
//             {
//                 targetIndex = payloadTypesCount - 2;
//             }
//             targetFound = true;
//         }
//         else if (extHeader->orig_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
//         {
//             targetIndex = payloadTypesCount - 2;
//             targetFound = true;
//
//             #if defined(USE_MSP_WIFI) && defined(TARGET_RX) && defined(PLATFORM_ESP8266)
//                 // this probably needs refactoring in the future, I think we should have this telemetry class inside the crsf module
//                 if (wifi2tcp.hasClient() && (header->type == CRSF_FRAMETYPE_MSP_RESP || header->type == CRSF_FRAMETYPE_MSP_REQ)) // if we have a client we probs wanna talk to it
//                 {
//                     DBGLN("Got MSP frame, forwarding to client, len: %d", currentTelemetryByte);
//                     crsf.crsf2msp.parse(package);
//                 }
//                 else // if no TCP client we just want to forward MSP over the link
//             #endif
//             {
//                 // larger msp resonses are sent in two chunks so special handling is needed so both get sent
//                 if (header->type == CRSF_FRAMETYPE_MSP_RESP)
//                 {
//                     // there is already another response stored
//                     if (payloadTypes[targetIndex].updated)
//                     {
//                         // use other slot
//                         targetIndex = payloadTypesCount - 1;
//                     }
//
//                     // if both slots are taked do not overwrite other data since the first chunk would be lost
//                     if (payloadTypes[targetIndex].updated)
//                     {
//                         targetFound = false;
//                     }
//                 }
//             }
//         }
//         else
//         {
//             targetIndex = payloadTypesCount - 1;
//             targetFound = true;
//         }
//     }
//     else
//     {
//         for (int8_t i = 0; i < payloadTypesCount - 2; i++)
//         {
//             if (header->type == payloadTypes[i].type)
//             {
//                 if (!payloadTypes[i].locked && CRSF_FRAME_SIZE(package[CRSF_TELEMETRY_LENGTH_INDEX]) <= payloadTypes[i].size)
//                 {
//                     targetIndex = i;
//                     targetFound = true;
//                 }
//                 #if defined(UNIT_TEST)
//                 else if (CRSF_FRAME_SIZE(package[CRSF_TELEMETRY_LENGTH_INDEX]) > payloadTypes[i].size)
//                 {
//                     cout << "buffer not large enough for type " << (int)payloadTypes[i].type  << " with size " << (int)payloadTypes[i].size << " would need " << CRSF_FRAME_SIZE(package[CRSF_TELEMETRY_LENGTH_INDEX]) << '\n';
//                 }
//                 #endif
//                 break;
//             }
//         }
//     }
//
//     if targetFound {
//         memcpy(payloadTypes[targetIndex].data, package, CRSF_FRAME_SIZE(package[CRSF_TELEMETRY_LENGTH_INDEX]));
//         payloadTypes[targetIndex].updated = true;
//     }
//
//     return targetFound;
// }
