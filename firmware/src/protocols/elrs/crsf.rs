#![allow(non_snake_case)]
#![allow(unused_parens)]
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

//! Adapted from:
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/CRSF/CRSF.h
//! https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/CRSF/CRSF.cpp
// todo: We don't need devCRSF files, right?

// todo: Trim this file down to only what you need; you're not really communicating over
// todo UART here; just find the bits the ELRS code has tightly integrated, and remove the rest.

const OpenTXsyncPacketInterval: u32 = 200; // in ms todo: define; what type
// this is the smallest telemetry FIFO size in ETX with CRSF defined
const HANDSET_TELEMETRY_FIFO_SIZE: u16 = 128; // todo: Define type

pub struct CRSF {

    // Todo: Is this a CRSF tx or rx module??
    // #if CRSF_RX_MODULE
    // CRSF(Stream *dev) : _dev(dev)
    // {
    // }
    //
    // CRSF(Stream &dev) : _dev(&dev) {}

    // #endif

    // static HardwareSerial Port;
    // static Stream *PortSecondary; // A second UART used to mirror telemetry out on the TX, not read from

    pub ChannelDataIn: [u16; 16],

    /////Variables/////
    pub ParameterUpdateData: [u8; 3],

    // #ifdef CRSF_TX_MODULE

    // static void (*RecvModelUpdate)();
    // static void (*RecvParameterUpdate)();
    // static void (*RCdataCallback)();

    // The model ID as received from the Transmitter
    pub modelId: u8,
    pub ForwardDevicePings: bool, // true if device pings should be forwarded OTA
    pub elrsLUAmode: bool,

    /// UART Handling ///
    pub GoodPktsCountResult: u32, // need to latch the results
    pub BadPktsCountResult: u32, // need to latch the results
    // #endif

    // #ifdef CRSF_RX_MODULE
    pub PackedRCdataOut: Channels,            // RC data in packed format for output.
    // static uint16_t GetChannelOutput(uint8_t ch);
    // #endif

    pub LinkStatistics: PayloadLinkStatistics, // Link Statisitics Stored as Struct

    // #ifdef CRSF_TX_MODULE

    ///// Variables for OpenTX Syncing //////////////////////////


    pub OpenTXsyncLastSent: u32,

    // #endif

    /////////////////////////////////////////////////////////////
    pub CRSFstate: bool,
    pub _dev: Stream,

    pub inBuffer: InBuffer,

// #if CRSF_TX_MODULE
    /// OpenTX mixer sync ///
    pub RequestedRCpacketInterval: u32,
    pub RCdataLastRecv: u32,
    pub OpenTXsyncOffset: i32,
    pub OpenTXsyncOffsetSafeMargin: u32,
    pub OpentxSyncActive: bool,
    pub CRSFoutBuffer: [u8; CRSF_MAX_PACKET_LEN],

    /// UART Handling ///
    pub SerialInPacketLen: u8,                  // length of the CRSF packet as measured
    pub SerialInPacketPtr: u8,                   // index where we are reading/writing
    pub CRSFframeActive: bool,  //since we get a copy of the serial data use this flag to know when to ignore it
    pub GoodPktsCount: u32,
    pub BadPktsCount: u32,
    pub UARTwdtLastChecked: u32,
    pub maxPacketBytes: u8,
    pub maxPeriodBytes: u8,
    pub TxToHandsetBauds: [u32; 6],
    pub UARTcurrentBaudIdx: usize,
    pub MspData: [u8; ELRS_MSP_BUFFER],
    pub MspDataLength: u8,
// #endif

}

impl CRSF {
    /// todo: Constructor of default impl instead of mutating?
    pub fn set_defaults(&mut self) {
        /// UART Handling ///
        self.SerialInPacketLen = 0; // length of the CRSF packet as measured
        self.SerialInPacketPtr = 0; // index where we are reading/writing
        self.CRSFframeActive = false; //since we get a copy of the serial data use this flag to know when to ignore it
        
        self.GoodPktsCountResult = 0;
        self.BadPktsCountResult = 0;
        
        self.modelId = 0;
        self.ForwardDevicePings = false;
        self.elrsLUAmode = false;
        
        /// OpenTX mixer sync ///
        self.OpenTXsyncLastSent = 0;
        self.RequestedRCpacketInterval = 5000; // default to 200hz as per 'normal'
        self.RCdataLastRecv = 0;
        self.OpenTXsyncOffset = 0;
        self.OpentxSyncActive = true;
        self.OpenTXsyncOffsetSafeMargin = 4000; // 400us
        
        /// UART Handling ///
        self.GoodPktsCount = 0;
        self.BadPktsCount = 0;
        self.UARTwdtLastChecked;
        
        self.CRSFoutBuffer[CRSF_MAX_PACKET_LEN] = [0];
        self.maxPacketBytes = CRSF_MAX_PACKET_LEN;
        self.maxPeriodBytes = CRSF_MAX_PACKET_LEN;
        self.TxToHandsetBauds = [400000, 115200, 5250000, 3750000, 1870000, 921600];
        self.UARTcurrentBaudIdx = 0;
        
        self.CRSFstate = false;
        self.MspData[ELRS_MSP_BUFFER] = [0];
        self.MspDataLength = 0;
    }
    
    fn GetMaxPacketBytes(&self) -> u8 {
        self.maxPacketBytes
    }
    fn GetCurrentBaudRate(&self) -> u32 {
        self.TxToHandsetBauds[self.UARTcurrentBaudIdx]
    }
    
    fn End() {
// #if CRSF_TX_MODULE
    let startTime: u32 = millis();
    while SerialOutFIFO.peek() > 0
    {
        handleUARTin();
        if millis() - startTime > 1000
        {
            break;
        }
    }
// #endif // CRSF_TX_MODULE
}

fn flush_port_input(&self) {
    // Make sure there is no garbage on the UART at the start
    while self.Port.available()
    {
        self.Port.read();
    }
}

// #if CRSF_TX_MODULE
fn sendLinkStatisticsToTX(&self)
{
    if !self.CRSFstate
    {
        return;
    }

   let outBuffer: [u8; 4] = [
        LinkStatisticsFrameLength + 4,
        CRSF_ADDRESS_RADIO_TRANSMITTER,
        LinkStatisticsFrameLength + 2,
        CRSF_FRAMETYPE_LINK_STATISTICS,
    ];

    let mut crc: u8 = crsf_crc.calc(outBuffer[3]);
    crc = crsf_crc.calc((byte *)&LinkStatistics, LinkStatisticsFrameLength, crc);

    if SerialOutFIFO.ensure(outBuffer[0] + 1)
    {
        SerialOutFIFO.pushBytes(outBuffer, sizeof(outBuffer));
        SerialOutFIFO.pushBytes((byte *)&LinkStatistics, LinkStatisticsFrameLength);
        SerialOutFIFO.push(crc);
    }
}

/**
 * Build a an extended type packet and queue it in the SerialOutFIFO
 * This is just a regular packet with 2 extra bytes with the sub src and target
 **/
fn packetQueueExtended(&self, type_: u8, void *data, len: u8)
{
    if !self.CRSFstate {
        return;
    }

    let buf: [u8; 6] = [
        (uint8_t)(len + 6),
        CRSF_ADDRESS_RADIO_TRANSMITTER,
        (uint8_t)(len + 4),
        type,
        CRSF_ADDRESS_RADIO_TRANSMITTER,
        CRSF_ADDRESS_CRSF_TRANSMITTER
    ];

    // CRC - Starts at type, ends before CRC
    uint8_t crc = crsf_crc.calc(&buf[3], sizeof(buf)-3);
    crc = crsf_crc.calc((byte *)data, len, crc);

#ifdef PLATFORM_ESP32
    portENTER_CRITICAL(&FIFOmux);
#endif
    if (SerialOutFIFO.ensure(buf[0] + 1))
    {
        SerialOutFIFO.pushBytes(buf, sizeof(buf));
        SerialOutFIFO.pushBytes((byte *)data, len);
        SerialOutFIFO.push(crc);
    }
#ifdef PLATFORM_ESP32
    portEXIT_CRITICAL(&FIFOmux);
#endif
}

fn sendTelemetryToTX(&self, data: &[u8])
{
    if (data[CRSF_TELEMETRY_LENGTH_INDEX] > CRSF_PAYLOAD_SIZE_MAX)
    {
        ERRLN("too large");
        return;
    }

    if self.CRSFstate
    {
        data[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
#ifdef PLATFORM_ESP32
        portENTER_CRITICAL(&FIFOmux);
#endif
        uint8_t size = CRSF_FRAME_SIZE(data[CRSF_TELEMETRY_LENGTH_INDEX]);
        if (SerialOutFIFO.ensure(size + 1))
        {
            SerialOutFIFO.push(size); // length
            SerialOutFIFO.pushBytes(data, size);
        }
#ifdef PLATFORM_ESP32
        portEXIT_CRITICAL(&FIFOmux);
#endif
    }
}

void ICACHE_RAM_ATTR CRSF::setSyncParams(uint32_t PacketInterval)
{
    CRSF::RequestedRCpacketInterval = PacketInterval;
    adjustMaxPacketSize();
}

uint32_t ICACHE_RAM_ATTR CRSF::GetRCdataLastRecv()
{
    return CRSF::RCdataLastRecv;
}

void ICACHE_RAM_ATTR CRSF::JustSentRFpacket()
{
    CRSF::OpenTXsyncOffset = micros() - CRSF::RCdataLastRecv;

    if (CRSF::OpenTXsyncOffset > (int32_t)CRSF::RequestedRCpacketInterval) // detect overrun case when the packet arrives too late and caculate negative offsets.
    {
        CRSF::OpenTXsyncOffset = -(CRSF::OpenTXsyncOffset % CRSF::RequestedRCpacketInterval);
    }
    //DBGLN("%d, %d", CRSF::OpenTXsyncOffset, CRSF::OpenTXsyncOffsetSafeMargin / 10);
}

fn disableOpentxSync()
{
    OpentxSyncActive = false;
}

fn enableOpentxSync()
{
    OpentxSyncActive = true;
}

void ICACHE_RAM_ATTR CRSF::sendSyncPacketToTX() // in values in us.
{
    uint32_t now = millis();
    if (CRSF::CRSFstate && (now - OpenTXsyncLastSent) >= OpenTXsyncPacketInterval)
    {
        uint32_t packetRate = CRSF::RequestedRCpacketInterval * 10; //convert from us to right format
        int32_t offset = CRSF::OpenTXsyncOffset * 10 - CRSF::OpenTXsyncOffsetSafeMargin; // + 400us offset that that opentx always has some headroom

        struct otxSyncData {
            uint8_t extendedType; // CRSF_FRAMETYPE_OPENTX_SYNC
            uint32_t rate; // Big-Endian
            uint32_t offset; // Big-Endian
        } PACKED;

        uint8_t buffer[sizeof(otxSyncData)];
        struct otxSyncData * const sync = (struct otxSyncData * const)buffer;

        sync->extendedType = CRSF_FRAMETYPE_OPENTX_SYNC;
        sync->rate = htobe32(packetRate);
        sync->offset = htobe32(offset);

        packetQueueExtended(CRSF_FRAMETYPE_RADIO_ID, buffer, sizeof(buffer));

        OpenTXsyncLastSent = now;
    }
}

fn GetChannelDataIn(&self)
{
     // data is packed as 11 bits per channel
    let rcChannels: Channels = self.inBuffer.asRCPacket_t.channels;
    let prev_AUX1: u16 = ChannelDataIn[4];

    ChannelDataIn[0] = (rcChannels.ch0);
    ChannelDataIn[1] = (rcChannels.ch1);
    ChannelDataIn[2] = (rcChannels.ch2);
    ChannelDataIn[3] = (rcChannels.ch3);
    ChannelDataIn[4] = (rcChannels.ch4);
    ChannelDataIn[5] = (rcChannels.ch5);
    ChannelDataIn[6] = (rcChannels.ch6);
    ChannelDataIn[7] = (rcChannels.ch7);
    ChannelDataIn[8] = (rcChannels.ch8);
    ChannelDataIn[9] = (rcChannels.ch9);
    ChannelDataIn[10] = (rcChannels.ch10);
    ChannelDataIn[11] = (rcChannels.ch11);
    ChannelDataIn[12] = (rcChannels.ch12);
    ChannelDataIn[13] = (rcChannels.ch13);
    ChannelDataIn[14] = (rcChannels.ch14);
    ChannelDataIn[15] = (rcChannels.ch15);
}

fn ProcessPacket(&self) -> bool {
    let mut packetReceived = false;

    if CRSFstate == false
    {
        CRSFstate = true;
        DBGLN("CRSF UART Connected");
        connected();
    }

    let packetType: u8 = CRSF::inBuffer.asRCPacket_t.header.type_;
    let SerialInBuffer: u8 = self.inBuffer.asUint8_t;

    if packetType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED {
        CRSF::RCdataLastRecv = micros();
        GetChannelDataIn();
        packetReceived = true;
    }
    // check for all extended frames that are a broadcast or a message to the FC
    else if (packetType >= CRSF_FRAMETYPE_DEVICE_PING &&
            (SerialInBuffer[3] == CRSF_ADDRESS_FLIGHT_CONTROLLER || SerialInBuffer[3] == CRSF_ADDRESS_BROADCAST || SerialInBuffer[3] == CRSF_ADDRESS_CRSF_RECEIVER))
    {
        // Some types trigger telemburst to attempt a connection even with telm off
        // but for pings (which are sent when the user loads Lua) do not forward
        // unless connected
        if (ForwardDevicePings || packetType != CRSF_FRAMETYPE_DEVICE_PING)
        {
            const uint8_t length = CRSF::inBuffer.asRCPacket_t.header.frame_size + 2;
            AddMspMessage(length, SerialInBuffer);
        }
        packetReceived = true;
    }

    // always execute this check since broadcast needs to be handeled in all cases
    if ((SerialInBuffer[3] == CRSF_ADDRESS_CRSF_TRANSMITTER || SerialInBuffer[3] == CRSF_ADDRESS_BROADCAST) &&
        (SerialInBuffer[4] == CRSF_ADDRESS_RADIO_TRANSMITTER || SerialInBuffer[4] == CRSF_ADDRESS_ELRS_LUA))
    {
        elrsLUAmode = SerialInBuffer[4] == CRSF_ADDRESS_ELRS_LUA;

        if (packetType == CRSF_FRAMETYPE_COMMAND && SerialInBuffer[5] == SUBCOMMAND_CRSF && SerialInBuffer[6] == COMMAND_MODEL_SELECT_ID)
        {
            modelId = SerialInBuffer[7];
            #if defined(PLATFORM_ESP32)
            rtcModelId = modelId;
            #endif
            RecvModelUpdate();
        }
        else
        {
            ParameterUpdateData[0] = packetType;
            ParameterUpdateData[1] = SerialInBuffer[5];
            ParameterUpdateData[2] = SerialInBuffer[6];
            RecvParameterUpdate();
        }

        packetReceived = true;
    }

    return packetReceived;
}

fn GetMspMessage(data: &mut [u8], len: &mut u8)
{
    *len = MspDataLength;
    *data = if MspDataLength > 0 { MspData } else { [0; 69] }; // nullptr instad of 0 in orig
}

fn ResetMspQueue()
{
    MspWriteFIFO.flush();
    MspDataLength = 0;
    memset(MspData, 0, ELRS_MSP_BUFFER);
}

fn UnlockMspMessage(&mut self)
{
    // current msp message is sent so restore next buffered write
    if MspWriteFIFO.peek() > 0 {
        let length: u8 = MspWriteFIFO.pop();
        self.MspDataLength = length;
        MspWriteFIFO.popBytes(MspData, length);
    }
    else
    {
        // no msp message is ready to send currently
        self.MspDataLength = 0;
        memset(MspData, 0, ELRS_MSP_BUFFER);
    }
}

fn AddMspMessage(mspPacket_t* packet) {
    if packet.payloadSize > ENCAPSULATED_MSP_MAX_PAYLOAD_SIZE {
        return;
    }

    let totalBufferLen: u8 = packet.payloadSize + ENCAPSULATED_MSP_HEADER_CRC_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_NOT_COUNTED_BYTES;
    uint8_t outBuffer[ENCAPSULATED_MSP_MAX_FRAME_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_NOT_COUNTED_BYTES];

    // CRSF extended frame header
    outBuffer[0] = CRSF_ADDRESS_BROADCAST;                                      // address
    outBuffer[1] = packet.payloadSize + ENCAPSULATED_MSP_HEADER_CRC_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC; // length
    outBuffer[2] = CRSF_FRAMETYPE_MSP_WRITE;                                    // packet type
    outBuffer[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER;                              // destination
    outBuffer[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;                              // origin

    // Encapsulated MSP payload
    outBuffer[5] = 0x30;                // header
    outBuffer[6] = packet.payloadSize; // mspPayloadSize
    outBuffer[7] = packet.function;    // packet->cmd

    for i in 0..packet.payloadSize {
        // copy packet payload into outBuffer
        outBuffer[8 + i] = packet.payload[i];
    }
    // Encapsulated MSP crc
    outBuffer[totalBufferLen - 2] = CalcCRCMsp(&outBuffer[6], packet->payloadSize + 2);

    // CRSF frame crc
    outBuffer[totalBufferLen - 1] = crsf_crc.calc(&outBuffer[2], packet->payloadSize + ENCAPSULATED_MSP_HEADER_CRC_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC - 1);
    AddMspMessage(totalBufferLen, outBuffer);
}

fn AddMspMessage(&self, length: u8, data: &[u8]) {
    if length > ELRS_MSP_BUFFER
    {
        return;
    }

    // store next msp message
    if MspDataLength == 0 {
        for in in 0..length {
            MspData[i] = data[i];
        }
        MspDataLength = length;
    }
    // store all write requests since an update does send multiple writes
    else
    {
        if MspWriteFIFO.ensure(length + 1)       {
            MspWriteFIFO.push(length);
            MspWriteFIFO.pushBytes(data, length);
        }
    }
}

// fn handleUARTin(&mut self) {
//     uint8_t *SerialInBuffer = CRSF::inBuffer.asUint8_t;
//
//     if UARTwdt()
//     {
//         return;
//     }
//
//     while self.Port.available() {
//         if CRSFframeActive == false {
//             let inChar: char = self.Port.read();
//             // stage 1 wait for sync byte //
//             if inChar == CRSF_ADDRESS_CRSF_TRANSMITTER ||
//                 inChar == CRSF_SYNC_BYTE
//             {
//                 // we got sync, reset write pointer
//                 self.SerialInPacketPtr = 0;
//                 SerialInPacketLen = 0;
//                 CRSFframeActive = true;
//                 SerialInBuffer[SerialInPacketPtr] = inChar;
//                 SerialInPacketPtr++;
//             }
//         }
//         else // frame is active so we do the processing
//         {
//             // first if things have gone wrong //
//             if SerialInPacketPtr > CRSF_MAX_PACKET_LEN - 1 {
//                 // we reached the maximum allowable packet length, so start again because shit fucked up hey.
//                 SerialInPacketPtr = 0;
//                 SerialInPacketLen = 0;
//                 CRSFframeActive = false;
//                 return;
//             }
//
//             // special case where we save the expected pkt len to buffer //
//             if (SerialInPacketPtr == 1)
//             {
//                 unsigned char const inChar = CRSF::Port.read();
//                 if (inChar <= CRSF_MAX_PACKET_LEN)
//                 {
//                     SerialInPacketLen = inChar;
//                     SerialInBuffer[SerialInPacketPtr] = inChar;
//                     SerialInPacketPtr++;
//                 }
//                 else
//                 {
//                     SerialInPacketPtr = 0;
//                     SerialInPacketLen = 0;
//                     CRSFframeActive = false;
//                     return;
//                 }
//             }
//
//             int toRead = (SerialInPacketLen + 2) - SerialInPacketPtr;
//             #if defined(PLATFORM_ESP32)
//             int count = CRSF::Port.read(&SerialInBuffer[SerialInPacketPtr], toRead);
//             #else
//             int count = 0;
//             int avail = CRSF::Port.available();
//             while (count < toRead && count < avail)
//             {
//                 SerialInBuffer[SerialInPacketPtr + count] = CRSF::Port.read();
//                 count++;
//             }
//             #endif
//             SerialInPacketPtr += count;
//
//             if (SerialInPacketPtr >= (SerialInPacketLen + 2)) // plus 2 because the packlen is referenced from the start of the 'type' flag, IE there are an extra 2 bytes.
//             {
//                 char CalculatedCRC = crsf_crc.calc(SerialInBuffer + 2, SerialInPacketPtr - 3);
//
//                 if (CalculatedCRC == SerialInBuffer[SerialInPacketPtr-1])
//                 {
//                     GoodPktsCount++;
//                     if (ProcessPacket())
//                     {
//                         //delayMicroseconds(50);
//                         handleUARTout();
//                         RCdataCallback();
//                     }
//                 }
//                 else
//                 {
//                     DBGLN("UART CRC failure");
//                     // cleanup input buffer
//                     flush_port_input();
//                     BadPktsCount++;
//                 }
//                 CRSFframeActive = false;
//                 SerialInPacketPtr = 0;
//                 SerialInPacketLen = 0;
//             }
//         }
//     }
// }

// void ICACHE_RAM_ATTR CRSF::handleUARTout()
// {
//     // both static to split up larger packages
//     static uint8_t packageLengthRemaining = 0;
//     static uint8_t sendingOffset = 0;
//
//     if (OpentxSyncActive)
//     {
//         sendSyncPacketToTX(); // calculate mixer sync packet if needed
//     }
//
//     // if partial package remaining, or data in the output FIFO that needs to be written
//     if (packageLengthRemaining > 0 || SerialOutFIFO.size() > 0) {
//         duplex_set_TX();
//
//         uint8_t periodBytesRemaining = maxPeriodBytes;
//         while (periodBytesRemaining)
//         {
// #ifdef PLATFORM_ESP32
//             portENTER_CRITICAL(&FIFOmux); // stops other tasks from writing to the FIFO when we want to read it
// #endif
//             // no package is in transit so get new data from the fifo
//             if (packageLengthRemaining == 0) {
//                 packageLengthRemaining = SerialOutFIFO.pop();
//                 SerialOutFIFO.popBytes(CRSFoutBuffer, packageLengthRemaining);
//                 sendingOffset = 0;
//             }
// #ifdef PLATFORM_ESP32
//             portEXIT_CRITICAL(&FIFOmux); // stops other tasks from writing to the FIFO when we want to read it
// #endif
//
//             // if the package is long we need to split it up so it fits in the sending interval
//             uint8_t writeLength;
//             if (packageLengthRemaining > periodBytesRemaining) {
//                 if (periodBytesRemaining < maxPeriodBytes) {  // only start to send a split packet as the first packet
//                     break;
//                 }
//                 writeLength = periodBytesRemaining;
//             } else {
//                 writeLength = packageLengthRemaining;
//             }
//
//             // write the packet out, if it's a large package the offset holds the starting position
//             CRSF::Port.write(CRSFoutBuffer + sendingOffset, writeLength);
//             if (CRSF::PortSecondary)
//                 CRSF::PortSecondary->write(CRSFoutBuffer + sendingOffset, writeLength);
//
//             sendingOffset += writeLength;
//             packageLengthRemaining -= writeLength;
//             periodBytesRemaining -= writeLength;
//
//             // No bytes left to send, exit
//             if (SerialOutFIFO.size() == 0)
//                 break;
//         }
//         CRSF::Port.flush();
//         duplex_set_RX();
//
//         // make sure there is no garbage on the UART left over
//         flush_port_input();
//     }
// }

void ICACHE_RAM_ATTR CRSF::duplex_set_RX()
{
#if defined(PLATFORM_ESP32)
  #if (GPIO_PIN_RCSIGNAL_TX == GPIO_PIN_RCSIGNAL_RX)
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_MODE_INPUT));
    #ifdef UART_INVERTED
    gpio_matrix_in((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, U0RXD_IN_IDX, true);
    gpio_pulldown_en((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
    gpio_pullup_dis((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
    #else
    gpio_matrix_in((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, U0RXD_IN_IDX, false);
    gpio_pullup_en((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
    gpio_pulldown_dis((gpio_num_t)GPIO_PIN_RCSIGNAL_RX);
    #endif
  #endif
#elif defined(PLATFORM_ESP8266)
    // Enable loopback on UART0 to connect the RX pin to the TX pin
    //USC0(UART0) |= BIT(UCLBE);
#elif defined(GPIO_PIN_BUFFER_OE) && (GPIO_PIN_BUFFER_OE != UNDEF_PIN)
    digitalWrite(GPIO_PIN_BUFFER_OE, LOW ^ GPIO_PIN_BUFFER_OE_INVERTED);
#elif (GPIO_PIN_RCSIGNAL_TX == GPIO_PIN_RCSIGNAL_RX)
    CRSF::Port.enableHalfDuplexRx();
#endif
}

void ICACHE_RAM_ATTR CRSF::duplex_set_TX()
{
#if defined(PLATFORM_ESP32)
  #if (GPIO_PIN_RCSIGNAL_TX == GPIO_PIN_RCSIGNAL_RX)
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)GPIO_PIN_RCSIGNAL_RX, GPIO_FLOATING));
    #ifdef UART_INVERTED
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, 0));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_MODE_OUTPUT));
    constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30; // routes 0 to matrix slot
    gpio_matrix_in(MATRIX_DETACH_IN_LOW, U0RXD_IN_IDX, false); // Disconnect RX from all pads
    gpio_matrix_out((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, U0TXD_OUT_IDX, true, false);
    #else
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, 1));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, GPIO_MODE_OUTPUT));
    constexpr uint8_t MATRIX_DETACH_IN_HIGH = 0x38; // routes 1 to matrix slot
    gpio_matrix_in(MATRIX_DETACH_IN_HIGH, U0RXD_IN_IDX, false); // Disconnect RX from all pads
    gpio_matrix_out((gpio_num_t)GPIO_PIN_RCSIGNAL_TX, U0TXD_OUT_IDX, false, false);
    #endif
  #endif
#elif defined(PLATFORM_ESP8266)
    // Disable loopback to disconnect the RX pin from the TX pin
    //USC0(UART0) &= ~BIT(UCLBE);
#elif defined(GPIO_PIN_BUFFER_OE) && (GPIO_PIN_BUFFER_OE != UNDEF_PIN)
    digitalWrite(GPIO_PIN_BUFFER_OE, HIGH ^ GPIO_PIN_BUFFER_OE_INVERTED);
#elif (GPIO_PIN_RCSIGNAL_TX == GPIO_PIN_RCSIGNAL_RX)
    // writing to the port switches the mode
#endif
}

void ICACHE_RAM_ATTR CRSF::adjustMaxPacketSize()
{
    uint32_t UARTrequestedBaud = TxToHandsetBauds[UARTcurrentBaudIdx];
    // baud / 10bits-per-byte / 2 windows (1RX, 1TX) / rate * 0.80 (leeway)
    maxPeriodBytes = UARTrequestedBaud / 10 / 2 / (1000000/RequestedRCpacketInterval) * 80 / 100;
    maxPeriodBytes = maxPeriodBytes > HANDSET_TELEMETRY_FIFO_SIZE ? HANDSET_TELEMETRY_FIFO_SIZE : maxPeriodBytes;
    // we need a minimum of 10 bytes otherwise our LUA will not make progress and at 8 we'd get a divide by 0!
    maxPeriodBytes = maxPeriodBytes < 10 ? 10 : maxPeriodBytes;
    maxPacketBytes = maxPeriodBytes > CRSF_MAX_PACKET_LEN ? CRSF_MAX_PACKET_LEN : maxPeriodBytes;
    DBGLN("Adjusted max packet size %u-%u", maxPacketBytes, maxPeriodBytes);
}

fn UARTwdt() -> bool {
    let mut retval = false;
// #if !defined(DEBUG_TX_FREERUN)
//     uint32_t now = millis();
//     if (now >= (UARTwdtLastChecked + UARTwdtInterval))
//     {
//         if (BadPktsCount >= GoodPktsCount)
//         {
//             DBGLN("Too many bad UART RX packets!");
//
//             if (CRSFstate == true)
//             {
//                 DBGLN("CRSF UART Disconnected");
//                 disconnected();
//                 CRSFstate = false;
//             }
//
//             UARTcurrentBaudIdx = (UARTcurrentBaudIdx + 1) % ARRAY_SIZE(TxToHandsetBauds);
//             uint32_t UARTrequestedBaud = TxToHandsetBauds[UARTcurrentBaudIdx];
//             DBGLN("UART WDT: Switch to: %d baud", UARTrequestedBaud);
//
//             adjustMaxPacketSize();
//
//             SerialOutFIFO.flush();
// #if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)
//             CRSF::Port.flush();
//             CRSF::Port.updateBaudRate(UARTrequestedBaud);
// #elif defined(TARGET_TX_GHOST)
//             CRSF::Port.begin(UARTrequestedBaud);
//             USART1->CR1 &= ~USART_CR1_UE;
//             USART1->CR3 |= USART_CR3_HDSEL;
//             USART1->CR2 |= USART_CR2_RXINV | USART_CR2_TXINV | USART_CR2_SWAP; //inverted/swapped
//             USART1->CR1 |= USART_CR1_UE;
// #elif defined(TARGET_TX_FM30_MINI)
//             CRSF::Port.begin(UARTrequestedBaud);
//             LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_2, LL_GPIO_PULL_DOWN); // default is PULLUP
//             USART2->CR1 &= ~USART_CR1_UE;
//             USART2->CR2 |= USART_CR2_RXINV | USART_CR2_TXINV; //inverted
//             USART2->CR1 |= USART_CR1_UE;
// #else
//             CRSF::Port.begin(UARTrequestedBaud);
// #endif
//             duplex_set_RX();
//             // cleanup input buffer
//             flush_port_input();
//
//             retval = true;
//         }
//         DBGLN("UART STATS Bad:Good = %u:%u", BadPktsCount, GoodPktsCount);
//
//         UARTwdtLastChecked = now;
//         if (retval)
//         {
//             // Speed up the cycling
//             UARTwdtLastChecked -= 3 * (UARTwdtInterval >> 2);
//         }
//
//         GoodPktsCountResult = GoodPktsCount;
//         BadPktsCountResult = BadPktsCount;
//         BadPktsCount = 0;
//         GoodPktsCount = 0;
//     }
// #endif
    retval
}

// #elif CRSF_RX_MODULE // !CRSF_TX_MODULE
fn RXhandleUARTout() -> bool
{
    let mut retVal = false;
// #if !defined(CRSF_RCVR_NO_SERIAL)
    // don't write more than 128 bytes at a time to avoid RX buffer overflow
    const int maxBytesPerCall = 128;
    uint32_t bytesWritten = 0;

    while SerialOutFIFO.size() > SerialOutFIFO.peek() && (bytesWritten + SerialOutFIFO.peek()) < maxBytesPerCall
    {
        noInterrupts();
        let OutPktLen: u8 = SerialOutFIFO.pop();
        let mut OutData: [u8; OutPktLen] = [0; OutPktLen];
        SerialOutFIFO.popBytes(OutData, OutPktLen);
        interrupts();
        // this->_dev->write(OutData, OutPktLen); // write the packet out
        bytesWritten += OutPktLen;
        retVal = true;
    }
// #endif // CRSF_RCVR_NO_SERIAL
    retVal
}

fn sendLinkStatisticsToFC()
{
#if !defined(CRSF_RCVR_NO_SERIAL) && !defined(DEBUG_CRSF_NO_OUTPUT)
    constexpr uint8_t outBuffer[4] = {
        LinkStatisticsFrameLength + 4,
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        LinkStatisticsFrameLength + 2,
        CRSF_FRAMETYPE_LINK_STATISTICS
    };

    let mut crc: u8 = crsf_crc.calc(outBuffer[3]);
    crc = crsf_crc.calc((byte *)&LinkStatistics, LinkStatisticsFrameLength, crc);

    if (SerialOutFIFO.ensure(outBuffer[0] + 1)) {
        SerialOutFIFO.pushBytes(outBuffer, sizeof(outBuffer));
        SerialOutFIFO.pushBytes((byte *)&LinkStatistics, LinkStatisticsFrameLength);
        SerialOutFIFO.push(crc);
    }

    //this->_dev->write(outBuffer, LinkStatisticsFrameLength + 4);
#endif // CRSF_RCVR_NO_SERIAL
}

fn sendRCFrameToFC(&self)
{
// #if !defined(CRSF_RCVR_NO_SERIAL) && !defined(DEBUG_CRSF_NO_OUTPUT)
    constexpr uint8_t outBuffer[] = {
        // No need for length prefix as we aren't using the FIFO
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        RCframeLength + 2,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED
    };

    let mut crc: u8 = crsf_crc.calc(outBuffer[2]);
    crc = crsf_crc.calc((byte *)&PackedRCdataOut, RCframeLength, crc);

    //SerialOutFIFO.push(RCframeLength + 4);
    //SerialOutFIFO.pushBytes(outBuffer, RCframeLength + 4);
    // this.dev->write(outBuffer, sizeof(outBuffer));
    // this.dev->write((byte *)&PackedRCdataOut, RCframeLength);
    // this.dev->write(crc);
// #endif // CRSF_RCVR_NO_SERIAL
}

fn sendMSPFrameToFC(data: &[u8])
{
// #if !defined(CRSF_RCVR_NO_SERIAL) && !defined(DEBUG_CRSF_NO_OUTPUT)
    lettotalBufferLen:  u8 = CRSF_FRAME_SIZE(data[1]);
    if totalBufferLen <= CRSF_FRAME_SIZE_MAX
    {
        data[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
        // this._dev.write(data, totalBufferLen);
    }
// #endif // CRSF_RCVR_NO_SERIAL
}

/**
 * @brief   Get encoded channel position from PackedRCdataOut
 * @param   ch: zero-based channel number
 * @return  CRSF-encoded channel position, or 0 if invalid channel
 **/
fn GetChannelOutput(&self, ch: u8) -> u16 {
    match ch {
        0 => self.PackedRCdataOut.ch0,
        1 => self.PackedRCdataOut.ch1,
        2 => self.PackedRCdataOut.ch2,
        3 => self.PackedRCdataOut.ch3,
        4 => self.PackedRCdataOut.ch4,
        5 => self.PackedRCdataOut.ch5,
        6 => self.PackedRCdataOut.ch6,
        7 => self.PackedRCdataOut.ch7,
        8 => self.PackedRCdataOut.ch8,
        9 => self.PackedRCdataOut.ch9,
        10 => self.PackedRCdataOut.ch10,
        11 => self.PackedRCdataOut.ch11,
        _ => 0
    }
}

// #endif // CRSF_RX_MODULE

// fn GetDeviceInformation(frame: &[u8], fieldCount: &[u8]) {
//     deviceInformationPacket_t *device = (deviceInformationPacket_t *)(frame + sizeof(crsf_ext_header_t) + device_name_size);
//     // Packet starts with device name
//     memcpy(frame + sizeof(crsf_ext_header_t), device_name, device_name_size);
//     // Followed by the device
//     device->serialNo = htobe32(0x454C5253); // ['E', 'L', 'R', 'S'], seen [0x00, 0x0a, 0xe7, 0xc6] // "Serial 177-714694" (value is 714694)
//     device->hardwareVer = 0; // unused currently by us, seen [ 0x00, 0x0b, 0x10, 0x01 ] // "Hardware: V 1.01" / "Bootloader: V 3.06"
//     device->softwareVer = 0; // unused currently by us, seen [ 0x00, 0x00, 0x05, 0x0f ] // "Firmware: V 5.15"
//     device->fieldCnt = fieldCount;
//     device->parameterVersion = 0;
// }

fn SetHeaderAndCrc(&self, frame: &[u8],frameType: u8,frameSize: u8, destAddr: u8)
{
    crsf_header_t *header = (crsf_header_t *)frame;
    header.device_addr = destAddr;
    header.frame_size = frameSize;
    header.type = frameType;

    uint8_t crc = crsf_crc.calc(&frame[CRSF_FRAME_NOT_COUNTED_BYTES], frameSize - 1, 0);
    frame[frameSize + CRSF_FRAME_NOT_COUNTED_BYTES - 1] = crc;
}

fnSetExtendedHeaderAndCrc(uint8_t *frame, uint8_t frameType, uint8_t frameSize, uint8_t senderAddr, uint8_t destAddr)
{
    crsf_ext_header_t *header = (crsf_ext_header_t *)frame;
    header->dest_addr = destAddr;
    header->orig_addr = senderAddr;
    SetHeaderAndCrc(frame, frameType, frameSize, destAddr);
}


}

// extern GENERIC_CRC8 crsf_crc;

// #endif

// UART0 is used since for DupleTX we can connect directly through IO_MUX and not the Matrix
// for better performance, and on other targets (mostly using pin 13), it always uses Matrix
HardwareSerial CRSF::Port(0);
portMUX_TYPE FIFOmux = portMUX_INITIALIZER_UNLOCKED;

RTC_DATA_ATTR int rtcModelId = 0;

GENERIC_CRC8 crsf_crc(CRSF_CRC_POLY);


///Out FIFO to buffer messages///
static FIFO SerialOutFIFO;

volatile uint16_t CRSF::ChannelDataIn[16] = {0};

inBuffer_U CRSF::inBuffer;

volatile crsfPayloadLinkstatistics_s CRSF::LinkStatistics;

volatile uint8_t CRSF::ParameterUpdateData[3] = {0};

static FIFO MspWriteFIFO;


// for the UART wdt, every 1000ms we change bauds when connect is lost
const UARTwdtInterval: u16 = 1_000; // todo: define type?


// #endif // CRSF_TX_MODULE

// #ifdef CRSF_RX_MODULE
crsf_channels_s CRSF::PackedRCdataOut;
// #endif

