/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

// note that this file avoids any use of raw ethernet

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>
#include <stdio.h>


#include <Amp1394/AmpIORevision.h>
#ifndef _MSC_VER
#include <termios.h>
#endif
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_EMIO
#include "ZynqEmioPort.h"
#endif
#include "EthUdpPort.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"
#include "MotorVoltage.h"
#include "BasePort.h"

const uint32_t VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const uint32_t DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */
const uint32_t PWR_ENABLE_MASK  = 0x00080000;  /*!< Power enable mask (write only) */
const uint32_t PWR_ENABLE_BIT   = 0x00040000;  /*!< Power enable status (read/write) */
const uint32_t PWR_ENABLE       = PWR_ENABLE_MASK|PWR_ENABLE_BIT;
const uint32_t PWR_DISABLE      = PWR_ENABLE_MASK;
const uint32_t RELAY_FB         = 0x00020000;  /*!< Safety relay feedback (read only), 0 for DQLA */
const uint32_t RELAY_MASK       = 0x00020000;  /*!< Safety relay enable mask (write only) */
const uint32_t RELAY_BIT        = 0x00010000;  /*!< Safety relay enable (read/write) */
const uint32_t RELAY_ON         = RELAY_MASK|RELAY_BIT;
const uint32_t RELAY_OFF        = RELAY_MASK;


// CRC16-CCIT (also called CRC-ITU)
const uint16_t crc16_table[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint32_t KSZ8851CRC(const unsigned char *data, size_t len)
{
    uint32_t crc = 0xffffffff;
    for (size_t i = 0; i < len; i++) {
        for (size_t j = 0; j < 8; j++) {
            if (((crc >> 31) ^ (data[i] >> j)) & 0x01)
                crc = (crc << 1) ^ 0x04c11db7;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

// Compute parameters to initialize multicast hash table
void ComputeMulticastHash(unsigned char *MulticastMAC, uint8_t &regAddr, uint16_t &regData)
{
    uint32_t crc = KSZ8851CRC(MulticastMAC, 6);
    int regOffset = (crc >> 29) & 0x0006;  // first 2 bits of CRC (x2)
    int regBit = (crc >> 26) & 0x00F;      // next 4 bits of CRC
    regAddr = 0xA0 + regOffset;            // 0xA0 --> MAHTR0 (MAC Address Hash Table Register 0)
    regData = (1 << regBit);
}

// Ethernet status from FPGA register 12
void PrintEthernetStatus(AmpIO &Board)
{
    uint32_t status;
    if (Board.ReadEthernetStatus(status))
        EthBasePort::PrintStatus(std::cout, status);
}

// Check contents of KSZ8851 register
bool CheckRegister(AmpIO &Board, uint8_t regNum, uint16_t mask, uint16_t value)
{
    uint16_t reg;
    Board.ReadKSZ8851Reg(regNum, reg);
    if ((reg&mask) != value) {
        std::cout << "Register " << std::hex << (int)regNum << ": read = " << reg
                  << ", expected = " << value << " (mask = " << mask << ")" << std::endl;
        return false;
    }
    return true;
}

// Check whether Ethernet initialized correctly
bool CheckEthernetV2(AmpIO &Board)
{
    std::cout << "Checking --- start ---" << "\n";
    bool ret = true;
    ret &= CheckRegister(Board, 0x10, 0xfff0, 0x9400);  // MAC address low = 0x940n (n = board id)
    ret &= CheckRegister(Board, 0x12, 0xffff, 0x0E13);  // MAC address middle = 0xOE13
    ret &= CheckRegister(Board, 0x14, 0xffff, 0xFA61);  // MAC address high = 0xFA61
    ret &= CheckRegister(Board, 0x84, 0x4000, 0x4000);  // Enable QMU transmit frame data pointer auto increment
    ret &= CheckRegister(Board, 0x70, 0x01ff, 0x01EF);  // Enable QMU transmit flow control, CRC, padding, and transmit module
    ret &= CheckRegister(Board, 0x86, 0x4000, 0x4000);  // Enable QMU receive frame data pointer auto increment
    ret &= CheckRegister(Board, 0x9C, 0x00ff, 0x0001);  // Configure receive frame threshold for 1 frame
    ret &= CheckRegister(Board, 0x74, 0xffff, 0x7CE1);  // Enable checksums, MAC address filtering, and receive module
    // Check multicast hash table
    unsigned char MulticastMAC[6];
    EthBasePort::GetDestMulticastMacAddr(MulticastMAC);
    uint8_t HashReg;
    uint16_t HashValue;
    ComputeMulticastHash(MulticastMAC, HashReg, HashValue);
    ret &= CheckRegister(Board, HashReg, 0xffff, HashValue);
    ret &= CheckRegister(Board, 0x82, 0x03f7, 0x0020);  // Enable QMU frame count threshold (1), no auto-dequeue
    ret &= CheckRegister(Board, 0x90, 0xffff, 0xa000);  // Enable receive and link change interrupts
    std::cout << "Checking ---- end ----" << "\n";
    return ret;
}

bool CheckRTL8211F_RegIO(AmpIO &Board, unsigned int chan, unsigned int phyAddr)
{
    uint16_t curPage;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, curPage)) {
        std::cout << "Failed to read PHY" << chan << " PAGSR" << std::endl;
        return false;
    }
    if (curPage != FpgaIO::RTL8211F_PAGE_DEFAULT) {
        std::cout << std::hex << "Changing page from " << curPage << " to " << FpgaIO::RTL8211F_PAGE_DEFAULT << std::endl;
        if (!Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, FpgaIO::RTL8211F_PAGE_DEFAULT)) {
            std::cout << "Failed to write PHY" << chan << " PAGSR" << std::endl;
            return false;
        }
    }
    // Perform walking bit test on INER (interrupt enable register, 18)
    uint16_t curIner, mask_read;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, curIner)) {
        std::cout << "Failed to read PHY" << chan << " INER" << std::endl;
        return false;
    }
    std::cout << "Testing RTL8211F PHY" << chan << " Register I/O, initial value = " << std::hex << curIner << std::endl;
    bool allOK = true;
    for (uint16_t mask = 0x1; mask != 0; mask <<= 1) {
        Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, mask);
        Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, mask_read);
        if (mask != mask_read) {
            std::cout << "Error: wrote " << mask << ", read " << mask_read << std::endl;
            allOK = false;
            //break;
        }
    }
    std::cout << std::dec << "Test complete" << std::endl;
    // Restore original values
    Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, curIner);
    Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, curPage);
    return allOK;
}

bool CheckEthernetV3(AmpIO &Board, unsigned int chan)
{
    unsigned int phyAddr = FpgaIO::PHY_RTL8211F;

    // Check Register I/O
    if (!CheckRTL8211F_RegIO(Board, chan, phyAddr))
        return false;

    // Check PHYID1 and PHYID2 (assumes we are on correct page)
    uint16_t phyid1 = 0, phyid2 = 0;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PHYID1, phyid1))
        std::cout << "Failed to read PHY" << chan << " PHYID1" << std::endl;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PHYID2, phyid2))
        std::cout << "Failed to read PHY" << chan << " PHYID2" << std::endl;
    std::cout << "PHY" << chan << std::hex
              << " PHYID1: " << std::setw(4) << std::setfill('0') << phyid1 << " (should be 001c),"
              << " PHYID2: " << std::setw(4) << std::setfill('0') << phyid2 << " (should be c916)"
              << std::dec << std::endl;

    // Now, check undocumented registers 0x11 (17) and 0x15 (21) on page 0xd08
    //   Register 17, Bit 8 (0x0100) indicates state of TX_DELAY
    //   Register 21, Bit 3 (0x0008) indicates state of RX_DELAY
    if (!Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, 0xd08))
        std::cout << "Failed to set PHY" << chan << " PAGSR to 0xd08" << std::endl;
    uint16_t phyreg17, phyreg21;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, 17, phyreg17))
        std::cout << "Failed to read PHY" << chan << " Page 0xd08, Reg 17" << std::endl;
    std::cout << "PHY" << chan << std::hex << " Page 0xd08, Register 17: " << phyreg17 << std::dec << std::endl;
    std::cout << "Tx Delay: " << ((phyreg17 & 0x0100) ? "ON" : "OFF") << std::endl;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, 21, phyreg21))
        std::cout << "Failed to read PHY" << chan << " Page 0xd08, Reg 21" << std::endl;
    std::cout << "PHY" << chan << std::hex << " Page 0xd08, Register 21: " << phyreg21 << std::dec << std::endl;
    std::cout << "Rx Delay: " << ((phyreg21 & 0x0008) ? "ON" : "OFF") << std::endl;

    // Restore default page
    if (!Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, FpgaIO::RTL8211F_PAGE_DEFAULT))
        std::cout << "Failed to write PHY" << chan << " PAGSR" << std::endl;

    // Check GMII to RGMII core PHY register
    // Based on the VHDL source code for this core, PHY Specific Control Register 1 should be at address 16
    uint16_t phyCR;
    if (!Board.ReadRTL8211F_Register(chan, FpgaIO::PHY_GMII_CORE, 16, phyCR)) {
        std::cout << "Failed to read GMII PHY" << chan << " Reg 16" << std::endl;
    }
    else {
        std::cout << std::hex << "GMII PHY Reg 16: " << phyCR << std::dec << std::endl;
    }

    return (phyid1 == 0x001c) && (phyid2 == 0xc916);
}

// eth_port:  0 for FPGA V2, 1 or 2 for FPGA V3
bool InitEthernet(AmpIO &Board, unsigned int eth_port) {
    if (Board.GetFirmwareVersion() < 5) {
        std::cout << "   No Ethernet controller, firmware version = " << Board.GetFirmwareVersion() << std::endl;
        return false;
    }

    unsigned int fpga_ver = Board.GetFpgaVersionMajor();
    if (fpga_ver < 2) {
        std::cout << "   No Ethernet in FPGA V" << fpga_ver << std::endl;
        return false;
    }
    PrintEthernetStatus(Board);

    // Reset the board
    Board.WriteEthernetPhyReset(eth_port);
    if (fpga_ver == 2) {
        // Wait 100 msec
        Amp1394_Sleep(0.1);
    }
    else {
        // Wait 500 msec
        Amp1394_Sleep(0.5);
    }

    // Read the status
    uint32_t status;
    Board.ReadEthernetStatus(status);
    std::cout << "   After reset, status = " << std::hex << ((fpga_ver == 2) ? (status>>16) : status) << std::endl;

    if (fpga_ver == 2) {
        if (!(status & FpgaIO::ETH_STAT_INIT_OK_V2)) {
            std::cout << "   Ethernet V2 failed initialization" << std::endl;
            EthBasePort::PrintStatus(std::cout, status);
            return false;
        }

        // Read the Chip ID (16-bit read)
        uint16_t chipID = Board.ReadKSZ8851ChipID();
        std::cout << "   Chip ID = " << std::hex << chipID << std::endl;
        if ((chipID&0xfff0) != 0x8870)
            return false;


        // Check that KSZ8851 registers are as expected
        if (!CheckEthernetV2(Board)) {
            PrintEthernetStatus(Board);
            return false;
        }

        // Display the MAC address
        uint16_t regLow, regMid, regHigh;
        Board.ReadKSZ8851Reg(0x10, regLow);   // MAC address low = 0x94nn (nn = board id)
        Board.ReadKSZ8851Reg(0x12, regMid);   // MAC address middle = 0xOE13
        Board.ReadKSZ8851Reg(0x14, regHigh);  // MAC address high = 0xFA61
        std::cout << "   MAC address = " << std::hex << regHigh << ":" << regMid << ":" << regLow << std::endl;
    }
    else if (fpga_ver == 3) {
        uint8_t portStatus = FpgaIO::GetEthernetPortStatusV3(status, eth_port);
        if (!(portStatus & FpgaIO::ETH_PORT_STAT_INIT_OK)) {
            std::cout << "   Ethernet V3 failed initialization" << std::endl;
            EthBasePort::PrintStatus(std::cout, status);
            return false;
        }

        // Check that RTL8211F registers are as expected
        if (!CheckEthernetV3(Board, eth_port)) {
            PrintEthernetStatus(Board);
            return false;
        }
    }

    // Wait 2.5 sec
    Amp1394_Sleep(2.5);
    return true;
}

bool InitFireWire(BasePort *&FwPort, std::vector<AmpIO *> &FwBoardList, int port) {
    #if Amp1394_HAS_RAW1394
        FwPort = new FirewirePort(port, std::cout);
        if (!FwPort->IsOK()) {
            std::cout << "Failed to initialize firewire port" << std::endl;
            return false;
        }
        for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++) {
            if (FwPort->GetNodeId(bnum) != BasePort::MAX_NODES) {
                std::cout << "Found Firewire board: " << bnum << std::endl;
                AmpIO *board = new AmpIO(bnum);
                FwPort->AddBoard(board);
                FwBoardList.push_back(board);
            }
        }
        return true;
    #else
        return false;
    #endif
}


/****************************************************************
*   @brief  Initialize a Zynq-EMIO communication port and discover boards
*
*   @details
*   Creates a new `ZynqEmioPort` object for the specified *port* number.
*   If the port opens successfully the routine:
*     1. Enables or disables verbose output according to *isVerbose*.
*     2. Assigns the newly-created object to the caller-provided reference
*        `ZyncPort`.
*     3. Queries node 0 (the only node on an EMIO link) for its board ID.
*        When the ID is in the valid range, the function constructs an
*        `AmpIO` handle for that board, registers it with the port, and
*        appends the pointer to *ZyncBoardList*.
*
*   @param  BasePort *&        ZyncPort        Reference that receives the
*                                              newly created `ZynqEmioPort`
*                                              pointer on success
*   @param  std::vector<AmpIO *>& ZyncBoardList
*                                              Vector to which discovered
*                                              `AmpIO` board objects are
*                                              appended
*   @param  int                port            Device index of the Zynq
*                                              EMIO interface (usually 0)
*   @param  bool               isVerbose       Enables verbose printing if
*                                              set to *true*
*
*   @note   The routine is compiled only when `Amp1394_HAS_EMIO` is defined.
*           When that macro is absent the stub always returns *false*.
*
*   @return bool
*           *true*  — Port initialised (even if no board detected)  
*           *false* — Port failed to open or EMIO support not available
****************************************************************/
bool InitZync(BasePort *&ZyncPort, std::vector<AmpIO *> &ZyncBoardList, int port, bool isVerbose) {
    #if Amp1394_HAS_EMIO
        ZynqEmioPort *ZynqPort;
        ZynqPort = new ZynqEmioPort(port, std::cout);
        if (!ZynqPort->IsOK()) {
            std::cout << "Failed to initialize Zynq EMIO port" << std::endl;
            return false;
        }
        ZynqPort->SetVerbose(isVerbose);
        ZyncPort = ZynqPort;
        // Zynq EMIO port always has one node (0)
        unsigned int bnum = ZyncPort->GetBoardId(0);
        if (bnum < BoardIO::MAX_BOARDS) {
            std::cout << "Found Zynq EMIO board: " << bnum << std::endl;
            AmpIO *board = new AmpIO(bnum);
            ZyncPort->AddBoard(board);
            ZyncBoardList.push_back(board);
        }
            return true;
    #else
        return false;
    #endif
}

/****************************************************************
*   @brief this function continuously reads from the same register 
*   using two different communication methods to test for glitches
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the value read by the methods are 
*   different and some statistics along the way and at the end
*
*   @return none
****************************************************************/
void ReadSameRegisterTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t compareFailures = 0;
    unsigned long count = 0;

    while (!done) {
        count++;
        read_data_port_one = 0;
        read_data_port_two = 0;
        if (portone->ReadQuadlet(boardNum, 4, read_data_port_one) && porttwo->ReadQuadlet(boardNum, 4, read_data_port_two)) {
            if (!(memcmp((void *)&read_data_port_one, (void *)&read_data_port_two, 4))) {
                success++;
            } else {
                compareFailures++;;
            }
        }

        if (compareFailures > 20) {
            done = true;
        } 

        if (count >= 10000) {
            done = true;
        }

        // print status
        if (count % 1000 == 0) {
            std::cout << "attempts = " << std::dec << count << ", success = " << success << ", compare failures = "
                      << compareFailures << std::endl;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << ", compare failures = "
                  << compareFailures << std::endl;
}

/****************************************************************
*   @brief This function tests for glitches when writing and
*   reading using different communication methods but to the
*   same register
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the value written and read are 
*   different and some statistics along the way and at the end
*   @note assumes the encoder preload in channel 1 exists (offset 4) for
*   the given board
*   @note accounts for reading and writing failures by not evaluating cases
*   when those occur
*
*   @return none
****************************************************************/
void WriteAndReadOneRegisterDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    // setup variables and test information
    bool done = false;
    quadlet_t read_data;
    size_t success = 0;
    size_t compareFailures = 0;
    quadlet_t write_data = 0x0;
    int count = 0;
    nodeaddr_t regnum = 0x14;  // Channel 1 preload (was 0x0F for REG_DEBUG)

    // continuously write and read and look for glithes
    while (!done) {
        read_data = -1;
        write_data++;
        count++;
        if (portone->WriteQuadlet(boardNum, regnum, write_data)) {
            Amp1394_Sleep(0*1e-6);  // sleep 0 us; ADJUSTABLE; this might get rid of the glitch when adjusted
            if (porttwo->ReadQuadlet(boardNum, regnum, read_data)) {
                if (memcmp((void *)&read_data, (void *)&write_data, 4)) {
                    compareFailures++;
                    std::cout << std::hex << "write_data = 0x" << write_data << "  " << " read_data = 0x" << read_data << std::endl;
                } else {
                    success++;
                }
            }
        }
        
        // end conditions and data tracking
        if (compareFailures > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << ", compare failures = " << compareFailures << std::endl;
        }

        if (count >= 10000) {
            done = true;
            std::cout << "end write_data = 0x" << std::hex << write_data << "\n";
        }
    }
}

/****************************************************************
*   @brief: identical to WriteAndReadOneRegisterDifferentMethodsTest,
*   but also ensures that both ports then read the same value.
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the value written and read are 
*   different and some statistics along the way and at the end
*   @note assumes the encoder preload in channel 1 exists (offset 4) for
*   the given board
*   @note accounts for reading and writing failures by not evaluating cases
*   when those occur
*
*   @return none
****************************************************************/
void WriteAndReadOneRegisterDifferentMethodsStressTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    // setup variables and test information
    bool done = false;
    quadlet_t read_data_one;
    quadlet_t read_data_two;
    size_t success = 0;
    size_t compareFailures = 0;
    quadlet_t write_data = 0x0;
    int count = 0;
    nodeaddr_t regnum = 0x14;  // Channel 1 preload (was 0x0F for REG_DEBUG)

    // continuously write and read and look for glithes
    while (!done) {
        read_data_one = -1;
        write_data++;
        count++;
        if (portone->WriteQuadlet(boardNum, regnum, write_data)) {
            Amp1394_Sleep(50*1e-6);  // sleep 50 us; ADJUSTABLE; this might get rid of the glitch
            if (porttwo->ReadQuadlet(boardNum, regnum, read_data_two) && portone->ReadQuadlet(boardNum, regnum, read_data_one)) {
                if (memcmp((void *)&read_data_one, (void *)&write_data, 4) || memcmp((void *)&read_data_one, (void *)&read_data_two, 4)) {
                    compareFailures++;
                    std::cout << std::hex << "write_data = 0x" << write_data << "  " << " read_data_one = 0x" << read_data_one << std::endl;
                    std::cout << std::hex << "read_data_one = 0x" << read_data_one << "  " << " read_data_two = 0x" << read_data_two << std::endl;
                } else {
                    success++;
                }
            }
        }
        
        // end conditions and data tracking
        if (compareFailures > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << ", compare failures = " << compareFailures << std::endl;
        }

        if (count >= 10000) {
            done = true;
            std::cout << "end write_data = 0x" << std::hex << write_data << "\n";
        }
    }
}

// good notes until ehre confirmed!!!

/****************************************************************
*   @brief tests that two communication methods can read the status 
*   register at the same time 
*
*   @details ensures safety relay and power are on before looping
*   having two different ports read the status at very close times 
*   and then comparing their results, which is an attempt to isolate 
*   for glitches caused exclusively by quick read with different 
*   communication methods.
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the values read are not the expected values
*   and some statistics along the way and at the end
*   @note this test does not gaurentee anything except that reading
*   alone at the same time does not cause issues. Any other circumstances
*   may change the results
*   @note this test does track how many times the actual attempt to read
*   also fails (i.e. a call to ReadQuadlet returns false), which may
*   indicate trouble with reading at the same time as well.
*
*   @return none
****************************************************************/
void ReadDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    size_t success = 0;
    quadlet_t status_result_one;
    quadlet_t status_result_two;
    size_t compareFailures = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadRead = 0;
    
    // ensure power is enabled and safety relay is on
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_ENABLE);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON);
    Amp1394_Sleep(50*1e-6);

    while (!done) {
        count++;
        // attempt for both to read
        if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_one) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_two)) {
            if (memcmp((void *)&status_result_one, (void *)&status_result_two, 4)) {
                compareFailures++;
                std::cout << portOneString << " read status read of " << std::hex << status_result_one << " " << portTwoString << " read status read of " << status_result_two << "\n";
            } else {
                success++;
            }
        } else {
            failedQuadRead++;
        }

        // end conditions and data tracking
        if (compareFailures > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "comapare failures = " << compareFailures << "\n";
    std::cout << "unsucessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";

    // ensure power is disabled and safety relay are off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*   @brief tests that two communication methods can write to enable the
*   safety relay and power at similar times
*
*   @details ensures safety relay and power are off each time before 
*   looping and having two different ports write at very close times,
*   waiting, and then reading, which is an attempt to isolate for glitches
*   caused exclusively by quick writes with different methods    
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the values read are not the expected values
*   and some statistics along the way and at the end
*   @note this test does track how many times the actual attempt to write
*   also fails (i.e. a call to WriteQuadlet returns false), which may
*   indicate trouble with writing at the same time as well.
*
*   @return none
****************************************************************/
void WriteDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    quadlet_t status_result;
    size_t compareFailuresPortOne = 0;
    size_t compareFailuresPortTwo = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    
    while (!done) {
        count++;
        iteration_success = true;
        // ensure power is disabled and safety relay are off
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
        Amp1394_Sleep(50*1e-6);
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);

        // attempt for both to write
        if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_ENABLE) && porttwo->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON)) {
            Amp1394_Sleep(50*1e-6);
            if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result)) {
                // reads that portone enabled power
                if (!(status_result & PWR_ENABLE_BIT)) {
                    compareFailuresPortOne++;
                    iteration_success = false;
                    std::cout << "Enabled power with " << portOneString << " but power is read as disabled \n";
                }
                // reads that porttwo enabled safety relay
                if (!(status_result & RELAY_BIT)) {
                    compareFailuresPortTwo++;
                    iteration_success = false;
                    std::cout << "Enabled safety relay with " << portTwoString<< " but power is read as disabled \n";
                }
                if (iteration_success) {
                    success++;
                }
            }
        } else {
            failedQuadWrite++;
        }
        
        // end conditions and data tracking
        if (compareFailuresPortOne + compareFailuresPortTwo > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }
    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with " << portOneString << std::dec << compareFailuresPortOne << "failures with " << portTwoString << compareFailuresPortTwo << "\n";
    std::cout << "unsucessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";

    // ensure power is disabled and safety relay are off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*   @brief: tests that two communication methods can write to enable the
*   safety relay and power at similar times and then read the results at
*   similar times
*
*   @details: ensures safety relay and power are off each time before 
*   looping and having two different ports write at very close times,
*   waiting, and then reading with both ports at very close times, which 
*   is an attempt to isolate for glitches that occur because of writing 
*   and then reading very quickly with different methods
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the values read do not match the written
*   values and some statistics along the way and at the end
*   @note: this test does track how many times the actual attempt to write
*   also fails (i.e. a call to WriteQuadlet returns false), which may
*   indicate trouble with writing at the same time as well.
*   @note: this test does track how many times the actual attempt to read
*   also fails (i.e. a call to ReadQuadlet returns false), which may
*   indicate trouble with reading at the same time as well.
*   @note: each type of failure can occur twice per iteration.
*
*   @return none
****************************************************************/
void WriteThenReadDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    quadlet_t status_result;
    size_t compareReadFailuresPortOne = 0;
    size_t compareReadFailuresPortTwo = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one;
    quadlet_t status_result_two;
    size_t compareFailures = 0;
    size_t failedQuadRead = 0;
    
    while (!done) {
        count++;
        iteration_success = true;
        // ensure power is disabled and safety relay are off each time
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
        Amp1394_Sleep(50*1e-6);
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);

        // attempt for both to write and then both to read in quick succession
        if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_ENABLE) && porttwo->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON)) {
            if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_one) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_two)) {
                // portone reads that portone enabled power
                if (!(status_result_one & PWR_ENABLE_BIT)) {
                    compareReadFailuresPortOne++;
                    iteration_success = false;
                    std::cout << "Enabled power with " << portOneString << " but power is read as disabled by " << portOneString << "\n";
                }

                // portone reads that porttwo enabled safety relay
                if (!(status_result_one & RELAY_BIT)) {
                    compareReadFailuresPortOne++;
                    iteration_success = false;
                    std::cout << "Enabled safety relay with " << portTwoString << " but power is read as disabled by " << portOneString << "\n";
                }

                // porttwo reads that portone enabled power
                if (!(status_result_two & PWR_ENABLE_BIT)) {
                    compareReadFailuresPortTwo++;
                    iteration_success = false;
                    std::cout << "Enabled power with " << portOneString << " but power is read as disabled by " << portTwoString << "\n";
                }

                // porttwo reads that porttwo enabled safety relay
                if (!(status_result_two & RELAY_BIT)) {
                    compareReadFailuresPortTwo++;
                    iteration_success = false;
                    std::cout << "Enabled safety relay with " << portTwoString << " but power is read as disabled by " << portTwoString << "\n";
                }
                if (iteration_success) {
                    success++;
                }
            } else {
                failedQuadRead++;
            }
        } else {
            failedQuadWrite++;
        }
        
        // end conditions and data tracking
        if (compareReadFailuresPortOne + compareReadFailuresPortTwo > 400) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with " << portOneString << std::dec << compareReadFailuresPortOne << "failures with " << portTwoString << compareReadFailuresPortTwo << "\n";
    std::cout << "unsucessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsucessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure power is disabled and safety relay are off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*   @brief: tests that two communication methods can read the status and
*   and then enable the safety relay without messing up the intial reads.
*
*   @details: ensures safety relay is off each time before looping and 
*   having two different ports read at very close times, and then writing
*   with one port at a very close time to turn on the safety relay, and
*   attempting to read again with both communication methods at very close
*   times, which is an attempt to isolate for glitches that occur because 
*   of reading and then writing very quickly with different methods
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the value written and read are 
*   not the expected values
*   @note: this test does track how many times the actual attempt to write
*   also fails (i.e. a call to WriteQuadlet returns false), which may
*   indicate trouble with writing at the same time as well.
*   @note: this test does track how many times the actual attempt to read
*   also fails (i.e. a call to ReadQuadlet returns false), which may
*   indicate trouble with reading at the same time as well.
*   @note: each type of failure can occur twice per iteration.
*
*   @return none
****************************************************************/
void ReadThenWriteDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    quadlet_t status_result;
    size_t compareReadFailuresPortOne = 0;
    size_t compareReadFailuresPortTwo = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one_stage_one;
    quadlet_t status_result_two_stage_one;
    quadlet_t status_result_stage_two;
    size_t compareFailuresIntial = 0;
    size_t compareFailuresSecond = 0;
    size_t compareFailuresIntialVsSecond = 0;
    size_t failedQuadRead = 0;
    
    while (!done) {
        count++;
        iteration_success = true;
        // ensure power is disabled and safety relay are off each time
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);

        if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_one_stage_one) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_two_stage_one)) {
            if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON)) {
                Amp1394_Sleep(50*1e-6);
                if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_stage_two)) {
                    Amp1394_Sleep(50*1e-6);
                    // check initial reads are equal
                    if (status_result_one_stage_one != status_result_two_stage_one) {
                        std::cout << "Initial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", initial status reading from " << portTwoString << ": " << status_result_two_stage_one << "\n";
                        compareFailuresIntial++;
                        iteration_success = false;
                    }

                    // check inital and second reads are not equal
                    if (status_result_stage_two == status_result_one_stage_one || status_result_stage_two == status_result_two_stage_one) {
                        std::cout << "Intial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        std::cout << "Intial Status reading from " << portTwoString << ": " << std::hex << status_result_two_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        compareFailuresIntialVsSecond++;
                        iteration_success = false;
                    }
                    if (iteration_success) {
                        success++;
                    }
                } else {
                    failedQuadRead++;
                }
            } else {
                failedQuadWrite++;
            }
        } else {
            failedQuadRead++;
        }
        
        // end conditions and data tracking
        if (compareFailuresIntial + compareFailuresIntialVsSecond > 400) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with initial readings not matching: " << std::dec << compareFailuresIntial << "\n";
    std::cout << "failures with first and second readings matching: " << std::dec << compareFailuresIntialVsSecond << "\n";
    std::cout << "unsucessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsucessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure safety relay is off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*   @brief: tests that two communication methods can read the status and
*   and then enable the safety relay and then read the status again.
*
*   @details: ensures safety relay is off each time before looping and 
*   having two different ports read at very close times, and then writing
*   with one port at a very close time to turn on the safety relay, and
*   attempting to read again with both communication methods at very close
*   times, which is an attempt to isolate for glitches that occur because 
*   of a rapid read-write-read rapid sequence with different communication 
*   methods.
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note: this test does track how many times the actual attempt to write
*   also fails (i.e. a call to WriteQuadlet returns false), which may
*   indicate trouble with writing at the same time as well.
*   @note: this test does track how many times the actual attempt to read
*   also fails (i.e. a call to ReadQuadlet returns false), which may
*   indicate trouble with reading at the same time as well.
*   @note: each type of failure can occur twice per iteration.
*
*   @return none
****************************************************************/
void RapidReadWriteReadDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    quadlet_t status_result;
    size_t compareReadFailuresPortOne = 0;
    size_t compareReadFailuresPortTwo = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one_stage_one;
    quadlet_t status_result_two_stage_one;
    quadlet_t status_result_one_stage_two;
    quadlet_t status_result_two_stage_two;
    size_t compareFailuresIntial = 0;
    size_t compareFailuresSecond = 0;
    size_t compareFailuresIntialVsSecond = 0;
    size_t failedQuadRead = 0;
    
    while (!done) {
        count++;
        iteration_success = true;
        // ensure power is disabled and safety relay are off each time
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);

        if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_one_stage_one) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_two_stage_one)) {
            if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON)) {
                if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_one_stage_two) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_two_stage_two)) {
                    // check initial reads are equal
                    if (status_result_one_stage_one != status_result_two_stage_one) {
                        std::cout << "Initial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", initial status reading from " << portTwoString << ": " << status_result_two_stage_one << "\n";
                        compareFailuresIntial++;
                        iteration_success = false;
                    }

                    // check second reads are equal
                    if (status_result_one_stage_two != status_result_two_stage_two) {
                        std::cout << "Second Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_two << ", second status reading from " << portTwoString << ": " << status_result_two_stage_two << "\n";
                        compareFailuresSecond++;
                        iteration_success = false;
                    }

                    // check inital and second reads are not equal
                    if (status_result_one_stage_two == status_result_one_stage_one || status_result_two_stage_two == status_result_two_stage_one) {
                        std::cout << "Intial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", second status reading from " << portOneString << ": " << status_result_one_stage_two << "\n";
                        std::cout << "Intial Status reading from " << portTwoString << ": " << std::hex << status_result_two_stage_one << ", second status reading from " << portTwoString << ": " << status_result_two_stage_two << "\n";
                        compareFailuresIntialVsSecond++;
                        iteration_success = false;
                    }
                    if (iteration_success) {
                        success++;
                    }
                } else {
                    failedQuadRead++;
                }
            } else {
                failedQuadWrite++;
            }
        } else {
            failedQuadRead++;
        }
        
        // end conditions and data tracking
        if (compareFailuresIntial + compareFailuresSecond + compareFailuresIntialVsSecond > 400) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with initial readings not matching: " << std::dec << compareFailuresIntial << "\n";
    std::cout << "failures with second readings not matching: " << std::dec << compareFailuresSecond << "\n";
    std::cout << "failures with first and second readings matching: " << std::dec << compareFailuresIntialVsSecond << "\n";
    std::cout << "unsucessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsucessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure safety relay is off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*   @brief: tests that two communication methods can disable the safety 
*   relay and then read the status and then write the status and
*   and then enable the safety relay
*
*   @details: ensures safety relay is on each time before looping and 
*   having one port write at very close time to both ports reading, and 
*   then writing again to the relay all in quick sucession and then 
*   reading afterwards, which is an attempt to isolate for glitches that 
*   occur because of a rapid write-read-write sequence with different 
*   communication methods.
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param unsignedchar (boardNum): Board number to use
*
*   @note Outputs a message when the value written and read are 
*   different and some statistics along the way and at the end
*   @note: this test does track how many times the actual attempt to write
*   also fails (i.e. a call to WriteQuadlet returns false), which may
*   indicate trouble with writing at the same time as well.
*   @note: this test does track how many times the actual attempt to read
*   also fails (i.e. a call to ReadQuadlet returns false), which may
*   indicate trouble with reading at the same time as well.
*   @note: each type of failure can occur twice per iteration.
*
*   @return none
****************************************************************/
void RapidWriteReadWriteDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    quadlet_t status_result;
    size_t compareReadFailuresPortOne = 0;
    size_t compareReadFailuresPortTwo = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one_stage_one;
    quadlet_t status_result_two_stage_one;
    quadlet_t status_result_stage_two;
    size_t compareFailuresIntial = 0;
    size_t compareFailuresIntialVsSecond = 0;
    size_t failedQuadRead = 0;
    
    while (!done) {
        count++;
        iteration_success = true;
        // ensure safety relay is on each time
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON);
        Amp1394_Sleep(50*1e-6);
        if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF)) {
            if (portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_one_stage_one) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_two_stage_one)) {
                if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON)) {
                    Amp1394_Sleep(50*1e-6);
                    portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_result_stage_two);
                    Amp1394_Sleep(50*1e-6);

                    // check initial reads are equal
                    if (status_result_one_stage_one != status_result_two_stage_one) {
                        std::cout << "Initial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", initial status reading from " << portTwoString << ": " << status_result_two_stage_one << "\n";
                        compareFailuresIntial++;
                        iteration_success = false;
                    }

                    // check inital and second reads are not equal
                    if (status_result_stage_two == status_result_one_stage_one || status_result_stage_two == status_result_two_stage_one) {
                        std::cout << "Intial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        std::cout << "Intial Status reading from " << portTwoString << ": " << std::hex << status_result_two_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        compareFailuresIntialVsSecond++;
                        iteration_success = false;
                    }
                    if (iteration_success) {
                        success++;
                    }
                } else {
                    failedQuadWrite++;
                }
            } else {
                failedQuadRead++;
            }
        } else {
            failedQuadWrite++;
        }
        
        // end conditions and data tracking
        if (compareFailuresIntial + compareFailuresIntialVsSecond > 400) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with initial readings not matching: " << std::dec << compareFailuresIntial << "\n";
    std::cout << "failures with first and second readings matching: " << std::dec << compareFailuresIntialVsSecond << "\n";
    std::cout << "unsucessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsucessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure safety relay is off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*   @brief: This function tests for glitches when writing and
*   reading with the waveform with different communication methods
*  
*   @details: Outputs a message when the value written and read are 
*   different
*
*   @param BasePort     (*portone): First communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param BasePort     (*porttwo): Second communication interface object 
*                                   that provides access to the physical 
*                                   connection to the controller boards   
*   @param AmpIO          (*board): board object of the interface for the
*                                   board being used in this function
*
*   @note rereads the waveform a second time to see if any glitches
*   corrupt the waveform or are temporary
*
*   @return none
****************************************************************/
void WaveformReadAndWriteDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum, AmpIO *board) {
    const unsigned int WLEN = 256;
    quadlet_t waveform[WLEN];
    quadlet_t waveform_read[WLEN];
    size_t i;
    // Set up square waves that change every 1 msec
    double clkPer = board->GetFPGAClockPeriod();
    uint32_t numTicks = static_cast<uint32_t>(0.001/clkPer);
    std::cout << "Setting waveform with " << numTicks << " counts (1 msec edges)" << std::endl;
    if ((numTicks&0x7fffff) != numTicks)
        std::cout << "Warning: numTicks does not fit in 23 bits" << std::endl;
    unsigned char dout1 = 0;
    unsigned char dout2 = 0;
    unsigned char dout3 = 0;
    unsigned char dout4 = 0;
    for (i = 0; i < WLEN-1; i++) {
        if (i%4 == 0) dout1 = 1-dout1;
        if (i%4 == 1) dout2 = 1-dout2;
        if (i%4 == 2) dout3 = 1-dout3;
        if (i%4 == 3) dout4 = 1-dout4;
        waveform[i] = 0x80000000 | (numTicks << 8) | (dout4 << 3) | (dout3 << 2) | (dout2 << 1) | dout1;
        waveform_read[i] = 0;
    }
    waveform[WLEN-1] = 0;
    waveform_read[WLEN-1] = 0;
    std::cout << "Writing test pattern" << std::endl;

    // explicitly write the waveform table to directly use portone to write (code based on AmpIO.cpp)
    if (portone->GetFirmwareVersion(boardNum) < 7 || portone->GetHardwareVersion(boardNum) == dRA1_String) {
        std::cout << "Writing to waveform table failed" << std::endl;
        return;
    }
    if (WLEN > (portone->GetMaxWriteDataSize()/sizeof(quadlet_t))) {
        std::cout << "Writing to waveform table failed" << std::endl;
        return;
    }
    static quadlet_t localBuffer[MAX_POSSIBLE_DATA_SIZE/sizeof(quadlet_t)];
    nodeaddr_t address = 0x8000;
    for (unsigned short i = 0; i < WLEN; i++) {
        localBuffer[i] = bswap_32(waveform[i]^0x0000000f);
    }
    if (!(portone->WriteBlock(boardNum, address, localBuffer, WLEN*sizeof(quadlet_t)))) {
        std::cout << "Writing to waveform table failed" << std::endl;
        return;
    }

    // adjustable - might fix any glitch
    // Amp1394_Sleep(0.0);
   
    std::cout << "Reading data";
    // read once to test glitch 
     if (porttwo->GetFirmwareVersion(boardNum) < 7 || porttwo->GetHardwareVersion(boardNum) == dRA1_String) {
         std::cout << "Reading from waveform table failed" << std::endl;
        return;
    }
    if (WLEN > (porttwo->GetMaxReadDataSize()/sizeof(quadlet_t))) {
        std::cout << "Reading from waveform table failed" << std::endl;
        return;
    }
    bool ret = porttwo->ReadBlock(boardNum, address, waveform_read, WLEN*sizeof(quadlet_t));
    if (ret) {
        // Byteswap and invert digital output bits (see WriteDigitalOutput and GetDigitalOutput)
        for (unsigned short i = 0; i < WLEN; i++) {
            waveform_read[i] = bswap_32(waveform_read[i])^0x0000000f;
        }
    } else {
        std::cout << "Reading from waveform table failed" << std::endl;
        return;
    }

    bool mismatch = false;
    for (i = 0; i < WLEN; i++) {
        if (waveform_read[i] != waveform[i]) {
            std::cout << "Mismatch at quadlet " << i << ", read " << std::hex
                        << waveform_read[i] << ", expected " << waveform[i]
                        << std::dec << std::endl;
            mismatch = true;
        }
    }

    // if there was a mismatch above, read again after waiting to see if everything has been 
    // corrected or waveform table becomes corrupted
    if (mismatch) {
        Amp1394_Sleep(0.05);
        ret = porttwo->ReadBlock(boardNum, address, waveform_read, WLEN*sizeof(quadlet_t));
        if (ret) {
            // Byteswap and invert digital output bits (see WriteDigitalOutput and GetDigitalOutput)
            for (unsigned short i = 0; i < WLEN; i++) {
                waveform_read[i] = bswap_32(waveform_read[i])^0x0000000f;
            }
        } else {
            std::cout << "Reading from waveform table (after waiting) failed" << std::endl;
            return;
        }
        for (i = 0; i < WLEN; i++) {
            if (waveform_read[i] != waveform[i]) {
                std::cout << "Mismatch at quadlet " << i << ", read " << std::hex
                            << waveform_read[i] << ", expected " << waveform[i]
                            << std::dec << std::endl;
            }
        }
    }
}


void ReadRegisterAndStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t compareFailures = 0;
    size_t preload_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t readFailures = 0;
    size_t relay = 0;
    unsigned long count = 0;
    bool preload_correct_flag = false;
    bool relay_correct_flag = false;

    // ensure safety relay is on
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON);
    Amp1394_Sleep(50*1e-6);

    while (!done) {
        read_data_port_one = -1;
        read_data_port_two = -1;
        preload_correct_flag = false;
        relay_correct_flag = false;
        count++;

        // check they both read correctly
        if (!(portone->ReadQuadlet(boardNum, 4, read_data_port_one) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, read_data_port_two))) {
            readFailures++;
        } else {
            if (memcmp((void *)&read_data_port_one, buf, 4) == 0) {
                preload_correct_flag = true;
            }
            if (read_data_port_two & RELAY_BIT) {
                relay_correct_flag = true;
            }

            if (preload_correct_flag && relay_correct_flag) {
                success++;
            } else {
                 std::cout << "Preload read as: " << std::hex << read_data_port_one << " should be 1ALQ";
                if (!preload_correct_flag) {
                    preload_incorrect++;
                }
                if (!relay_correct_flag) {
                    relay_incorrect++;
                    std::cout << "Relay was read as OFF should be ON\n";
                } else {
                    std::cout << "Relay was read as ON should be ON\n";
                }
            }
        }

        // end conditions and data tracking
        if (preload_incorrect + relay_incorrect > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with preload reading: " << std::dec << preload_incorrect << "\n";
    std::cout << "failures with relay reading: " << std::dec << relay_incorrect << "\n";
    std::cout << "unsucessful attempts to read " << std::dec << readFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}


void ReadRegisterWriteStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t compareFailures = 0;
    size_t preload_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t instructionFailures = 0;
    size_t relay = 0;
    unsigned long count = 0;
    bool preload_correct_flag = false;
    bool relay_correct_flag = false;


    while (!done) {
        // ensure safety relay is off each time
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);

        read_data_port_one = -1;
        read_data_port_two = -1;
        preload_correct_flag = false;
        relay_correct_flag = false;
        count++;

        // check they both perform correctly
        if (!(portone->ReadQuadlet(boardNum, 4, read_data_port_one) && porttwo->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON))) {
            instructionFailures++;
        } else {
            Amp1394_Sleep(50*1e-6); // ensure next read is good
            if (porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, read_data_port_two)) {
                
                // now ensure read and write were done correctly 
                if (memcmp((void *)&read_data_port_one, buf, 4) == 0) {
                    preload_correct_flag = true;
                }
                if (read_data_port_two & RELAY_BIT) {
                    relay_correct_flag = true;
                }

                if (memcmp((void *)&read_data_port_one, buf, 4) == 0) {
                    preload_correct_flag = true;
                }
                if (read_data_port_two & RELAY_BIT) {
                    relay_correct_flag = true;
                }

                if (preload_correct_flag && relay_correct_flag) {
                    success++;
                } else {
                    std::cout << "Preload read as: " << std::hex << read_data_port_one << " should be 1ALQ";
                    if (!preload_correct_flag) {
                        preload_incorrect++;
                    }
                    if (!relay_correct_flag) {
                        relay_incorrect++;
                        std::cout << "Relay was read as OFF should be ON\n";
                    } else {
                        std::cout << "Relay was read as ON should be ON\n";
                    }
                }
            } else {
                instructionFailures++;
            }
        }

        // end conditions and data tracking
        if (preload_incorrect + relay_incorrect > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with preload reading: " << std::dec << preload_incorrect << "\n";
    std::cout << "failures with relay writing: " << std::dec << relay_incorrect << "\n";
    std::cout << "unsucessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

void WriteRegisterReadStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    nodeaddr_t regnum = 0x14; 
    quadlet_t write_data = 0x0;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t compareFailures = 0;
    size_t register_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t instructionFailures = 0;
    size_t relay = 0;
    unsigned long count = 0;
    bool register_correct_flag = false;
    bool relay_correct_flag = false;

    // ensure relay is on
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON);
    Amp1394_Sleep(50*1e-6);


    while (!done) {
        read_data_port_one = -1;
        read_data_port_two = -1;
        write_data++;
        count++;

        register_correct_flag = false;
        relay_correct_flag = false;

        if (portone->WriteQuadlet(boardNum, regnum, write_data) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, read_data_port_two)) {
            Amp1394_Sleep(50*1e-6);  // sleep 50 us to ensure next read is good
            if (portone->ReadQuadlet(boardNum, regnum, read_data_port_one)) {
                // ensure everything is good
                if (!memcmp((void *)&read_data_port_one, (void *)&write_data, 4)) {
                    register_correct_flag = true;
                }

                if (read_data_port_two & RELAY_BIT) {
                    relay_correct_flag = true;
                }

                if (register_correct_flag && relay_correct_flag) {
                    success++;
                } else {
                     std::cout << std::hex << "write_data = 0x" << write_data << "  " << " read_data = 0x" << read_data_port_one << std::endl;
                    if (!register_correct_flag) {
                       register_incorrect++;
                    }
                    if (!relay_correct_flag) {
                        relay_incorrect++;
                        std::cout << "Relay was read as OFF should be ON\n";
                    } else {
                        std::cout << "Relay was read as ON should be ON\n";
                    }
                }
            } else {
                instructionFailures++;
            }
        } else {
            instructionFailures++;
        }

        // end conditions and data tracking
        if (register_incorrect + relay_incorrect > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with register writing: " << std::dec << register_incorrect << "\n";
    std::cout << "failures with relay reading: " << std::dec << relay_incorrect << "\n";
    std::cout << "unsucessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

void WriteRegisterWriteStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    nodeaddr_t regnum = 0x14; 
    quadlet_t write_data = 0x0;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t compareFailures = 0;
    size_t register_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t instructionFailures = 0;
    size_t relay = 0;
    unsigned long count = 0;
    bool register_correct_flag = false;
    bool relay_correct_flag = false;


    while (!done) {
        // ensure relay is off each time
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);
        read_data_port_one = -1;
        read_data_port_two = -1;
        write_data++;
        count++;

        register_correct_flag = false;
        relay_correct_flag = false;

        if (portone->WriteQuadlet(boardNum, regnum, write_data) && porttwo->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON)) {
            Amp1394_Sleep(50*1e-6);  // sleep 50 us to ensure next read is good
            if (portone->ReadQuadlet(boardNum, regnum, read_data_port_one)) {
                Amp1394_Sleep(50*1e-6);  // sleep 50 us to ensure next read is good
                if (porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, read_data_port_two)) {
                    if (!memcmp((void *)&read_data_port_one, (void *)&write_data, 4)) {
                        register_correct_flag = true;
                    }
                    if (read_data_port_two & RELAY_BIT) {
                        relay_correct_flag = true;
                    }

                    if (register_correct_flag && relay_correct_flag) {
                        success++;
                    } else {
                        std::cout << std::hex << "write_data = 0x" << write_data << "  " << " read_data = 0x" << read_data_port_one << std::endl;
                        if (!register_correct_flag) {
                        register_incorrect++;
                        }
                        if (!relay_correct_flag) {
                            relay_incorrect++;
                            std::cout << "Relay was read as OFF should be ON\n";
                        } else {
                            std::cout << "Relay was read as ON should be ON\n";
                        }
                    }
                } else {
                    instructionFailures++;
                }
            } else {
                instructionFailures++;
            }
        } else {
            instructionFailures++;
        }

        // end conditions and data tracking
        if (register_incorrect + relay_incorrect > 200) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with register writing: " << std::dec << register_incorrect << "\n";
    std::cout << "failures with relay writing: " << std::dec << relay_incorrect << "\n";
    std::cout << "unsucessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

void ReadRegisterAndStatusStressTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one_first_read;
    quadlet_t read_data_port_two_first_read;
    quadlet_t read_data_port_one_second_read;
    quadlet_t read_data_port_two_second_read;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t compareFailures = 0;
    size_t preload_portone_failures = 0;
    size_t preload_porttwo_failures = 0;
    size_t relay_portone_failures = 0;
    size_t relay_porttwo_failures = 0;
    size_t relay_incorrect = 0;
    size_t readFailures = 0;
    size_t relay = 0;
    unsigned long count = 0;
    bool preload_correct_flag_first_read = false;
    bool relay_correct_flag_first_read = false;
    bool preload_correct_flag_second_read = false;
    bool relay_correct_flag_second_read = false;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();

    // ensure safety relay is on
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON);
    Amp1394_Sleep(50*1e-6);

    while (!done) {
        read_data_port_one_first_read = -1;
        read_data_port_two_first_read = -1;
        read_data_port_one_second_read = -1;
        read_data_port_two_second_read = -1;
        preload_correct_flag_first_read = false;
        relay_correct_flag_first_read = false;
        preload_correct_flag_second_read = false;
        relay_correct_flag_second_read = false;

        count++;

        // check they both read correctly
        if ((portone->ReadQuadlet(boardNum, 4, read_data_port_one_first_read) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, read_data_port_two_first_read))
            && portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, read_data_port_one_second_read) && porttwo->ReadQuadlet(boardNum, 4, read_data_port_two_second_read)) {
            
            // ensure preload read correctly by both
            if (memcmp((void *)&read_data_port_one_first_read, buf, 4) == 0) {
                preload_correct_flag_first_read = true;
            }

            if (memcmp((void *)&read_data_port_two_second_read, buf, 4) == 0) {
                preload_correct_flag_second_read = true;
            }

            // ensure relay read correctly by both
            if (read_data_port_two_first_read & RELAY_BIT) {
                relay_correct_flag_first_read = true;
            }

            if (read_data_port_one_second_read & RELAY_BIT) {
                relay_correct_flag_second_read = true;
            }

            if (preload_correct_flag_first_read && preload_correct_flag_second_read && relay_correct_flag_first_read && relay_correct_flag_second_read) {
                    success++;
            } else {
                // error messages
                std::cout << "Preload read by " << portOneString << " as: " << std::hex << read_data_port_one_first_read << " should be 1ALQ\n";
                std::cout << "Preload read by " << portTwoString << " as: " << std::hex << read_data_port_two_second_read << " should be 1ALQ\n";
                if (!preload_correct_flag_first_read) {
                    preload_portone_failures++;
                }
                if (!preload_correct_flag_second_read) {
                    preload_porttwo_failures++;
                }

                
                if (!relay_correct_flag_second_read) {
                    relay_portone_failures++;
                    std::cout << "Relay was read by " << portOneString << " as OFF should be ON\n";
                } else {
                    std::cout << "Relay was read by " << portOneString << "as ON should be ON\n";
                }

                if (!relay_correct_flag_first_read) {
                    relay_porttwo_failures++;
                    std::cout << "Relay was read by " << portTwoString << " as OFF should be ON\n";
                } else {
                    std::cout << "Relay was read by " << portTwoString << "as ON should be ON\n";
                }
            }
        } else {
            readFailures++;
        }
           

        // end conditions and data tracking
        if (preload_portone_failures + preload_porttwo_failures + relay_portone_failures + relay_porttwo_failures > 400) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with " << portOneString << "preload reading: " << std::dec << preload_portone_failures << "\n";
    std::cout << "failures with " << portTwoString << "preload reading: " << std::dec << preload_porttwo_failures << "\n";
    std::cout << "failures with " << portOneString << "relay reading: " << std::dec << relay_portone_failures << "\n";
    std::cout << "failures with " << portTwoString << "relay reading: " << std::dec << relay_porttwo_failures << "\n";
    std::cout << "unsucessful attempts to read " << std::dec << readFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}


void WriteRegisterAndStatusStressTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t register_read;
    quadlet_t status_read;
    quadlet_t read_data_port_one_second_read;
    quadlet_t read_data_port_two_second_read;
    //char buf[5] = "QLA1";
    nodeaddr_t regnum = 0x14; 
    quadlet_t write_data = 0x0;
    quadlet_t write_data_adj = write_data+1;
    size_t success = 0;
    size_t compareFailures = 0;
    size_t preload_portone_failures = 0;
    size_t preload_porttwo_failures = 0;
    size_t relay_portone_failures = 0;
    size_t relay_porttwo_failures = 0;
    size_t relay_failures = 0;
    size_t power_failures = 0;
    size_t register_failures = 0;
    size_t relay_incorrect = 0;
    size_t instructionFailures = 0;
    size_t relay = 0;
    unsigned long count = 0;
    bool register_correct_flag = false;
    bool relay_correct_flag = false;
    bool power_correct_flag = false;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();

    while (!done) {
        register_read = -1;
        status_read = -1;
        register_correct_flag = false;
        relay_correct_flag = false;
        power_correct_flag = false;
        // ensure power is disabled and safety relay is off
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
        Amp1394_Sleep(50*1e-6);
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);

        write_data++;
        write_data_adj++;
        count++;

        // check they everythign wrote correctly
        if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_ENABLE) && porttwo->WriteQuadlet(boardNum, regnum, write_data)
            && portone->WriteQuadlet(boardNum, regnum, write_data_adj) && porttwo->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON)) {
            // ensure reads are done correctly
            Amp1394_Sleep(50*1e-6);
            portone->ReadQuadlet(boardNum, regnum, register_read);
            Amp1394_Sleep(50*1e-6);
            portone->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_read);
            Amp1394_Sleep(50*1e-6);
            
            // ensure register written correctly
            // note does not ensure that first didn't fail

            if (memcmp((void *)&register_read, &(write_data_adj), 4) == 0) {
                register_correct_flag = true;
            }

            // ensure relay written correctly
            if (status_read & RELAY_BIT) {
                relay_correct_flag = true;
            }

            if (status_read & PWR_ENABLE_BIT) {
                power_correct_flag = true;
            }

            if (power_correct_flag && relay_correct_flag && register_correct_flag) {
                    success++;
            } else {
                // error messages
                std::cout << "Register read as: " << std::hex << register_read << " should be " << (write_data_adj) << "\n";
                if (!register_correct_flag) {
                    register_failures++;
                }

                if (!relay_correct_flag) {
                    relay_failures++;
                    std::cout << "Relay was read as OFF should be ON\n";
                } else {
                    std::cout << "Relay was read as ON should be ON\n";
                }

                if (!power_correct_flag) {
                    power_failures++;
                    std::cout << "Power was read as OFF should be ON\n";
                } else {
                    std::cout << "Power was read as ON should be ON\n";
                }
            }
        } else {
            instructionFailures++;
        }
           
        // end conditions and data tracking
        if (power_failures + relay_failures + register_failures > 300) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with register writing: " << std::dec << register_failures << "\n";
    std::cout << "failures with relay writing: " << std::dec << relay_failures << "\n";
    std::cout << "failures with power writing: " << std::dec << power_failures << "\n";
    std::cout << "unsucessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
}

void AlternatingReadAndWriteRegisterAndStatusStressTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum){
    bool done = false;
    quadlet_t read_data_port_one_first_read;
    quadlet_t read_data_port_two_first_read;
    quadlet_t read_data_port_one_second_read;
    quadlet_t read_data_port_two_second_read;
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t compareFailures = 0;
    size_t preload_portone_failures = 0;
    size_t preload_porttwo_failures = 0;
    size_t relay_portone_failures = 0;
    size_t relay_porttwo_failures = 0;
    size_t relay_incorrect = 0;
    size_t readFailures = 0;
    size_t power_failures = 0;
    size_t register_failures = 0;
    size_t relay_failures = 0;
    size_t instructionFailures = 0;
    size_t relay = 0;
    unsigned long count = 0;
    bool preload_correct_flag_first_read = false;
    bool relay_correct_flag_first_read = false;
    bool preload_correct_flag_second_read = false;
    bool relay_correct_flag_second_read = false;
    bool register_correct_flag_first = false;
    bool register_correct_flag_second = false;
    bool relay_correct_flag = false;
    bool power_correct_flag = false;
    quadlet_t register_read = -1;
    quadlet_t status_read = -1;
    quadlet_t register_read_first;
    quadlet_t register_read_second;
    quadlet_t status_read_first;
    quadlet_t status_read_second;
    nodeaddr_t regnum = 0x14;
    quadlet_t write_data = 0x0;
    quadlet_t write_data_adj = write_data+1;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();

    while (!done) {
        register_read = -1;
        status_read = -1;
        register_correct_flag_first = false;
        register_correct_flag_second = false;
        relay_correct_flag = false;
        power_correct_flag = false;
        // ensure power is disabled and safety relay is off
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
        Amp1394_Sleep(50*1e-6);
        portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
        Amp1394_Sleep(50*1e-6);

        write_data++;
        write_data_adj++;
        count++;

        // check they everythign wrote and read correctly
        if (portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_ENABLE) && porttwo->WriteQuadlet(boardNum, regnum, write_data)
            && portone->ReadQuadlet(boardNum, regnum, register_read_first) && porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_read_first)
            && porttwo->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON) && portone->ReadQuadlet(boardNum, regnum, status_read_second)
            && portone->WriteQuadlet(boardNum, regnum, (write_data_adj)) &&  porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, register_read_second)) {
            // ensure register written correctly
            // note does not ensure that first didn't fail
            if (memcmp((void *)&register_read_first, &(write_data), 4) == 0) {
                register_correct_flag_first = true;
            }

            if (memcmp((void *)&register_read_second, &(write_data_adj), 4) == 0) {
                register_correct_flag_second = true;
            }

            // ensure relay and power written and read correctly
            if (status_read_first & RELAY_BIT) {
                relay_correct_flag = true;
            }

            if (status_read_second & PWR_ENABLE_BIT) {
                power_correct_flag = true;
            }

            if (register_correct_flag_first && register_correct_flag_second && relay_correct_flag && power_correct_flag) {
                    success++;
            } else {
                // error messages
                std::cout << "Register read at first as: " << std::hex << register_read_first << " should be " << write_data << "\n";
                std::cout << "Register read at second as: " << std::hex << register_read_second<< " should be " << (write_data_adj) << "\n";
                if (!(register_correct_flag_first && register_correct_flag_second)) {
                    register_failures++;
                }

                if (!relay_correct_flag) {
                    relay_failures++;
                    std::cout << "Relay was read as OFF should be ON\n";
                } else {
                    std::cout << "Relay was read as ON should be ON\n";
                }

                if (!power_correct_flag) {
                    power_failures++;
                    std::cout << "Power was read as OFF should be ON\n";
                } else {
                    std::cout << "Power was read as ON should be ON\n";
                }
            }
        } else {
            instructionFailures++;
        }
           
        // end conditions and data tracking
        if (power_failures + relay_failures + register_failures > 300) { // ADJUSTABLE
            done = true;
        }

        if (count % 1000 == 0) {
             std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
        }

        if (count >= 10000) {
            done = true;
        }
    }

    std::cout << "attempts = " << std::dec << count << ", success = " << success << "\n";
    std::cout << "failures with register writing: " << std::dec << register_failures << "\n";
    std::cout << "failures with relay writing: " << std::dec << relay_failures << "\n";
    std::cout << "failures with power writing: " << std::dec << power_failures << "\n";
    std::cout << "unsucessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
}

bool SelectCommonBoard(const std::string &portNameOne, const std::string &portNameTwo,
                       const std::vector<AmpIO *> &boardListOne,
                       const std::vector<AmpIO *> &boardListTwo, 
                       AmpIO *&selectedBoardOne, AmpIO *&selectedBoardTwo,
                       unsigned char &curBoardNum) {
    
    // Find common boards between the two lists
    std::vector<AmpIO *> commonBoardsOne;
    std::vector<AmpIO *> commonBoardsTwo;
    
    for (size_t i = 0; i < boardListOne.size(); i++) {
        for (size_t j = 0; j < boardListTwo.size(); j++) {
            if (boardListOne[i]->GetBoardId() == boardListTwo[j]->GetBoardId()) {
                commonBoardsOne.push_back(boardListOne[i]);
                commonBoardsTwo.push_back(boardListTwo[j]);
                break;
            }
        }
    }
    
    // If no common boards found, return false
    if (commonBoardsOne.empty()) {
        std::cout << "No common boards found between " << portNameOne << " and " << portNameTwo << std::endl;
        return false;
    }
    
    // If only one common board, select it automatically
    if (commonBoardsOne.size() == 1) {
        selectedBoardOne = commonBoardsOne[0];
        selectedBoardTwo = commonBoardsTwo[0];
        curBoardNum = static_cast<unsigned char>(commonBoardsOne[0]->GetBoardId());
        std::cout << "Automatically selected common board: " << std::hex 
                  << static_cast<unsigned int>(commonBoardsOne[0]->GetBoardId()) << std::dec << std::endl;
        return true;
    }
    
    // Multiple common boards - user must select one
    std::cout << "Select common board between " << portNameOne << " and " << portNameTwo << ": ";
    for (size_t i = 0; i < commonBoardsOne.size(); i++) {
        std::cout << std::hex << static_cast<unsigned int>(commonBoardsOne[i]->GetBoardId()) << " ";
    }
    std::cout << std::dec << std::endl;
    
    int num = getchar();
    if ((num >= '0') && (num <= '9')) num -= '0';
    else if ((num >= 'a') && (num <= 'f')) num = 10 + (num - 'a');
    else if ((num >= 'A') && (num <= 'F')) num = 10 + (num - 'A');
    else {
        std::cout << "Invalid input. Selection failed." << std::endl;
        return false;
    }
    
    std::cout << std::endl;
    
    // Find the selected board in common boards list
    for (size_t i = 0; i < commonBoardsOne.size(); i++) {
        if (num == commonBoardsOne[i]->GetBoardId()) {
            selectedBoardOne = commonBoardsOne[i];
            selectedBoardTwo = commonBoardsTwo[i];
            curBoardNum = static_cast<unsigned char>(commonBoardsOne[i]->GetBoardId());
            std::cout << "Selected common board: " << std::hex 
                      << static_cast<unsigned int>(commonBoardsOne[i]->GetBoardId()) << std::dec << std::endl;
            return true;
        }
    }
    
    // Selected board ID not found in common boards
    std::cout << "Selected board not available in both ports. Selection failed." << std::endl;
    return false;
}

bool SelectPorts(bool usingZync, bool usingFW, bool usingEth, BasePort *&portUsingOne, BasePort *&portUsingTwo,
    std::string EthPortString, std::string FwPortString, std::string ZyncPortString,  BasePort *FwPort, 
    BasePort *ZyncPort, EthBasePort *EthPort, std::vector<AmpIO *> ZyncBoardList, std::vector<AmpIO *> FwBoardList,
    std::vector<AmpIO *> EthBoardList, std::vector<AmpIO *> &portUsingOneBoardList, 
    std::vector<AmpIO *> &portUsingTwoBoardList) {
    std::cout << "Ports availible: \n";
    if (usingEth) {
        std::cout << "1) " << EthPortString << "\n";
    } 
    if (usingFW) {
        std::cout << "2) " << FwPortString << "\n";
    } 
    if (usingZync) {
        std::cout << "3) " << ZyncPortString << "\n";
    } 

    std::cout << "enter the two numbers of the ports next to each other to choose them \n";
    std::cout << "Example: enter '13' and then a newline to choose ethernet as the first port and Zync as the second port\n";
    std::string input;
    std::getline(std::cin, input);
    input.erase(std::remove_if(input.begin(), input.end(), ::isspace), input.end());
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);
    if (input == "12") {
        if (usingEth && usingFW) {
            portUsingOne = EthPort;
            portUsingTwo = FwPort;
            portUsingOneBoardList = EthBoardList;
            portUsingTwoBoardList = FwBoardList;
            return true;
        } else {
            return false;
        }
    } else if (input == "13") {
        if (usingEth && usingZync) {
            portUsingOne = EthPort;
            portUsingTwo = ZyncPort;
            portUsingOneBoardList = EthBoardList;
            portUsingTwoBoardList = ZyncBoardList;
            return true;
        } else {
            return false;
        }
    } else if (input == "21") {
        if (usingFW && usingEth) {
            portUsingOne = FwPort;
            portUsingTwo = EthPort;
            portUsingOneBoardList = FwBoardList;
            portUsingTwoBoardList = EthBoardList;
            return true;
        } else {
            return false;
        }
    } else if (input == "23") {
        if (usingFW && usingZync) {
            portUsingOne = FwPort;
            portUsingTwo = ZyncPort;
            portUsingOneBoardList = FwBoardList;
            portUsingTwoBoardList = ZyncBoardList;
            return true;
        } else {
            return false;
        }
    } else if (input == "31") {
        if (usingZync && usingEth) {
            portUsingOne = ZyncPort;
            portUsingTwo = EthPort;
            portUsingOneBoardList = ZyncBoardList;
            portUsingTwoBoardList = EthBoardList;
            return true;
        } else {
            return false;
        }
    } else if (input == "32") {
        if (usingZync && usingFW) {
            portUsingOne = ZyncPort;
            portUsingTwo = FwPort;
            portUsingOneBoardList = ZyncBoardList;
            portUsingTwoBoardList = FwBoardList;
            return true;
        } else {
           return false;
        }
    } else {
        return false;
    }
}

bool isValidConfig(bool validPorts, bool validBoard) {
    return validPorts && validBoard;
}


int main(int argc, char **argv) {
    bool useEthernet = true;
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
    int port = 0;
    std::string IPaddr(ETH_UDP_DEFAULT_IP);
    bool fwBridge = false;
    bool isVerbose = false;

    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
                if (!BasePort::ParseOptions(argv[i]+2, desiredPort, port, IPaddr, fwBridge)) {
                    std::cerr << "Failed to parse option: " << argv[i] << std::endl;
                    return 0;
                }
                if ((desiredPort != BasePort::PORT_ETH_UDP) && (desiredPort != BasePort::PORT_ETH_RAW)) {
                    useEthernet = false;
                }
            }
            else if ((argv[i][0] == '-') && (argv[i][1] == 'v')) {
                isVerbose = true;
            }
            else {
                    std::cerr << "Usage: eth1394test [-pP] [-v]" << std::endl
                    << "       where P = port number (default 0)" << std::endl
                    << "                 can also specify -pethP or -pudp" << std::endl
                    << "            -v   verbose output (ZynqEmioPort)" << std::endl;
                    return 0;
            }

        }
    }

    // Compute the hash table values used by the KSZ8851 chip to filter for multicast packets.
    // The results (RegAddr and RegData) are hard-coded in the FPGA code (EthernetIO.v).
    unsigned char MulticastMAC[6];
    EthBasePort::GetDestMulticastMacAddr(MulticastMAC);
    uint8_t RegAddr;
    uint16_t RegData;
    ComputeMulticastHash(MulticastMAC, RegAddr, RegData);
    std::cout << "Multicast hash table: register " << std::hex << (int)RegAddr << ", data = " << RegData << std::endl;
    unsigned char UdpMulticastMAC[6] = { 0x01, 0x00, 0x5e, 0x00, 0x00, 0x64 };  // Corresponds to 224.0.0.100
    ComputeMulticastHash(UdpMulticastMAC, RegAddr, RegData);
    std::cout << "UDP Multicast hash table: register " << std::hex << (int)RegAddr << ", data = " << RegData << std::endl;

    std::vector<AmpIO *> ZyncBoardList;
    std::vector<AmpIO *> FwBoardList;
    std::vector<AmpIO *> EthBoardList;

    AmpIO *curBoard = 0;     // Current board via Ethernet or Firewire / Zynq-EMIO
    AmpIO *curBoardFw = 0;   // Current board via Firewire / Zynq-EMIO, sets boardNumFw
    AmpIO *curBoardZync = 0;  // Current board via Zynq-EMIO
    AmpIO *curBoardEth = 0;  // Current board via Ethernet, sets boardNumEth

    BasePort *curPort = 0;   // Current port (Ethernet, Firewire or Zynq-EMIO)

    BasePort *FwPort = 0;    // Firewire port
    BasePort *ZyncPort = 0;    // Zynq-EMIO port
    EthBasePort *EthPort = 0;  // Ethernet port

    std::string EthPortString;
    std::string FwPortString;
    std::string ZyncPortString;
    std::string curPortString;

    // set up firewire and/or zync
    bool usingFW = false;
    bool usingZync = false;
    bool usingEth = false;

    if (InitFireWire(FwPort, FwBoardList, port)) {
        std::cout << "FireWire Initialized \n";
        usingFW = true;
    } 

    if (InitZync(ZyncPort, ZyncBoardList, port, isVerbose)) {
        std::cout << "Zync Initialized \n";
        usingZync = true;
    } 

    if (!(usingFW || usingZync)) {
        std::cout << "Failed to initialize both Zync and FireWire; need at least two ports for this test program \n";
        return -1;
    }

    // set up FireWire boards
    if (usingFW && FwBoardList.size() > 0) {
        curBoardFw = FwBoardList[0];
        FwPortString = FwPort->GetPortTypeString();
        curBoard = curBoardFw;
        curPort = FwPort;
        curPortString = FwPortString;
    }

    // set up Zync boards
    if (usingZync && ZyncBoardList.size() > 0) {
        curBoardZync = ZyncBoardList[0];
        ZyncPortString = ZyncPort->GetPortTypeString();
        curBoard = curBoardZync;
        curPort = ZyncPort;
        curPortString = ZyncPortString;
    }

    // ensure a board to setup ethernet
    if (!curBoardZync && !curBoardFw) {
        std::cout << "No current board for Zync and FireWire; failed to setup Zync and FireWire \n";
        return -1;
    } else if (!curBoard && !curBoardZync) {
        curBoard = curBoardFw;
    } else if (!curBoard && !curBoardFw) {
        curBoard = curBoardZync;
    }
    unsigned int fpga_ver = curBoard->GetFpgaVersionMajor();

    // setup ethernet
    if (useEthernet) {
        if (desiredPort == BasePort::PORT_ETH_UDP) {
            std::cout << "Creating Ethernet UDP port, IP address = " << IPaddr << std::endl;
            EthPort = new EthUdpPort(port, IPaddr, fwBridge, std::cout);
        }
        if (!EthPort) {
            std::cout << "Failed to create Ethernet port" << std::endl;
            if (!(usingFW && usingZync)) {
                std::cout << "Only one port is active; need at least two ports for this porgram" << std::endl;
                return -1;
            }
        }
        if (EthPort->IsOK()) {
            EthPortString = EthPort->GetPortTypeString();
            for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++) {
                if (EthPort->GetNodeId(bnum) != BasePort::MAX_NODES) {
                    std::cout << "Found Ethernet board: " << bnum << std::endl;
                    AmpIO *board = new AmpIO(bnum);
                    EthPort->AddBoard(board);
                    EthBoardList.push_back(board);
                 }
            }
            if (EthBoardList.size() > 0) {
                curBoardEth = EthBoardList[0];
            }
            if (curPort == FwPort || curPort == ZyncPort) {
                if (fpga_ver == 2) {
                    InitEthernet(*curBoard, 0);
                } else if (fpga_ver == 3) {
                    std::cout << "FPGA V3, Eth1: ";
                    InitEthernet(*curBoard, 1);
                    std::cout << "FPGA V3, Eth2: ";
                    InitEthernet(*curBoard, 2);
                }
            }
            usingEth = true;
        }
    } else if (!(usingFW && usingZync)) {
        std::cout << "Only one port is active; need at least two ports for this porgram" << std::endl;
        return -1;
    }

    if ((!curBoardEth) + (!curBoardFw) + (!curBoardZync) > 1) {
        std::cout << "Fewer than 2 boards found - exiting" << std::endl;
        return -1;
    }

#ifndef _MSC_VER
    // Turn off buffered I/O for keyboard
    struct termios oldTerm, newTerm;
    tcgetattr(0, &oldTerm);
    newTerm = oldTerm;
    newTerm.c_lflag &= ~ICANON;
    newTerm.c_lflag &= ECHO;
    tcsetattr(0, TCSANOW, &newTerm);
#endif

    bool done = false;
    quadlet_t read_data;
    quadlet_t write_data = 0L;
    quadlet_t buffer[128];
    BasePort *portUsingOne = 0;   // first port being used (Ethernet, Firewire or Zynq-EMIO)
    BasePort *portUsingTwo = 0;   // second port being used (Ethernet, Firewire or Zynq-EMIO)
    std::vector<AmpIO *> portUsingOneBoardList;
    std::vector<AmpIO *> portUsingTwoBoardList;
    AmpIO *selectedBoardOne = 0;
    AmpIO *selectedBoardTwo = 0;
    bool validPorts = false;
    bool validBoard = false;
    unsigned char curBoardNum = 0;

    std::cout << std::endl << "Glitch Test Program" << std::endl;
    validPorts = SelectPorts(usingZync, usingFW, usingEth, portUsingOne, portUsingTwo, EthPortString, 
                    FwPortString, ZyncPortString, FwPort, ZyncPort, EthPort, ZyncBoardList, FwBoardList,
                    EthBoardList, portUsingOneBoardList, portUsingTwoBoardList);
    std::cout << "Port one now selected to be " << portUsingOne->GetPortTypeString() << "\n";
    std::cout << "Port two now selected to be " << portUsingTwo->GetPortTypeString() << "\n";
    if (validPorts) {
        std::cout << "These are valid ports for use\n";
    } else {
        std::cout << "These are invalid ports for use\n";
    }

    if (portUsingOne && portUsingTwo && validPorts) { // ensure not null pointers or invalid ports
        validBoard = SelectCommonBoard(portUsingOne->GetPortTypeString(), portUsingTwo->GetPortTypeString(),
                        portUsingOneBoardList, portUsingTwoBoardList, selectedBoardOne, selectedBoardTwo, curBoardNum);
        std::cout << "Board now set to board " << curBoardNum << "\n";
        if (validBoard) {
            std::cout << "This board is valid for use\n";
        } else {
            std::cout << "This board is invalid for use\n";
        }
    } else {
        std::cout << "One of the current ports is null or the current ports are invalid\n";
    }

    std::cout << "To run a test, enter the number or letter before the ')' and then a newline\n";
    std::cout << "Fox example, only enter \"q\" or \"4\" or \"17\" and then a newline\n";
    
    std::string input;
    while (!done) {
        unsigned char EthBoardNum = 0;
        unsigned char FwBoardNum = 0;
        unsigned char ZyncBoardNum = 0;
        unsigned int fpga_ver = curBoard->GetFpgaVersionMajor();
        double clkPeriod = curBoard->GetFPGAClockPeriod();


/*      if (curBoardFw) {
            FwBoardNum = curBoardFw->GetBoardId();
            std::cout << "  " << FwPortString << " board: "
                      << static_cast<unsigned int>(FwBoardNum);
            if (curPort == FwPort)
                std::cout << "  <-- active";
            std::cout << std::endl;
        }

        if (curBoardZync) {
            ZyncBoardNum = curBoardZync->GetBoardId();
            std::cout << "  " << ZyncPortString << " board: "
                      << static_cast<unsigned int>(ZyncBoardNum);
            if (curPort == ZyncPort)
                std::cout << "  <-- active";
            std::cout << std::endl;
        }

        if (curBoardEth) {
            EthBoardNum = curBoardEth->GetBoardId();
            std::cout << "  " << EthPortString << " board: "
                      << static_cast<unsigned int>(EthBoardNum);
            if (curPort == EthPort)
                std::cout << "  <-- active";
            std::cout << std::endl;
        }
            */

        
        

        
        std::cout << "-------------------------------------- Logistical Functions --------------------------------------\n";
        std::cout << "  q) Quit \n";
        std::cout << "  s) Ethernet status \n";
        std::cout << "  p) Change ports \n";
        std::cout << "  b) Change board \n";
        std::cout << "  c) Current ports and board (and their validity for use) \n";
        std::cout << "--------------------------------------    Test Functions    --------------------------------------\n";
        std::cout << "  0) two commuications methods read; one register \n";
        std::cout << "  1) one commuication method writes, one reads; one register \n";
        std::cout << "  2) one commuication method writes, one reads; one register; stress test (both read correctly) \n";
        std::cout << "  3) two commuication methods read status \n";
        std::cout << "  4) two commuication methods write different things to status \n";
        std::cout << "  5) two commuication methods write different things to status and then read status\n";
        std::cout << "  6) two commuication methods read status and then write different things to status\n";
        std::cout << "  7) two commuication methods read, then write different things, then read status\n";
        std::cout << "  8) two commuication methods write different things, then read, then write different things to status\n";
        std::cout << "  9) one communication method writes to waveform and one reads from waveform\n";
        std::cout << "  10) one communication method reads status and one reads register \n";
        std::cout << "  11) one communication method writes to status and one reads register\n";
        std::cout << "  12) one communication method reads status and one write to register\n";
        std::cout << "  13) one commuication methods writes to status, one writes to register\n";
        std::cout << "  14) two communication methods read both status and register\n";
        std::cout << "  15) two communication methods write to both status and register\n";
        std::cout << "  16) two communication methods write to and read from both status and register\n";


        std::getline(std::cin, input);
        input.erase(std::remove_if(input.begin(), input.end(), ::isspace), input.end());
        std::transform(input.begin(), input.end(), input.begin(), ::tolower);

        if (input == "q") {
            std::cout << "Quitting program...\n";
            done = true;
        } else if (input == "s") {
            std::cout << "Checking Ethernet status...\n";
            PrintEthernetStatus(*curBoard);
        } else if (input == "p") {
            std::cout << "Changing ports...\n";
            validPorts = SelectPorts(usingZync, usingFW, usingEth, portUsingOne, portUsingTwo, EthPortString, 
                    FwPortString, ZyncPortString, FwPort, ZyncPort, EthPort, ZyncBoardList, FwBoardList,
                    EthBoardList, portUsingOneBoardList, portUsingTwoBoardList);
                    std::cout << "Port one now selected to be " << portUsingOne->GetPortTypeString() << "\n";
                    std::cout << "Port two now selected to be " << portUsingTwo->GetPortTypeString() << "\n";
                    if (validPorts) {
                        std::cout << "These are valid ports for use\n";
                    } else {
                        std::cout << "These are invalid ports for use\n";
                    }
        } else if (input == "b") {
            std::cout << "Changing board...\n";
            if (portUsingOne && portUsingTwo) { // ensure not null pointers
                validBoard = SelectCommonBoard(portUsingOne->GetPortTypeString(), portUsingTwo->GetPortTypeString(),
                                portUsingOneBoardList, portUsingTwoBoardList, selectedBoardOne, selectedBoardTwo, curBoardNum);
                std::cout << "Board now set to board " << curBoardNum << "\n";
                if (validBoard) {
                    std::cout << "This board is valid for use\n";
                } else {
                    std::cout << "This board is invalid for use\n";
                }
            } else {
                std::cout << "One of the current ports is null\n";
            }
        } else if (input == "c") {
            if (portUsingOne && portUsingTwo) { // ensure not null pointers
                std::cout << "Current Port One: " << portUsingOne->GetPortTypeString() << "\n";
                std::cout << "Current Port Two: " << portUsingTwo->GetPortTypeString() << "\n";
                std::cout << "Current Board ID: " << curBoardNum << "\n";
            }
            if (validPorts && validBoard) {
                std::cout << "This configuration is valid for use\n";
            } else {
                std::cout << "This configuration is invalid for use\n";
            }
        } else if (input == "0") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 0: two communications methods read; one register\n";
                ReadSameRegisterTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "1") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 1: one communication method writes, one reads; one register\n";
                WriteAndReadOneRegisterDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "2") {
            if (isValidConfig(validPorts, validBoard)) {
                 std::cout << "Running test 2: stress test\n";
                 WriteAndReadOneRegisterDifferentMethodsStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "3") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 3: two communication methods read status\n";
                ReadDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "4") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 4: two communication methods write different things to status\n";
                WriteDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "5") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 5: write then read status\n";
                WriteThenReadDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum); 
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "6") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 6: read then write status\n";
                ReadThenWriteDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "7") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 7: read, write, read status\n";
                RapidReadWriteReadDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "8") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 8: write, read, write status\n";
                RapidWriteReadWriteDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum); 
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "9") {
             if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 9: waveform write/read\n";
                WaveformReadAndWriteDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum, selectedBoardOne);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "10") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 10: one reads status, one reads register\n";
                ReadRegisterAndStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "11") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 11: one writes status, one reads register\n";
                ReadRegisterWriteStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "12") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 12: one reads status, one writes register\n";
                WriteRegisterReadStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "13") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 13: one writes status, one writes register\n";
                WriteRegisterWriteStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "14") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 14: two methods read both status and register\n";
                ReadRegisterAndStatusStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "15") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 15: two methods write to both status and register\n";
                WriteRegisterAndStatusStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else if (input == "16") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 16: two methods write/read both status and register\n";
                AlternatingReadAndWriteRegisterAndStatusStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuartion is invalid; test not run\n";
            }
        } else {
            std::cout << "Invalid option. Please try again.\n";
        }
    }



#ifndef _MSC_VER
    tcsetattr(0, TCSANOW, &oldTerm);  // Restore terminal I/O settings
#endif

    // Clean up Firewire port and boards
    if (FwPort) {
        for (unsigned int bd = 0; bd < FwBoardList.size(); bd++) {
            FwPort->RemoveBoard(FwBoardList[bd]->GetBoardId());
            delete FwBoardList[bd];
        }
        delete FwPort;
    }

    // Clean up Zynq port and boards
    if (ZyncPort) {
        for (unsigned int bd = 0; bd < ZyncBoardList.size(); bd++) {
            ZyncPort->RemoveBoard(ZyncBoardList[bd]->GetBoardId());
            delete ZyncBoardList[bd];
        }
        delete ZyncPort;
    }

    // Clean up Ethernet port and boards
    if (EthPort) {
        for (unsigned int bd = 0; bd < EthBoardList.size(); bd++) {
            EthPort->RemoveBoard(EthBoardList[bd]->GetBoardId());
            delete EthBoardList[bd];
        }
        delete EthPort;
    }

    return 0;
}
