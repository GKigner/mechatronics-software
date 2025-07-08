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

const uint32_t PWR_ENABLE_MASK  = 0x00080000;  /*!< Power enable mask (write only) */
const uint32_t PWR_ENABLE_BIT   = 0x00040000;  /*!< Power enable status (read/write) */
const uint32_t PWR_ENABLE       = PWR_ENABLE_MASK|PWR_ENABLE_BIT;
const uint32_t PWR_DISABLE      = PWR_ENABLE_MASK;
const uint32_t RELAY_MASK       = 0x00020000;  /*!< Safety relay enable mask (write only) */
const uint32_t RELAY_BIT        = 0x00010000;  /*!< Safety relay enable (read/write) */
const uint32_t RELAY_ON         = RELAY_MASK|RELAY_BIT;
const uint32_t RELAY_OFF        = RELAY_MASK;

/****************************************************************
*  @brief  Calculate CRC-32 checksum for KSZ8851 Ethernet controller.
*
*  @param   const unsigned char *data   Data buffer to calculate CRC for.
*  @param   size_t              len     Length of data buffer in bytes.
*
*  @note    This function comes from fpgatest.cpp.
*  @note    Uses polynomial 0x04c11db7 with initial value 0xffffffff.
*
*  @return  uint32_t 32-bit CRC checksum value
****************************************************************/
uint32_t KSZ8851CRC(const unsigned char *data, size_t len) {
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

/****************************************************************
*  @brief  Compute multicast hash table parameters for KSZ8851
*          Ethernet controller.
*
*  @param   unsigned char *MulticastMAC  6-byte multicast MAC address.
*  @param   uint8_t       &regAddr       Reference to receive hash table
*                                        register address.
*  @param   uint16_t      &regData       Reference to receive hash table
*                                        register bit mask.
*
*  @note    This function comes from fpgatest.cpp.
*  @note    Calculates CRC of MAC address and extracts register offset
*           and bit position for hash table configuration.
*
*  @return  none
****************************************************************/
void ComputeMulticastHash(unsigned char *MulticastMAC, uint8_t &regAddr, uint16_t &regData) {
    uint32_t crc = KSZ8851CRC(MulticastMAC, 6);
    int regOffset = (crc >> 29) & 0x0006;  // first 2 bits of CRC (x2)
    int regBit = (crc >> 26) & 0x00F;      // next 4 bits of CRC
    regAddr = 0xA0 + regOffset;            // 0xA0 --> MAHTR0 (MAC Address Hash Table Register 0)
    regData = (1 << regBit);
}

/****************************************************************
*  @brief  Print Ethernet status information from FPGA register.
*
*  @param   AmpIO &Board    Board object interface for accessing
*                           board-specific registers.
*
*  @note    This function comes from fpgatest.cpp.
*  @note    Reads status from FPGA register 12 and displays formatted
*           output using EthBasePort::PrintStatus.
*
*  @return  none
****************************************************************/
void PrintEthernetStatus(AmpIO &Board) {
    uint32_t status;
    if (Board.ReadEthernetStatus(status))
        EthBasePort::PrintStatus(std::cout, status);
}

/****************************************************************
*  @brief  Check KSZ8851 register contents against expected values.
*
*  @param   AmpIO    &Board    Board object interface for accessing
*                              board-specific registers.
*  @param   uint8_t  regNum    Register number to check.
*  @param   uint16_t mask      Bit mask to apply before comparison.
*  @param   uint16_t value     Expected value after masking.
*
*  @note    This function comes from fpgatest.cpp.
*  @note    Logs register mismatches with actual vs expected values
*           in hex format.
*
*  @return  bool that indicates if register contents match expected 
*           value
****************************************************************/
bool CheckRegister(AmpIO &Board, uint8_t regNum, uint16_t mask, uint16_t value) {
    uint16_t reg;
    Board.ReadKSZ8851Reg(regNum, reg);
    if ((reg&mask) != value) {
        std::cout << "Register " << std::hex << (int)regNum << ": read = " << reg
                  << ", expected = " << value << " (mask = " << mask << ")" << std::endl;
        return false;
    }
    return true;
}

/****************************************************************
*  @brief  Verify KSZ8851 Ethernet controller registers are properly
*          configured for operation.
*
*  @param   AmpIO &Board    Board object interface for accessing
*                           board-specific registers.
*
*  @note    This function comes from fpgatest.cpp.
*  @note    Checks MAC address, QMU settings, receive/transmit configuration,
*           multicast hash table, and interrupt settings.
*
*  @return  bool that indicates if the all register checks passed
****************************************************************/
bool CheckEthernetV2(AmpIO &Board) {
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

/****************************************************************
*  @brief  Test RTL8211F PHY register read/write functionality using
*          walking bit pattern.
*
*  @param   AmpIO        &Board    Board object interface for accessing
*                                  board-specific registers.
*  @param   unsigned int chan      Ethernet channel number.
*  @param   unsigned int phyAddr   PHY address for register access.
*
*  @note    This function comes from fpgatest.cpp.
*  @note    Performs walking bit test on interrupt enable register and
*           restores original values after testing.
*
*  @return  bool that indicates if register I/O test passed
****************************************************************/
bool CheckRTL8211F_RegIO(AmpIO &Board, unsigned int chan, unsigned int phyAddr) {
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


/****************************************************************
*  @brief  Verify RTL8211F Ethernet PHY registers are properly
*          configured and accessible.
*
*  @param   AmpIO        &Board    Board object interface for accessing
*                                  board-specific registers.
*  @param   unsigned int chan      Ethernet channel number to verify.
*
*  @note    This function comes from fpgatest.cpp.
*  @note    Checks PHY ID registers, TX/RX delay settings, and GMII
*           to RGMII core configuration.
*
*  @return  bool that indicates if PHY verification passed
****************************************************************/
bool CheckEthernetV3(AmpIO &Board, unsigned int chan) {
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

/****************************************************************
*  @brief  Initialize an Ethernet communication port and configure
*          the physical layer interface.
*
*  @param   AmpIO          &Board       Board object interface for accessing
*                                       board-specific parameters and registers.
*  @param   unsigned int   eth_port     Ethernet port number (0 for FPGA V2,
*                                       1 or 2 for FPGA V3).
*
*  @note    Requires firmware version 5 or higher and FPGA version 2 or higher.
*  @note    Performs PHY reset, status verification, and register validation
*           specific to the FPGA version.
*
*  @return  bool that indicates if the initialization was successful
****************************************************************/
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
    PrintEthernetStatus(Board);

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

/****************************************************************
*  @brief  Initialize a FireWire communication port and discover
*          connected boards.
*
*  @param   BasePort *&              FwPort        Reference that receives the
*                                                  newly created FirewirePort
*                                                  pointer on success.
*  @param   std::vector<AmpIO *>&    FwBoardList   Vector to which discovered
*                                                  AmpIO board objects are
*                                                  appended.
*  @param   int                      port          FireWire port number to
*                                                  initialize.
*
*  @note    Compiled only when Amp1394_HAS_RAW1394 is defined, otherwise
*           returns false.
*  @note    Automatically discovers and registers all boards found on the
*           FireWire bus.
*
*  @return  bool that indicates if the initialization was successful
****************************************************************/
bool InitFireWire(BasePort *&FwPort, std::vector<AmpIO *> &FwBoardList, int port) {
    #if Amp1394_HAS_RAW1394
        FwPort = new FirewirePort(port, std::cout);
        if (!FwPort->IsOK()) {
            std::cout << "Failed to initialize firewire port" << std::endl;
            delete FwPort; 
            FwPort = nullptr;
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
*  @brief  Initialize a Zynq-EMIO communication port and discover boards.
*
*  @param   BasePort *&              ZynqPort      Reference that receives the
*                                                  newly created ZynqEmioPort
*                                                  pointer on success.
*  @param   std::vector<AmpIO *>&    ZynqBoardList Vector to which discovered
*                                                  AmpIO board objects are
*                                                  appended.
*  @param   int                      port          Device index of the Zynq
*                                                  EMIO interface (usually 0).
*  @param   bool                     isVerbose     Enables verbose printing if
*                                                  set to true.
*
*  @note    Compiled only when Amp1394_HAS_EMIO is defined, otherwise returns
*           false and suppresses unused parameter warnings.
*  @note    Zynq EMIO port always has exactly one node (node 0) which is
*           automatically queried for board discovery.
*
*  @return  bool that indicates if the initialization was successful
****************************************************************/
bool InitZynq(BasePort *&ZynqPort, std::vector<AmpIO *> &ZynqBoardList, int port, bool isVerbose) {
    #if Amp1394_HAS_EMIO
        ZynqEmioPort *ZynqPortTemp;
        ZynqPortTemp = new ZynqEmioPort(port, std::cout);
        if (!ZynqPortTemp->IsOK()) {
            std::cout << "Failed to initialize Zynq EMIO port" << std::endl;
            delete ZynqPortTemp; 
            ZynqPortTemp = nullptr;
            ZynqPort = nullptr;
            return false;
        }
        ZynqPortTemp->SetVerbose(isVerbose);

        ZynqPort = ZynqPortTemp;
        // Zynq EMIO port always has one node (0)
        unsigned int bnum = ZynqPort->GetBoardId(0);
        if (bnum < BoardIO::MAX_BOARDS) {
            std::cout << "Found Zynq EMIO board: " << bnum << std::endl;
            AmpIO *board = new AmpIO(bnum);
            ZynqPort->AddBoard(board);
            ZynqBoardList.push_back(board);
        }
            return true;
    #else
         // Suppress unused parameter warning
        (void)ZynqPort;    
        (void)ZynqBoardList;
        (void)port;        
        (void)isVerbose;
        return false;
    #endif
}


/****************************************************************
*  @brief  Tests that two communication methods read the same register
*          and produce matching data.
*
*  @details Targets glitches that might occur from rapid concurrent reads
*           using different communication methods. Both ports read the
*           board status register back-to-back and compare the 32-bit values.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs mismatched readings with both ports' 32-bit status words
*           in hex format.
*  @note    Tracks successful reads, comparison failures, and low-level
*           ReadQuadlet errors.
*
*  @return  none
****************************************************************/
void ReadSameRegisterTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
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
*  @brief  Tests that one communication method can write while another
*          reads the same register without data corruption.
*
*  @details Targets glitches that might occur from rapid write-then-read
*           operations using different communication methods. One port writes
*           incrementing values to a register while the other port reads
*           back the data for verification.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs data mismatches showing expected vs actual values in hex.
*  @note    Tracks successful operations, comparison failures, and failed
*           ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
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

    // continuously write and read and look for glitches
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
*  @brief  Tests that one communication method can write while both
*          methods read the same register for cross-verification.
*
*  @details Targets glitches that might occur from rapid write-then-read
*           operations with multiple readers using different communication
*           methods. One port writes incrementing values while both ports
*           read back the data for cross-verification.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs data mismatches showing write data vs read data from both
*           ports in hex format.
*  @note    Tracks successful operations, comparison failures, and failed
*           ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
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

    // continuously write and read and look for glitches
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

/****************************************************************
*  @brief  Tests that two communication methods can read the board status
*          register simultaneously and produce consistent results.
*
*  @details Targets glitches that might occur from concurrent reads of the
*           same register using different communication methods. Both ports
*           read the board status register in quick succession after enabling
*           power and safety relay.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs mismatched status readings with port type identification
*           and hex values.
*  @note    Tracks successful reads, comparison failures, and failed
*           ReadQuadlet calls.
*
*  @return  none
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
    std::cout << "compare failures = " << compareFailures << "\n";
    std::cout << "unsuccessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";

    // ensure power is disabled and safety relay are off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can write different
*          control bits and both changes are properly applied.
*
*  @details Targets glitches that might occur from concurrent writes to
*           different control bits using different communication methods.
*           One port enables power while the other enables the safety relay,
*           then verifies both settings took effect.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs missing control bits with port identification and expected
*           vs actual state.
*  @note    Tracks successful operations, per-port write failures, and failed
*           WriteQuadlet calls.
*
*  @return  none
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
    std::cout << " failures with " << portOneString << std::dec << compareFailuresPortOne << " failures with " << portTwoString << compareFailuresPortTwo << "\n";
    std::cout << "unsuccessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";

    // ensure power is disabled and safety relay are off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can write control bits
*          and both can read back the combined status consistently.
*
*  @details Targets glitches that might occur from rapid write-then-read
*           sequences using different communication methods. Both ports write
*           different control bits, then both read back the status register
*           to verify all changes are visible to both interfaces.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs missing control bits with port identification and which
*           read operation detected the inconsistency.
*  @note    Tracks successful operations, per-port read failures, and failed
*           WriteQuadlet/ReadQuadlet calls.
*
*  @return  none
****************************************************************/
void WriteThenReadDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    size_t compareReadFailuresPortOne = 0;
    size_t compareReadFailuresPortTwo = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one;
    quadlet_t status_result_two;
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
    std::cout << " failures with " << portOneString << std::dec << compareReadFailuresPortOne << " failures with " << portTwoString << compareReadFailuresPortTwo << "\n";
    std::cout << "unsuccessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsuccessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure power is disabled and safety relay are off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can read status, then
*          one can write, without corrupting the initial read data.
*
*  @details Targets glitches that might occur from rapid read-write
*           sequences using different communication methods. Both ports
*           read the status register, then one port writes to enable
*           the safety relay, followed by verification that the status
*           changed appropriately.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs initial status readings that don't match and cases where
*           writes don't take effect as expected.
*  @note    Tracks successful operations, initial read mismatches, write
*           verification failures, and failed ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void ReadThenWriteDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one_stage_one;
    quadlet_t status_result_two_stage_one;
    quadlet_t status_result_stage_two;
    size_t compareFailuresInitial = 0;
    size_t compareFailuresInitialVsSecond = 0;
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
                        compareFailuresInitial++;
                        iteration_success = false;
                    }

                    // check inital and second reads are not equal
                    if (status_result_stage_two == status_result_one_stage_one || status_result_stage_two == status_result_two_stage_one) {
                        std::cout << "Initial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        std::cout << "Initial Status reading from " << portTwoString << ": " << std::hex << status_result_two_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        compareFailuresInitialVsSecond++;
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
        if (compareFailuresInitial + compareFailuresInitialVsSecond > 400) { // ADJUSTABLE
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
    std::cout << "failures with initial readings not matching: " << std::dec << compareFailuresInitial << "\n";
    std::cout << "failures where first and second readings should not match: " << std::dec << compareFailuresInitialVsSecond << "\n";
    std::cout << "unsuccessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsuccessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure safety relay is off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can perform rapid
*          read-write-read sequences without data corruption.
*
*  @details Targets glitches that might occur from rapid read-write-read
*           sequences using different communication methods. Both ports
*           read status, one port writes to enable the safety relay, then
*           both ports read again to verify consistency throughout the
*           operation sequence.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs mismatched readings at each stage and cases where expected
*           status changes don't occur.
*  @note    Tracks successful operations, initial read mismatches, second
*           read mismatches, write verification failures, and failed
*           ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void RapidReadWriteReadDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one_stage_one;
    quadlet_t status_result_two_stage_one;
    quadlet_t status_result_one_stage_two;
    quadlet_t status_result_two_stage_two;
    size_t compareFailuresInitial = 0;
    size_t compareFailuresSecond = 0;
    size_t compareFailuresInitialVsSecond = 0;
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
                        compareFailuresInitial++;
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
                        std::cout << "Initial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", second status reading from " << portOneString << ": " << status_result_one_stage_two << "\n";
                        std::cout << "Initial Status reading from " << portTwoString << ": " << std::hex << status_result_two_stage_one << ", second status reading from " << portTwoString << ": " << status_result_two_stage_two << "\n";
                        compareFailuresInitialVsSecond++;
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
        if (compareFailuresInitial + compareFailuresSecond + compareFailuresInitialVsSecond > 400) { // ADJUSTABLE
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
    std::cout << "failures with initial readings not matching: " << std::dec << compareFailuresInitial << "\n";
    std::cout << "failures with second readings not matching: " << std::dec << compareFailuresSecond << "\n";
    std::cout << "failures where first and second readings should not match: " << std::dec << compareFailuresInitialVsSecond << "\n";
    std::cout << "unsuccessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsuccessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure safety relay is off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can perform rapid
*          write-read-write sequences without data corruption.
*
*  @details Targets glitches that might occur from rapid write-read-write
*           sequences using different communication methods. One port
*           disables the safety relay, both ports read status, then one
*           port re-enables the relay to verify the sequence operates
*           correctly.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs mismatched initial readings and cases where write operations
*           don't take effect as expected.
*  @note    Tracks successful operations, initial read mismatches, write
*           verification failures, and failed ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void RapidWriteReadWriteDifferentThingsDifferentMethodsTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    int count = 0;
    bool iteration_success;
    size_t success = 0;
    std::string portOneString = portone->GetPortTypeString();
    std::string portTwoString = porttwo->GetPortTypeString();
    size_t failedQuadWrite = 0;
    quadlet_t status_result_one_stage_one;
    quadlet_t status_result_two_stage_one;
    quadlet_t status_result_stage_two;
    size_t compareFailuresInitial = 0;
    size_t compareFailuresInitialVsSecond = 0;
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
                        compareFailuresInitial++;
                        iteration_success = false;
                    }

                    // check inital and second reads are not equal
                    if (status_result_stage_two == status_result_one_stage_one || status_result_stage_two == status_result_two_stage_one) {
                        std::cout << "Initial Status reading from " << portOneString << ": " << std::hex << status_result_one_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        std::cout << "Initial Status reading from " << portTwoString << ": " << std::hex << status_result_two_stage_one << ", second status reading: " << status_result_stage_two << "\n";
                        compareFailuresInitialVsSecond++;
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
        if (compareFailuresInitial + compareFailuresInitialVsSecond > 400) { // ADJUSTABLE
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
    std::cout << "failures with initial readings not matching: " << std::dec << compareFailuresInitial << "\n";
    std::cout << "failures where first and second readings should not match: " << std::dec << compareFailuresInitialVsSecond << "\n";
    std::cout << "unsuccessful attempts to write (i.e. One of the WriteQuadlet calls returned false) " << std::dec << failedQuadWrite <<"\n";    
    std::cout << "unsuccessful attempts to read (i.e. One of the ReadQuadlet calls returned false) " << std::dec << failedQuadRead <<"\n";
    
    // ensure safety relay is off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can write and read
*          waveform data without corruption or glitches.
*
*  @details Targets glitches that might occur from large block transfers
*           using different communication methods. One port writes a test
*           waveform pattern while another port reads it back to verify
*           data integrity across the complete waveform table.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*  @param   AmpIO          (*board)     Board object interface for accessing
*                                       board-specific parameters.
*
*  @note    Logs waveform mismatches showing quadlet index, read value, and
*           expected value in hex format.
*  @note    Tracks waveform data integrity and performs a second read after
*           delays if initial mismatches are detected.
*
*  @return  none
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

/****************************************************************
*  @brief  Tests that two communication methods can read different
*          registers simultaneously without interfering with each other.
*
*  @details Targets glitches that might occur from concurrent reads of
*           different registers using different communication methods.
*           One port reads the preload register while the other reads
*           the board status register to verify both operations complete
*           successfully.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs incorrect preload and relay readings with expected vs
*           actual values.
*  @note    Tracks successful operations, preload read failures, relay
*           read failures, and failed ReadQuadlet calls.
*
*  @return  none
****************************************************************/
void ReadRegisterAndStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t preload_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t readFailures = 0;
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
    std::cout << "unsuccessful attempts to read " << std::dec << readFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that one communication method can read a register while
*          another writes to a different register simultaneously.
*
*  @details Targets glitches that might occur from concurrent read and write
*           operations to different registers using different communication
*           methods. One port reads the preload register while the other
*           writes to the board status register.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs incorrect preload readings and relay write verification
*           failures with expected vs actual values.
*  @note    Tracks successful operations, preload read failures, relay
*           write failures, and failed ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void ReadRegisterWriteStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t preload_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t instructionFailures = 0;
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
    std::cout << "unsuccessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that one communication method can write to a register
*          while another reads from a different register simultaneously.
*
*  @details Targets glitches that might occur from concurrent write and read
*           operations to different registers using different communication
*           methods. One port writes to a register while the other reads
*           the board status register.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs register write verification failures and incorrect relay
*           readings with expected vs actual values in hex.
*  @note    Tracks successful operations, register write failures, relay
*           read failures, and failed ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void WriteRegisterReadStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    nodeaddr_t regnum = 0x14; 
    quadlet_t write_data = 0x0;
    size_t success = 0;
    size_t register_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t instructionFailures = 0;
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
    std::cout << "unsuccessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can write to different
*          registers simultaneously without interfering with each other.
*
*  @details Targets glitches that might occur from concurrent writes to
*           different registers using different communication methods.
*           One port writes to a data register while the other writes
*           to the board status register.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs register and relay write verification failures with
*           expected vs actual values in hex.
*  @note    Tracks successful operations, register write failures, relay
*           write failures, and failed ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void WriteRegisterWriteStatusTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one;
    quadlet_t read_data_port_two;
    nodeaddr_t regnum = 0x14; 
    quadlet_t write_data = 0x0;
    size_t success = 0;
    size_t register_incorrect = 0;
    size_t relay_incorrect = 0;
    size_t instructionFailures = 0;
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
    std::cout << "unsuccessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can read different
*          registers multiple times without cross-interference.
*
*  @details Targets glitches that might occur from repeated concurrent
*           reads of different registers using different communication
*           methods. Both ports read different registers twice in sequence
*           to verify consistent operation under stress conditions.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs per-port preload and relay reading failures with port
*           identification and expected vs actual values.
*  @note    Tracks successful operations, per-port preload failures,
*           per-port relay failures, and failed ReadQuadlet calls.
*
*  @return  none
****************************************************************/
void ReadRegisterAndStatusStressTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t read_data_port_one_first_read;
    quadlet_t read_data_port_two_first_read;
    quadlet_t read_data_port_one_second_read;
    quadlet_t read_data_port_two_second_read;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t preload_portone_failures = 0;
    size_t preload_porttwo_failures = 0;
    size_t relay_portone_failures = 0;
    size_t relay_porttwo_failures = 0;
    size_t readFailures = 0;
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
    std::cout << " failures with " << portOneString << " preload reading: " << std::dec << preload_portone_failures << "\n";
    std::cout << " failures with " << portTwoString << " preload reading: " << std::dec << preload_porttwo_failures << "\n";
    std::cout << " failures with " << portOneString << " relay reading: " << std::dec << relay_portone_failures << "\n";
    std::cout << " failures with " << portTwoString << " relay reading: " << std::dec << relay_porttwo_failures << "\n";
    std::cout << "unsuccessful attempts to read " << std::dec << readFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can write to different
*          registers multiple times under stress conditions.
*
*  @details Targets glitches that might occur from repeated concurrent
*           writes to different registers using different communication
*           methods. Multiple sequential writes are performed to both
*           data and control registers to verify sustained operation
*           integrity.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs register, relay, and power write verification failures
*           with expected vs actual values in hex.
*  @note    Tracks successful operations, register write failures, relay
*           write failures, power write failures, and failed ReadQuadlet/
*           WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void WriteRegisterAndStatusStressTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    quadlet_t register_read;
    quadlet_t status_read;
    //char buf[5] = "QLA1";
    nodeaddr_t regnum = 0x14; 
    quadlet_t write_data = 0x0;
    quadlet_t write_data_adj = write_data+1;
    size_t success = 0;
    size_t relay_failures = 0;
    size_t power_failures = 0;
    size_t register_failures = 0;
    size_t instructionFailures = 0;
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
    std::cout << "unsuccessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Tests that two communication methods can perform alternating
*          read and write operations without interference.
*
*  @details Targets glitches that might occur from rapid alternating
*           read-write sequences using different communication methods.
*           Interleaved read and write operations are performed on both
*           data and control registers to verify operation coherence
*           under mixed access patterns.
*
*  @param   BasePort       (*portone)   First communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   BasePort       (*porttwo)   Second communication interface object
*                                       that provides access to the physical
*                                       connection to the controller boards.
*  @param   unsigned char  (boardNum)   Target board number.
*
*  @note    Logs register, relay, and power operation failures at different
*           stages with expected vs actual values in hex.
*  @note    Tracks successful operations, register operation failures,
*           relay operation failures, power operation failures, and failed
*           ReadQuadlet/WriteQuadlet calls.
*
*  @return  none
****************************************************************/
void AlternatingReadAndWriteRegisterAndStatusStressTest(BasePort *portone, BasePort *porttwo, unsigned char boardNum) {
    bool done = false;
    size_t success = 0;
    size_t power_failures = 0;
    size_t register_failures = 0;
    size_t relay_failures = 0;
    size_t instructionFailures = 0;
    unsigned long count = 0;
    bool register_correct_flag_first = false;
    bool register_correct_flag_second = false;
    bool relay_correct_flag = false;
    bool power_correct_flag = false;
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
        register_read_first = -1;
        register_read_second = -1;
        status_read_first = -1;
        status_read_second = -1;
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
            && porttwo->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_ON) && portone->ReadQuadlet(boardNum, regnum, register_read_second)
            && portone->WriteQuadlet(boardNum, regnum, (write_data_adj)) &&  porttwo->ReadQuadlet(boardNum, BoardIO::BOARD_STATUS, status_read_second)) {
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
    std::cout << "unsuccessful attempts to read or write " << std::dec << instructionFailures <<"\n";
    
    // ensure safety relay is back off
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, RELAY_OFF);
    Amp1394_Sleep(50*1e-6);
    portone->WriteQuadlet(boardNum, BoardIO::BOARD_STATUS, PWR_DISABLE);
    Amp1394_Sleep(50*1e-6);
}

/****************************************************************
*  @brief  Find and select a common board accessible through two
*          different communication ports.
*
*  @param   const std::string &        portNameOne      Name of first port for
*                                                        display purposes.
*  @param   const std::string &        portNameTwo      Name of second port for
*                                                        display purposes.
*  @param   const std::vector<AmpIO *> &boardListOne    Vector of boards
*                                                        available on first port.
*  @param   const std::vector<AmpIO *> &boardListTwo    Vector of boards
*                                                        available on second port.
*  @param   AmpIO *&                   selectedBoardOne Reference to receive
*                                                        selected board object
*                                                        for first port.
*  @param   AmpIO *&                   selectedBoardTwo Reference to receive
*                                                        selected board object
*                                                        for second port.
*  @param   unsigned char &            curBoardNum      Reference to receive
*                                                        selected board ID.
*
*  @note    Automatically selects board if only one common board exists,
*           otherwise prompts user for selection.
*  @note    Accepts hex input (0-9, a-f, A-F) for board selection and
*           handles EOF conditions gracefully.
*
*  @return  bool that indicates if a common board was successfully selected
****************************************************************/
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
    if (num == EOF) {  // Check for EOF
        std::cout << "\nEOF detected. Selection failed." << std::endl;
        return false;
    }
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

/****************************************************************
*  @brief  Select two communication ports from available options
*          for dual-port testing.
*
*  @param   bool                    usingZynq           Flag indicating Zynq
*                                                       port availability.
*  @param   bool                    usingFW             Flag indicating FireWire
*                                                       port availability.
*  @param   bool                    usingEth            Flag indicating Ethernet
*                                                       port availability.
*  @param   BasePort *&             portUsingOne        Reference to receive
*                                                       first selected port.
*  @param   BasePort *&             portUsingTwo        Reference to receive
*                                                       second selected port.
*  @param   std::string             EthPortString       Display name for
*                                                       Ethernet port.
*  @param   std::string             FwPortString        Display name for
*                                                       FireWire port.
*  @param   std::string             ZynqPortString      Display name for
*                                                       Zynq port.
*  @param   BasePort *              FwPort              Pointer to FireWire
*                                                       port object.
*  @param   BasePort *              ZynqPort            Pointer to Zynq
*                                                       port object.
*  @param   EthBasePort *           EthPort             Pointer to Ethernet
*                                                       port object.
*  @param   const std::vector<AmpIO *> &ZynqBoardList   Vector of boards
*                                                       on Zynq port.
*  @param   const std::vector<AmpIO *> &FwBoardList     Vector of boards
*                                                       on FireWire port.
*  @param   const std::vector<AmpIO *> &EthBoardList    Vector of boards
*                                                       on Ethernet port.
*  @param   std::vector<AmpIO *> &  portUsingOneBoardList Reference to receive
*                                                       board list for first port.
*  @param   std::vector<AmpIO *> &  portUsingTwoBoardList Reference to receive
*                                                       board list for second port.
*
*  @note    Prompts user to enter two digits representing port choices
*           (e.g., "13" for Ethernet then Zynq).
*  @note    Validates input format and port availability before assignment.
*
*  @return  bool that indicates if the ports were successfully selected and 
*           assigned.
****************************************************************/
bool SelectPorts(bool usingZynq, bool usingFW, bool usingEth, BasePort *&portUsingOne, BasePort *&portUsingTwo,
                std::string EthPortString, std::string FwPortString, std::string ZynqPortString, 
                BasePort *FwPort, BasePort *ZynqPort, EthBasePort *EthPort, 
                const std::vector<AmpIO *> &ZynqBoardList,  const std::vector<AmpIO *> &FwBoardList,     
                const std::vector<AmpIO *> &EthBoardList, std::vector<AmpIO *> &portUsingOneBoardList, 
                std::vector<AmpIO *> &portUsingTwoBoardList) {
    std::cout << "Ports available: \n";
    if (usingEth) {
        std::cout << "1) " << EthPortString << "\n";
    } 
    if (usingFW) {
        std::cout << "2) " << FwPortString << "\n";
    } 
    if (usingZynq) {
        std::cout << "3) " << ZynqPortString << "\n";
    } 

    std::cout << "enter the two numbers of the ports next to each other to choose them \n";
    std::cout << "Example: enter '13' and then a newline to choose ethernet as the first port and Zynq as the second port\n";
    std::string input;
    if (!std::getline(std::cin, input)) {
        std::cout << "\nEOF detected. Port selection failed." << std::endl;
        return false;
    }

    input.erase(std::remove_if(input.begin(), input.end(), ::isspace), input.end());
    std::transform(input.begin(), input.end(), input.begin(), ::tolower);

    if (input.empty()) {
        std::cout << "No input provided. Port selection failed." << std::endl;
        return false;
    }
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
        if (usingEth && usingZynq) {
            portUsingOne = EthPort;
            portUsingTwo = ZynqPort;
            portUsingOneBoardList = EthBoardList;
            portUsingTwoBoardList = ZynqBoardList;
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
        if (usingFW && usingZynq) {
            portUsingOne = FwPort;
            portUsingTwo = ZynqPort;
            portUsingOneBoardList = FwBoardList;
            portUsingTwoBoardList = ZynqBoardList;
            return true;
        } else {
            return false;
        }
    } else if (input == "31") {
        if (usingZynq && usingEth) {
            portUsingOne = ZynqPort;
            portUsingTwo = EthPort;
            portUsingOneBoardList = ZynqBoardList;
            portUsingTwoBoardList = EthBoardList;
            return true;
        } else {
            return false;
        }
    } else if (input == "32") {
        if (usingZynq && usingFW) {
            portUsingOne = ZynqPort;
            portUsingTwo = FwPort;
            portUsingOneBoardList = ZynqBoardList;
            portUsingTwoBoardList = FwBoardList;
            return true;
        } else {
           return false;
        }
    } else {
        return false;
    }
}

/****************************************************************
*  @brief  Validate that both port selection and board selection
*          are successful for testing configuration.
*
*  @param   bool validPorts    Flag indicating successful port selection.
*  @param   bool validBoard    Flag indicating successful board selection.
*
*  @note    Simple validation function for configuration state checking.
*
*  @return  bool that indicates if the ports and board are valid
****************************************************************/
bool isValidConfig(bool validPorts, bool validBoard) {
    return validPorts && validBoard;
}

/****************************************************************
*  @brief  Display available communication ports and their
*          associated boards for user information.
*
*  @param   bool                       usingZynq        Flag indicating Zynq
*                                                       port availability.
*  @param   bool                       usingFW          Flag indicating FireWire
*                                                       port availability.
*  @param   bool                       usingEth         Flag indicating Ethernet
*                                                       port availability.
*  @param   std::string                EthPortString    Display name for
*                                                       Ethernet port.
*  @param   std::string                FwPortString     Display name for
*                                                       FireWire port.
*  @param   std::string                ZynqPortString   Display name for
*                                                       Zynq port.
*  @param   const std::vector<AmpIO *> &ZynqBoardList   Vector of boards
*                                                       on Zynq port.
*  @param   const std::vector<AmpIO *> &FwBoardList     Vector of boards
*                                                       on FireWire port.
*  @param   const std::vector<AmpIO *> &EthBoardList    Vector of boards
*                                                       on Ethernet port.
*
*  @note    Displays board IDs in hexadecimal format with comma separation.
*  @note    Only shows information for ports that are marked as available.
*
*  @return  none
****************************************************************/
void ListBoardsAndPorts(bool usingZynq, bool usingFW, bool usingEth, std::string EthPortString, std::string FwPortString, 
                        std::string ZynqPortString, const std::vector<AmpIO *> &ZynqBoardList, 
                        const std::vector<AmpIO *> &FwBoardList, const std::vector<AmpIO *> &EthBoardList) {
        if (usingEth) {
            std::cout << EthPortString << " is available as an Ethernet Port\n";
            std::cout << "available boards on this Ethernet Port: ";
            for (size_t i = 0; i < EthBoardList.size(); i++) {
                std::cout << std::hex << static_cast<unsigned int>(EthBoardList[i]->GetBoardId());
                if (i < EthBoardList.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << std::dec << "\n";
        }

        if (usingFW) {
            std::cout << FwPortString << " is available as a Firewire Port\n";
            std::cout << "available boards on this Firewire Port: ";
            for (size_t i = 0; i < FwBoardList.size(); i++) {
                std::cout << std::hex << static_cast<unsigned int>(FwBoardList[i]->GetBoardId());
                if (i < FwBoardList.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << std::dec << "\n";
        }

        if (usingZynq) {
            std::cout << ZynqPortString << " is available as a Zynq Port\n";
            std::cout << "available boards on this Zynq Port: ";
            for (size_t i = 0; i < ZynqBoardList.size(); i++) {
                std::cout << std::hex << static_cast<unsigned int>(ZynqBoardList[i]->GetBoardId());
                if (i < ZynqBoardList.size() - 1) {
                    std::cout << ", ";
                }
            }   
            std::cout << std::dec << "\n";
        }
}

// main
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

    std::vector<AmpIO *> ZynqBoardList;
    std::vector<AmpIO *> FwBoardList;
    std::vector<AmpIO *> EthBoardList;

    AmpIO *curBoard = 0;     // Current board via Ethernet or Firewire / Zynq-EMIO
    AmpIO *curBoardFw = 0;   // Current board via Firewire / Zynq-EMIO, sets boardNumFw
    AmpIO *curBoardZynq = 0;  // Current board via Zynq-EMIO
    AmpIO *curBoardEth = 0;  // Current board via Ethernet, sets boardNumEth

    BasePort *curPort = 0;   // Current port (Ethernet, Firewire or Zynq-EMIO)

    BasePort *FwPort = 0;    // Firewire port
    BasePort *ZynqPort = 0;    // Zynq-EMIO port
    EthBasePort *EthPort = 0;  // Ethernet port

    std::string EthPortString;
    std::string FwPortString;
    std::string ZynqPortString;
    std::string curPortString;

    // set up firewire and/or Zynq
    bool usingFW = false;
    bool usingZynq = false;
    bool usingEth = false;

    if (InitFireWire(FwPort, FwBoardList, port)) {
        std::cout << "FireWire Initialized \n";
        usingFW = true;
    } 

    if (InitZynq(ZynqPort, ZynqBoardList, port, isVerbose)) {
        std::cout << "Zynq Initialized \n";
        usingZynq = true;
    } 

    if (!(usingFW || usingZynq)) {
        std::cout << "Failed to initialize both Zynq and FireWire; need at least two ports for this test program \n";
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

    // set up Zynq boards
    if (usingZynq && ZynqBoardList.size() > 0) {
        curBoardZynq = ZynqBoardList[0];
        ZynqPortString = ZynqPort->GetPortTypeString();
        curBoard = curBoardZynq;
        curPort = ZynqPort;
        curPortString = ZynqPortString;
    }

    // ensure a board to setup ethernet
    if (!curBoardZynq && !curBoardFw) {
        std::cout << "No current board for Zynq and FireWire; failed to setup Zynq and FireWire \n";
        return -1;
    } else if (!curBoard && !curBoardZynq) {
        curBoard = curBoardFw;
    } else if (!curBoard && !curBoardFw) {
        curBoard = curBoardZynq;
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
            if (!(usingFW && usingZynq)) {
                std::cout << "Only one port is active; need at least two ports for this program" << std::endl;
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
            if (curPort == FwPort || curPort == ZynqPort) {
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
    } else if (!(usingFW && usingZynq)) {
        std::cout << "Only one port is active; need at least two ports for this program" << std::endl;
        return -1;
    }

    if ((!curBoardEth) + (!curBoardFw) + (!curBoardZynq) > 1) {
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
    validPorts = SelectPorts(usingZynq, usingFW, usingEth, portUsingOne, portUsingTwo, EthPortString, 
                    FwPortString, ZynqPortString, FwPort, ZynqPort, EthPort, ZynqBoardList, FwBoardList,
                    EthBoardList, portUsingOneBoardList, portUsingTwoBoardList);
    if (portUsingOne && portUsingTwo) { // ensure not null
        std::cout << "Port one now selected to be " << portUsingOne->GetPortTypeString() << "\n";
        std::cout << "Port two now selected to be " << portUsingTwo->GetPortTypeString() << "\n";
    }
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
    std::cout << "For example, only enter \"q\" or \"4\" or \"17\" and then a newline\n";
    
    std::string input;
    while (!done) {        
        std::cout << "-------------------------------------- Logistical Functions --------------------------------------\n";
        std::cout << "  q) Quit \n";
        std::cout << "  s) Ethernet status \n";
        std::cout << "  p) Change ports \n";
        std::cout << "  b) Change board \n";
        std::cout << "  c) Current ports and board (and their validity for use) \n";
        std::cout << "  l) list all ports and boards available \n";
        std::cout << "--------------------------------------    Test Functions    --------------------------------------\n";
        std::cout << "  0) two communications methods read; one register \n";
        std::cout << "  1) one communication method writes, one reads; one register \n";
        std::cout << "  2) one communication method writes, one reads; one register; stress test (both read correctly) \n";
        std::cout << "  3) two communication methods read status \n";
        std::cout << "  4) two communication methods write different things to status \n";
        std::cout << "  5) two communication methods write different things to status and then read status\n";
        std::cout << "  6) two communication methods read status and then write different things to status\n";
        std::cout << "  7) two communication methods read, then write different things, then read status\n";
        std::cout << "  8) two communication methods write different things, then read, then write different things to status\n";
        std::cout << "  9) one communication method writes to waveform and one reads from waveform\n";
        std::cout << "  10) one communication method reads status and one reads register \n";
        std::cout << "  11) one communication method writes to status and one reads register\n";
        std::cout << "  12) one communication method reads status and one write to register\n";
        std::cout << "  13) one communication methods writes to status, one writes to register\n";
        std::cout << "  14) two communication methods read both status and register\n";
        std::cout << "  15) two communication methods write to both status and register\n";
        std::cout << "  16) two communication methods write to and read from both status and register\n";


        if (!std::getline(std::cin, input)) {  // Check if getline failed
            std::cout << "\nEOF detected. Exiting program.\n";
            done = true;
        }
        
        input.erase(std::remove_if(input.begin(), input.end(), ::isspace), input.end());
        std::transform(input.begin(), input.end(), input.begin(), ::tolower);
        if (input.empty()) {  // Handle empty input gracefully
            std::cout << "Please enter a valid option.\n";
            continue;
        }

        if (input == "q") {
            std::cout << "Quitting program...\n";
            done = true;
        } else if (input == "s") {
            std::cout << "Checking Ethernet status...\n";
            PrintEthernetStatus(*curBoard);
        } else if (input == "p") {
            std::cout << "Changing ports...\n";
            validPorts = SelectPorts(usingZynq, usingFW, usingEth, portUsingOne, portUsingTwo, EthPortString, 
                    FwPortString, ZynqPortString, FwPort, ZynqPort, EthPort, ZynqBoardList, FwBoardList,
                    EthBoardList, portUsingOneBoardList, portUsingTwoBoardList);
                    if (portUsingOne && portUsingTwo) { // ensure not null
                        std::cout << "Port one now selected to be " << portUsingOne->GetPortTypeString() << "\n";
                        std::cout << "Port two now selected to be " << portUsingTwo->GetPortTypeString() << "\n";
                    }
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
        } else if (input == "l") {
            if (portUsingOne && portUsingTwo) { // ensure not null pointers
                ListBoardsAndPorts(usingZynq, usingFW, usingEth, EthPortString, FwPortString, ZynqPortString, ZynqBoardList, FwBoardList, EthBoardList);
            }
        } else if (input == "0") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 0: two communications methods read; one register\n";
                ReadSameRegisterTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "1") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 1: one communication method writes, one reads; one register\n";
                WriteAndReadOneRegisterDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "2") {
            if (isValidConfig(validPorts, validBoard)) {
                 std::cout << "Running test 2: stress test\n";
                 WriteAndReadOneRegisterDifferentMethodsStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "3") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 3: two communication methods read status\n";
                ReadDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "4") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 4: two communication methods write different things to status\n";
                WriteDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "5") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 5: write then read status\n";
                WriteThenReadDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum); 
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "6") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 6: read then write status\n";
                ReadThenWriteDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "7") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 7: read, write, read status\n";
                RapidReadWriteReadDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "8") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 8: write, read, write status\n";
                RapidWriteReadWriteDifferentThingsDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum); 
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "9") {
             if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 9: waveform write/read\n";
                WaveformReadAndWriteDifferentMethodsTest(portUsingOne, portUsingTwo, curBoardNum, selectedBoardOne);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "10") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 10: one reads status, one reads register\n";
                ReadRegisterAndStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "11") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 11: one writes status, one reads register\n";
                ReadRegisterWriteStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "12") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 12: one reads status, one writes register\n";
                WriteRegisterReadStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "13") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 13: one writes status, one writes register\n";
                WriteRegisterWriteStatusTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "14") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 14: two methods read both status and register\n";
                ReadRegisterAndStatusStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "15") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 15: two methods write to both status and register\n";
                WriteRegisterAndStatusStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
            }
        } else if (input == "16") {
            if (isValidConfig(validPorts, validBoard)) {
                std::cout << "Running test 16: two methods write/read both status and register\n";
                AlternatingReadAndWriteRegisterAndStatusStressTest(portUsingOne, portUsingTwo, curBoardNum);
            } else {
                 std::cout << "Current ports and board configuration is invalid; test not run\n";
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
    if (ZynqPort) {
        for (unsigned int bd = 0; bd < ZynqBoardList.size(); bd++) {
            ZynqPort->RemoveBoard(ZynqBoardList[bd]->GetBoardId());
            delete ZynqBoardList[bd];
        }
        delete ZynqPort;
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
