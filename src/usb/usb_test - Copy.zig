const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;
const peripherals = hal.peripherals;
const zusb = microzig.usb;

const LED_PIN = hal.parse_pin(board.pin_map.LED);

pub const std_options = struct {
    pub const log_level = .debug;
    pub const logFn = hal.usart.log;
};

const USBM_PIN = hal.parse_pin("PA11");
const USBD_PIN = hal.parse_pin("PA12");

var num_interrupts: u32 = 0;

const USB_PMASIZE = 0x200; // size in BYTES! -> 256 u16
const USB_PMRRECS = USB_PMASIZE / 2; // number of PMA record 256 u16

const USB_NUMBDT = 8;
const USB_PMAADDR = 0x40006000;
pub const usb_pma = @intToPtr([*]volatile PmaElem, USB_PMAADDR);
pub const usb_bdt = @intToPtr([*]volatile BufferDescriptorTableEntry, USB_PMAADDR); // place BufferDescriptorTableEntry[USB_NUMBDT] at start of PMA memory
pub const usb_epr = @ptrCast([*]volatile u32, peripherals.USB);

const EPCOUNT = 8;

pub const PmaElem = packed struct {
    data: u16,
    reserved: u16,
};

// needed for BufferDescriptorTableEntry? then remove
pub const PMA_REC = packed struct {
    addr: PmaElem,
    cnt: PmaElem,
};

const BufferDescriptorTableEntry = packed struct {
    tx_addr: PmaElem,
    tx_cnt: PmaElem,
    rx_addr: PmaElem,
    rx_cnt: PmaElem,
};

//const  USB_EPREG_MASK = USB_EP_CTR_R;
//|USB_EP_T_FIELD|USB_EP_KIND|USB_EP_CTR_TX|USB_EPADDR_FIELD)

pub const ISTR = struct {

    // *******************  Bit definition for USB_ISTR register  *******************
    pub const USB_ISTR_EP_ID_POS = 0;
    pub const USB_ISTR_EP_ID_MASK = 0b1111 << USB_ISTR_EP_ID_POS;
    pub const USB_ISTR_EP_ID = USB_ISTR_EP_ID_MASK;
    pub const USB_ISTR_DIR_POS = 4;
    pub const USB_ISTR_DIR_MASK = 0b1 << USB_ISTR_DIR_POS;
    pub const USB_ISTR_DIR = USB_ISTR_DIR_MASK;
    pub const USB_ISTR_ESOF_POS = 8;
    pub const USB_ISTR_ESOF_MASK: u32 = 0x1 << USB_ISTR_ESOF_POS;
    pub const USB_ISTR_ESOF = USB_ISTR_ESOF_MASK;
    pub const USB_ISTR_SOF_POS = 9;
    pub const USB_ISTR_SOF_MASK: u32 = 0x1 << USB_ISTR_SOF_POS;
    pub const USB_ISTR_SOF = USB_ISTR_SOF_MASK;
    pub const USB_ISTR_RESET_POS = 10;
    pub const USB_ISTR_RESET_MASK: u32 = 0x1 << USB_ISTR_RESET_POS;
    pub const USB_ISTR_SUSP_POS = 11;
    pub const USB_ISTR_SUSP_MASK: u32 = 0b1 << USB_ISTR_SUSP_POS;
    pub const USB_ISTR_WKUP_POS = 12;
    pub const USB_ISTR_WKUP_MASK: u32 = 0b1 << USB_ISTR_WKUP_POS;
    pub const USB_ISTR_ERR_POS = 13;
    pub const USB_ISTR_ERR_MASK: u32 = 0b1 << USB_ISTR_ERR_POS;
    pub const USB_ISTR_PMAOVR_POS = 14;
    pub const USB_ISTR_PMAOVR_MASK: u32 = 0b1 << USB_ISTR_PMAOVR_POS;
    pub const USB_ISTR_CTR_POS = 15;
    pub const USB_ISTR_CTR_MASK: u32 = 0b1 << USB_ISTR_CTR_POS;

    // get the register value
    pub fn get() u32 {
        return peripherals.USB.ISTR.raw;
    }

    pub fn clear_pmaovr() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_PMAOVR_MASK);
    }

    pub fn clear_err() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_ERR_MASK);
    }

    pub fn clear_wkup() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_WKUP_MASK);
    }

    pub fn clear_susp() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_SUSP_MASK);
    }

    pub fn clear_reset() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_RESET_MASK);
    }
    pub fn clear_ctr() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_CTR_MASK);
    }

    pub fn clear_sof() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_SOF_MASK);
    }

    pub fn clear_esof() void {
        peripherals.USB.ISTR.write_raw(0xFFFFFFFF & ~USB_ISTR_ESOF_MASK);
    }
};

pub const StatusTxRx = enum(u2) {
    disabled,
    stall,
    nak,
    valid,
};

pub const EPRegs = struct {

    // get the register value
    pub fn get(ep_num: usize) u32 {
        return usb_epr[ep_num];
    }

    const EP_CTR_RX = 15;
    pub const EP_CTR_RX_MASK = (0b1 << EP_CTR_RX);

    const EP_DTOG_RX = 14;
    const EP_DTOG_RX_MASK = (0b1 << EP_DTOG_RX);

    const EP_STAT_RX = 12;
    const EP_STAT_RX_MASK = (0b11 << EP_STAT_RX);

    const EP_STAT_SETUP = 11;
    const EP_STAT_SETUP_MASK = (0b1 << EP_STAT_SETUP);

    const EP_FIELD = 9;
    const EP_FIELD_MASK = (0b11 << EP_FIELD);

    const EP_KIND = 8;
    const EP_KIND_MASK = (0b1 << EP_KIND);

    const EP_CTR_TX = 7;
    const EP_CTR_TX_MASK = (0b1 << EP_CTR_TX);

    const EP_DTOG_TX = 6;
    const EP_DTOG_TX_MASK = (0b1 << EP_DTOG_TX);

    const EP_STAT_TX = 4;
    const EP_STAT_TX_MASK = (0b11 << EP_STAT_TX);

    const EP_EA = 0;
    const EP_EA_MASK = (0b1111 << EP_EA);

    pub const EP_TX_DIS = 0x00000000; // EndPoint TX DISabled
    pub const EP_TX_STALL = 0x00000010; // EndPoint TX STALLed
    pub const EP_TX_NAK = 0x00000020; // EndPoint TX NAKed
    pub const EP_TX_VALID = 0x00000030; // EndPoint TX VALID
    pub const EPTX_DTOG1 = 0x00000010; // EndPoint TX Data TOGgle bit1
    pub const EPTX_DTOG2 = 0x00000020; // EndPoint TX Data TOGgle bit2

    pub const EP_RX_DIS = 0x00000000; // EndPoint RX DISabled
    pub const EP_RX_STALL = 0x00001000; // EndPoint RX STALLed
    pub const EP_RX_NAK = 0x00002000; // EndPoint RX NAKed
    pub const EP_RX_VALID = 0x00003000; // EndPoint RX VALID
    pub const EPRX_DTOG1 = 0x00001000; // EndPoint RX Data TOGgle bit1
    pub const EPRX_DTOG2 = 0x00002000; // EndPoint RX Data TOGgle bit1

    // These bits are cleared when written with 0 and remain unchanged when written with 1.
    const rc_w0: u16 = EP_CTR_RX_MASK | EP_CTR_TX_MASK;
    // These bits are toggled when written with 1 and remain unchanged when written with 0.
    const toggle: u16 = EP_DTOG_RX_MASK | EP_STAT_RX_MASK | EP_DTOG_TX_MASK | EP_STAT_TX_MASK;
    // These bits behave normally, meaning the written value is directly taken.
    const rw: u16 = EP_FIELD_MASK | EP_KIND_MASK | EP_EA_MASK;

    // * Because the individual bits of the EPnR registers have to be set in different ways and when writing
    // * care must be taken that the wrong bits are not accidentally written,
    // * This function encapsulates write access. The EP parameter specifies the index of the register. All
    // * Bits to be written (regardless of value) must be set to 1 in "mask";
    // * if mask=0, nothing is written at all. The actual values ​​to be written are stored in "data"
    // * specified. Bits which are 0 in "mask" are ignored in "data". In "old" becomes the previous one
    // * Pass the status of the register if it has already been queried beforehand. Isn't that
    // * is the case, the overloaded function can be used without this parameter.
    pub fn setEPnR(ep_num: usize, mask: u32, data: u32, old: u32) void {
        var wr0: u32 = rc_w0 & (~mask | data);
        var wr1: u32 = (mask & toggle) & (old ^ data);
        var wr2: u32 = rw & ((old & ~mask) | data);

        var valtowrite = wr0 | wr1 | wr2;
        usb_epr[ep_num] = valtowrite;
        // std.log.info("EP XXX  MASK:{b:0>16} OLD MASK :{b:0>16} OR DATA :{b:0>16} wr2:{b:0>16}",
        // .{ @truncate(u16, ~mask),
        //     @truncate(u16, old & ~mask),
        //     @truncate(u16, (old & ~mask) | data),
        //      @truncate(u16, wr2)});

        // std.log.info("EP val2write:{b:0>16} wr0:{b:0>16} wr1:{b:0>16} wr2:{b:0>16} toggle:{b:0>16}", .{ @truncate(u16, valtowrite), @truncate(u16, wr0), @truncate(u16, wr1), @truncate(u16, wr2), @truncate(u16, toggle) });
        //std.log.info("EP {} NEW:{b:0>16} MASK:{b:0>16} DATA:{b:0>16} OLD:{b:0>16}", .{ ep_num, @truncate(u16, usb_epr[ep_num]), @truncate(u16, mask), @truncate(u16, data), @truncate(u16, old) });
    }

    pub fn set_kind(ep_num: usize) void {
        _ = ep_num;
    }

    pub fn set_type(ep_num: usize, ep_type: EP_CONFIG_TYPES) void {
        const val = std.math.shl(u32, @enumToInt(ep_type), EP_FIELD);
        setEPnR(ep_num, EP_FIELD_MASK, val, get(ep_num));
    }

    pub fn set_tx_status(ep_num: usize, comptime status: StatusTxRx) void {
        const val = switch (status) {
            StatusTxRx.disabled => 0b00 << EP_STAT_TX,
            StatusTxRx.stall => 0b01 << EP_STAT_TX,
            StatusTxRx.nak => 0b10 << EP_STAT_TX,
            StatusTxRx.valid => 0b11 << EP_STAT_TX,
        };
        setEPnR(ep_num, EP_STAT_TX_MASK, val, get(ep_num));
    }

    pub fn set_rx_status(ep_num: usize, comptime status: StatusTxRx) void {
        const val = switch (status) {
            StatusTxRx.disabled => 0b00 << EP_STAT_RX,
            StatusTxRx.stall => 0b01 << EP_STAT_RX,
            StatusTxRx.nak => 0b10 << EP_STAT_RX,
            StatusTxRx.valid => 0b11 << EP_STAT_RX,
        };
        setEPnR(ep_num, EP_STAT_RX_MASK, val, get(ep_num));
    }

    pub fn clear_ctr_rx(ep_num: usize) void {
        setEPnR(ep_num, EP_CTR_RX_MASK, 0, get(ep_num));
    }

    pub fn clear_ctr_tx(ep_num: usize) void {
        setEPnR(ep_num, EP_CTR_TX_MASK, 0, get(ep_num));
    }

    pub fn set_addr(ep_num: usize, ea: u4) void {
        setEPnR(ep_num, EP_EA_MASK, ea, get(ep_num));
    }
};

fn map_mem_addr(mcu_address: u32) u13 {
    return @intCast(u13, (mcu_address - USB_PMAADDR) / 2);
}

// Returns next available PMA buffer.
// size:  Requested buffer size (in bytes!).
// returns Buffer address for PMA table.
// note PMA buffers grown from top to bottom like stack.

fn get_next_pma(size: u16) u16 {
    const USB_EPTABLE_SIZE = 0x20; // effective! buffer size of table, 4 entr CHECK CHECK CHECK
    var result: u16 = USB_PMASIZE; // start on TOP

    for (0..USB_NUMBDT) |idx| {
        var bdt = usb_bdt[idx];

        if ((bdt.tx.addr) and (bdt.tx.addr < result)) result = bdt.tx.addr;
        if ((bdt.rx.addr) and (bdt.rx.addr < result)) result = bdt.rx.addr;
    }

    if (result < (USB_EPTABLE_SIZE + size)) {
        return 0;
    } else {
        return result - size;
    }
}

// return frame number
fn usb_get_frame() u16 {
    return @bitCast(u16, peripherals.USB.FNR.FN);
}

fn usb_init() void {

    // DISABLE USB
    peripherals.RCC.APB1ENR.modify(.{ .USBEN = 0b0 });

    // USB pullup (for bluepill devices with incorrect pull up resistor)
    // http://amitesh-singh.github.io/stm32/2017/05/27/Overcoming-wrong-pullup-in-blue-pill.html
    // see https://www.mikrocontroller.net/articles/USB-Tutorial_mit_STM32
    hal.gpio.set_output(USBD_PIN, hal.gpio.OutputMode.opendrain, hal.gpio.OutputSpeed.output_50MHz);
    hal.gpio.write(USBD_PIN, hal.gpio.State.low);
    hal.systick.delay_ms(80);
    hal.gpio.set_input(USBD_PIN, hal.gpio.InputMode.floating);

    // Turn on USB interrupt
    // NVIC_EnableIRQ ( USB_LP_CAN1_RX0_IRQn );

    // Use 4 bits for 'priority' and 0 bits for 'subpriority'.
    peripherals.SCB.AIRCR.modify(.{
        .PRIGROUP = 0,
    });

    // CAN1_RX0 = 20
    // const USART2_IRQn = 38;
    // PRIO GROUP = 20 / 4 = 5
    // PRIO SGROUP = 20 % 0 = 2
    peripherals.NVIC.IPR5.modify(.{
        .IPR_N0 = 1,
    });
}

fn usb_connect() void {

    // ENABLE USB
    peripherals.RCC.APB1ENR.modify(.{ .USBEN = 0b1 });

    // RESET USB
    peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b1 });
    peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b0 });

    // clear all PMA memory, which includes the BDTs
    for (0..USB_PMRRECS) |idx| {
        usb_pma[idx].data = 0x00;
    }

    peripherals.USB.CNTR.modify(.{
        .FRES = 0b1,
    });

    peripherals.USB.CNTR.modify(.{
        .PDWN = 0b0,
    });

    hal.systick.delay_ms(1);

    // tstartup = 1us
    hal.systick.delay_ms(1);

    peripherals.USB.CNTR.modify(.{
        .CTRM = 0b1,
        .RESETM = 0b1,
        // .ERRM = 0b1,
        // //.SOFM = 0b1,
        // .SUSPM = 0b1,
        // .WKUPM = 0b1,
    });

    peripherals.USB.ISTR.write_raw(0x0);
    peripherals.USB.BTABLE.write_raw(0x0);

    peripherals.USB.CNTR.modify(.{
        .FRES = 0b0,
    });

    // Turn on USB interrupt
    // NVIC_EnableIRQ ( USB_LP_CAN1_RX0_IRQn );
}

fn usb_set_addr(addr: u7) void {
    peripherals.USB.DADDR.modify(.{ .ADD = addr, .EF = 0b1 });
}

const EPReg = microzig.mmio.Mmio(packed struct(u32) {
    /// Endpoint address
    EA: u4,
    /// Status bits, for transmission transfers
    STAT_TX: u2,
    /// Data Toggle, for transmission transfers
    DTOG_TX: u1,
    /// Correct Transfer for transmission
    CTR_TX: u1,
    /// Endpoint kind
    EP_KIND: u1,
    /// Endpoint type
    EP_TYPE: u2,
    /// Setup transaction completed
    SETUP: u1,
    /// Status bits, for reception transfers
    STAT_RX: u2,
    /// Data Toggle, for reception transfers
    DTOG_RX: u1,
    /// Correct transfer for reception
    CTR_RX: u1,
    padding: u16,
});

// fn usb_ep_deconfig(ep_num: usize) void {
//     var bdt = usb_bdt[ep_num];
//     bdt.tx_addr.data = 0;
//     bdt.rx_cnt.data = 0;
//     bdt.tx_addr.data = 0;
//     bdt.tx_cnt.data = 0;

//     EPRegs.reset(ep_num);
// }

fn usb_init_device() void {

    // This function is called when the host resets the device ("RESET"), possibly also when booting.
    // It should set the hardware and software to a defined initial state so that the driver on the host side of a
    // clean state.

    // set address of Buffer Description Table (PMA) address
    //peripherals.USB.BTABLE.write_raw(  map_mem_addr(@ptrToInt(usb_bdt)));

    // new address will be zero
    //peripherals.USB.DADDR.modify(.{ .EF = 0b1, .ADD = 0 });

    // Initalize all endpoints
    // 	for (EPBuffer* ep : m_epBuffers)
    // 		if (ep)
    // 			ep->onReset ();
    // }

}

const EP_CONFIG_TYPES = enum(u2) {
    ep_bulk,
    ep_control,
    ep_isochronous,
    ep_interrupt,
};

pub const EpConfigEntry = struct {
    number: usize,
    ep_type: EP_CONFIG_TYPES,
    tx_max: u16,
    rx_max: u16,
};

pub const EpConfigEntries = [_]EpConfigEntry{
    .{
        .number = 0,
        .ep_type = EP_CONFIG_TYPES.ep_control,
        .tx_max = 32,
        .rx_max = 32,
    },
    .{
        .number = 1,
        .ep_type = EP_CONFIG_TYPES.ep_interrupt,
        .tx_max = 16,
        .rx_max = 16,
    },
};

fn usb_reset() void {

    // start on top
    var addr: u16 = USB_PMASIZE;

    for (0..EPCOUNT) |idx| {
        if (idx < EpConfigEntries.len) {
            const epc = EpConfigEntries[idx];

            // no read
            //var bdt : BufferDescriptorTableEntry = BufferDescriptorTableEntry{}; //// {};
            var bdt = usb_bdt[epc.number];

            addr = addr - epc.tx_max;
            bdt.tx_addr.data = addr;
            bdt.tx_cnt.data = 0;

            addr -= epc.rx_max;
            bdt.rx_addr.data = addr;

            // todo assert values in range
            if (epc.rx_max > 62) {
                bdt.rx_cnt.data = 0x8000 | (epc.rx_max / 32) << 10;
            } else {
                bdt.rx_cnt.data = (epc.rx_max / 2) << 10;
            }
            std.log.info("bdt.rx_cnt.data set to {}", .{bdt.rx_cnt.data});

            usb_bdt[epc.number] = bdt;

            // usb_ep_deconfig(idx);
            EPRegs.set_addr(epc.number, @intCast(u4, epc.number));
            const ep_tp = epc.ep_type;

            EPRegs.set_type(epc.number, ep_tp);
            EPRegs.set_rx_status(idx, StatusTxRx.valid);
            EPRegs.set_tx_status(idx, StatusTxRx.nak);
        } else {
            EPRegs.set_tx_status(idx, StatusTxRx.disabled);
            EPRegs.set_rx_status(idx, StatusTxRx.disabled);
        }
    }

    peripherals.USB.CNTR.modify(.{
        .CTRM = 0b1,
        .RESETM = 0b1,
        .SUSPM = 0b1,
        .ERRM = 0b1,
        .SOFM = 0b1,
        // .WKUPM = 0b1,
    });

    peripherals.USB.ISTR.write_raw(0);
    peripherals.USB.BTABLE.write_raw(map_mem_addr(@ptrToInt(usb_bdt)));
    peripherals.USB.DADDR.modify(.{ .EF = 0b1, .ADD = 0 });

    std.log.info("RESET COMPLETE DUMP REGS", .{});
    dump_ep_regs();
    //dump_bdt_mem();
}

pub const MyDeviceDescription = [18]u8{
    18, // bLength
    1, // bDescriptorType: Device
    0x00, 0x02, // bcdUSB
    0xFF, // bDeviceClass: Vendor-specific
    0xFF, // bDeviceSubClass: ignored
    0xFF, // bDeviceProtocol: ignored
    32, // bMaxPacketSize0: allowed : 8,16,32,64
    0xAD, 0xDE, // idVendor
    0xEF, 0xBE, // idProduct
    0x00, 0x01, // bcdDevice
    0x00, // iManufacturer: No string descriptor 0
    0x00, // iProduct: No string descriptor 0
    0x00, // SerialNb: No string descriptor 0
    0x01, // bNumConfigurations
};

// *****************************************************************************
// USB Endpoint Interrupt Handler
// ***************************************************************************** */
fn USB_EPHandler(status: u32) void {
    var ep_num = status & 0x0F;
    var ep = EPRegs.get(ep_num);
    std.log.info("USB_EPHandler({}) status: 0b{b:0>16} ep 0b{b:0>16}  FNR 0b{b:0>16}", .{ ep_num, @truncate(u16, status), @truncate(u16, ep), @truncate(u16, peripherals.USB.FNR.raw) });
    dump_bdt_mem();
    //dump_pma_mem_range(0, 256);

    EPRegs.clear_ctr_rx(ep_num);
    EPRegs.clear_ctr_tx(ep_num);

    if ((ep & EPRegs.EP_CTR_RX_MASK) != 0) {
        var bdt = usb_bdt[ep_num];

        var start = bdt.rx_addr.data / 2;
        var length = (bdt.rx_cnt.data & 0x3FF);
        std.log.info("EP_CTR_RX_MASK start: 0x{x:0>4} len {}", .{ @truncate(u16, start), @truncate(u16, length) });
        dump_pma_mem_range(start, length / 2);

        // is setup? check also DIR?
        if ((ep & EPRegs.EP_STAT_SETUP_MASK) != 0) {
            var ptrTest = @ptrCast(*zusb.SetupPacket, &usb_pma[start].data);
            std.log.info("ptrTest  : 0x{x:0>4} 0x{x:0>4}", .{ @truncate(u16, ptrTest.request_type), @truncate(u16, ptrTest.request) });

            var rqtype = usb_pma[start].data;
            //var wvalue = usb_pma[start + 1].data;
            //var windex = usb_pma[start + 2].data;
            var wlength = usb_pma[start + 3].data;

            std.log.info("IS SETUP  start: 0x{x:0>4} len {}", .{ @truncate(u16, start), @truncate(u16, length) });
            if (rqtype == 0x0680) {
                // get descriptor
                std.log.info("GET DESCRIPTOR", .{});

                var xmit_len = std.math.min(@intCast(u16, MyDeviceDescription.len), wlength);

                // send MyDeviceDescription
                var w_idx = bdt.tx_addr.data / 2;

                // buffer to PMA, factor out this code
                for (0..xmit_len / 2) |idx| {
                    var val = @intCast(u16, MyDeviceDescription[2 * idx + 0]) + (std.math.shl(u16, @intCast(u16, MyDeviceDescription[2 * idx + 1]), 8));
                    usb_pma[w_idx + idx].data = val;
                }
                // debug

                dump_pma_mem_range(w_idx, xmit_len / 2);

                // update descriptor lengths and assign
                bdt.rx_addr.data &= (0b1111110000000000);
                bdt.tx_cnt.data = xmit_len;
                usb_bdt[ep_num] = bdt;
                EPRegs.set_tx_status(ep_num, StatusTxRx.valid);
            }
        }
    } else if ((ep & EPRegs.EP_CTR_TX_MASK) != 0) {
        // receive again
        std.log.info("EP_CTR_TX_MASK completed, setup receive again", .{});
        EPRegs.set_rx_status(ep_num, StatusTxRx.valid);
    }

    //endpoint number where occured
    //     uint8_t needAnswer = 0; //flag need ZLP or initiate send data
    //   #ifdef SWO_USB_LOG
    //     pFloat = stradd (pFloat,"\r\nEP=");
    //     pFloat = itoa(EP ,pFloat,2,0);
    //   #endif
    //     if (EP & EP_CTR_RX)     //something received ?
    //     {
    //       USB->EPR[EPn] &= 0x78f; //reset flag CTR_RX
    //     #ifdef SWO_USB_LOG
    //       pFloat = stradd (pFloat,"\r\nReceived in EP[");
    //       pFloat = itoa(EPn ,pFloat,10,0);
    //       pFloat = stradd (pFloat,"]: ");
    //     #endif
}

fn usb_poll() void {
    while (true) {
        var istr = peripherals.USB.ISTR.read();
        var ep: usize = istr.EP_ID;
        _ = ep;

        if (istr.SUSP != 0) {
            ISTR.clear_susp();
            if (peripherals.USB.DADDR.read().ADD != 0) {
                std.log.info("SUSP !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
                peripherals.USB.DADDR.modify(.{ .ADD = 0 });
                ISTR.clear_susp();
                ISTR.clear_wkup();
            } else {
                std.log.info(" S", .{});
            }
        } else if (istr.RESET != 0) {
            // A "RESET" occurs when there are missing packets from the host. This is the case at the beginning of every connection,
            // until the host recognizes the device.
            std.log.info("RESET !!!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
            usb_reset();
            //ISTR.clear_reset();
        } else if (istr.CTR != 0) {

            // // Richtung des letzten Transfers. false bedeutet "IN" transfer (Device->Host), true bedeutet "IN" oder "OUT"
            // var dir = istr.DIR; // 0=TX, 1=RX/TX
            // // Die Nummer des EPnR-Registers, welches zu diesem Transfer gehört.
            // var ep_id = istr.EP_ID;
            // var ep_val = EPRegs.get(ep_id);
            // std.log.info("CTR !!! ISTR 0b{b:0>16} FNR 0b{b:0>16} DADDR 0x{x:0>4} DIR: {} ep: {} EPVAL: 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw), @truncate(u16, peripherals.USB.DADDR.raw), dir, ep_id, @truncate(u16, ep_val) });

            // // Stelle sicher, dass das EPnR Register existiert
            // // Lösche im EPnR-Register die RX/TX-Flags, falls sie gesetzt sind. Falls die Hardware zwischen Abfragen und Löschen
            // // eines der Bits setzt, wird dies nicht gelöscht und im nächsten Schleifendurchlauf behandelt.
            // EPRegs.set_rx_status(ep_id, StatusTxRx.disabled);
            // EPRegs.set_tx_status(ep_id, StatusTxRx.disabled);

            USB_EPHandler(peripherals.USB.ISTR.raw);
            ISTR.clear_ctr();
        } else if (istr.SOF != 0) {
            if (peripherals.USB.EP0R.read().STAT_TX == EPRegs.EP_TX_STALL) {
                // send NAK
                std.log.info("STALL -> NAK !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
                EPRegs.set_tx_status(0, StatusTxRx.nak);
            }
            ISTR.clear_sof();

            // todo SEND DATA

        } else if (istr.ESOF != 0) {
            //std.log.info("ESOF !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
            ISTR.clear_esof();
        } else if (istr.PMAOVR != 0) {
            std.log.info("PMAOVR !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
            ISTR.clear_pmaovr();
        } else if (istr.WKUP != 0) {
            std.log.info("WKUP !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
            ISTR.clear_wkup();
        } else if (istr.ERR != 0) {
            std.log.info("ERR !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
            ISTR.clear_err();
        } else {
            peripherals.USB.ISTR.write_raw(0);
            // std.log.info("GOT ISTR RESET !!!! 0b{b:0>8}", .{peripherals.USB.ISTR.raw});
            break;
        }
    }

    // if (istr.RESET != 0) {
    //    peripherals.USB.ISTR.modify(.{ .RESET = 0b0 });
    //    std.log.info("GOT ISTR RESET !!!!", .{});

    //    // BDT table is placed at bottom of PMA memory
    //    //peripherals.USB.BTABLE.BT.BTABLE = map_mem_addr(&usb_bdt[0]);
    //    peripherals.USB.BTABLE.write_raw( map_mem_addr(@ptrToInt(usb_bdt)));

    //    for (0..USB_NUMBDT) |idx| {
    //        usb_ep_deconfig(idx);
    //    }
    // }
}

pub fn main() !void {

    // setup high speed configuration, board has a 8MHz crystal
    hal.rcc.setup_high_performance(8_000_000);

    // use USART2 for logging
    var uart = hal.usart.num(2);
    uart.apply(115200);

    hal.systick.apply();

    //hal.time.init();
    hal.usart.init_logger(uart);
    hal.gpio.set_output(LED_PIN, hal.gpio.OutputMode.pushpull, hal.gpio.OutputSpeed.output_10MHz);

    // main loop
    var loop_idx: u32 = 0;

    std.log.info("USB Test...", .{});

    usb_init();
    std.log.info("USB Init Done...", .{});

    usb_connect();
    std.log.info("USB connect Done...", .{});

    // const mpa = map_mem_addr(@ptrToInt(&usb_bdt[1]));
    // const epr2 = usb_epr;

    // std.log.info("TEST: {} 0x{X:0>8} 0x{X:0>8} ", .{ mpa, @ptrToInt(usb_epr), @ptrToInt(epr2) });

    // std.log.info("TEST111: {} 0x{X:0>8} 0x{X:0>8} ", .{ mpa, peripherals.USB.EP0R.raw, EPRegs.get(7) });
    // peripherals.USB.EP7R.write_raw(0xFFFFFFFF);
    // std.log.info("TEST222: {} 0x{X:0>8} 0x{X:0>8} ", .{ mpa, peripherals.USB.EP0R.raw, EPRegs.get(7) });

    // const bs1 = @bitOffsetOf(microzig.chip.types.USB, "CTR_TX");
    // const bs2 = @bitOffsetOf(microzig.chip.types.USB.EP0R, "CTR_RX");

    // std.log.info("TEST333: {} {}", .{ bs1, bs2 });

    //dump_pma_mem();
    //dump_bdt_mem();

    microzig.cpu.disable_interrupts();

    // const RX0_IRQn = 20;
    // // PRIO GROUP = 20 / 4 = 5
    // // PRIO SGROUP = 20 % 4 = 1
    // hal.peripherals.NVIC.IPR5.modify(.{
    //    .IPR_N0 = 1,
    // });

    // var val = hal.peripherals.NVIC.ISER0.read();
    // val.SETENA = val.SETENA | (1 << (RX0_IRQn));
    // hal.peripherals.NVIC.ISER0.write(val);

    microzig.cpu.enable_interrupts();

    //dump_regs();

    while (true) {
        usb_poll();

        //uart.put_hex(loop_idx);
        //uart.put_char('.');
        loop_idx += 1;
        if ((loop_idx % 5000) == 0) {
            var istr = peripherals.USB.ISTR;
            std.log.info("Loops: {} interrupts: {} ISTR 0x{X:0>8}", .{ loop_idx, num_interrupts, istr.raw });
        }

        hal.systick.delay_ms(1);
    }
}

// USB interrupt handling function
pub const microzig_options = struct {
    pub const interrupts = struct {
        pub fn CAN_RX0() void {
            num_interrupts += 1;
        }
    };
};

pub fn dump_pma_mem() void {
    std.log.info("==== PMA memory dump ====", .{});
    for (0..USB_NUMBDT) |idx| {
        std.log.info("PMA[{}] 0x{X:0>4} ", .{ idx, usb_pma[idx].data });
    }
}

// range in words!! so from 0..255
pub fn dump_pma_mem_range(start: usize, length: usize) void {
    for (start..start + length) |idx| {
        std.log.info(" 0x{X:0>4} 0x{X:0>4} ", .{ idx, usb_pma[idx].data });
    }
}

pub fn dump_regs() void {
    std.log.info("==== USB REGS dump ====", .{});
    std.log.info("USB.BTABLE 0x{X:0>8}", .{peripherals.USB.BTABLE.raw});
}

pub fn dump_bdt_mem() void {
    std.log.info("==== BDT memory dump ====", .{});
    for (0..USB_NUMBDT) |idx| {
        const bdt = usb_bdt[idx];
        std.log.info("usb_pma {} 0x{X:0>4} 0x{X:0>4} 0x{X:0>4} 0x{X:0>4} ", .{ idx, bdt.tx_addr.data, bdt.tx_cnt.data, bdt.rx_addr.data, bdt.rx_cnt.data });
    }
}

pub fn dump_ep_regs() void {
    std.log.info("==== EP regs dump ====", .{});
    for (0..USB_NUMBDT) |idx| {
        std.log.info("ep {}  0x{X:0>4} ", .{ idx, @truncate(u16, EPRegs.get(idx)) });
    }
}

pub fn test_bdtmem() void {
    //var tptr = @ptrToInt(&TPTR.*);
    //std.log.info("TPRE  0x{X:0>8} ", .{tptr});
    var ptr = @ptrToInt(&(usb_pma[255]));
    std.log.info("PMA  0x{X:0>8} ", .{ptr});

    for (0..(USB_PMRRECS + 1)) |idx| {
        usb_pma[idx].data = 0xFACE +% @intCast(u16, idx);
    }

    // //@memset(USB_PMA, 0xBA);
    // var ptr = @ptrToInt(&( USB_PMA.mem[1]));
    std.log.info("usb_pma 0   0x{X:0>4} 0x{X:0>4} ", .{ usb_pma[0].data, usb_pma[0].reserved });
    std.log.info("usb_pma 1   0x{X:0>4} 0x{X:0>4} ", .{ usb_pma[1].data, usb_pma[1].reserved });
    std.log.info("usb_pma 2   0x{X:0>4} 0x{X:0>4} ", .{ usb_pma[2].data, usb_pma[2].reserved });
    std.log.info("usb_pma 254 0x{X:0>4} 0x{X:0>4} ", .{ usb_pma[254].data, usb_pma[254].reserved });
    std.log.info("usb_pma 255 0x{X:0>4} 0x{X:0>4} ", .{ usb_pma[255].data, usb_pma[255].reserved });
    // out of bounds check std.log.info("usb_pma 256 0x{X:0>4} 0x{X:0>4} ", .{ usb_pma[256].data, usb_pma[256].reserved });
    dump_bdt_mem();
}

pub fn test_xor() void {
    const tst_reg = 2;

    // for (0..10) |_| {
    //     std.log.info("BEFORE TEST 0b11  0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});
    //     usb_epr[tst_reg] = 0b1000_0000_1000_0000 | 0b0000_0000_0011_0000;
    //     std.log.info("AFTER TEST  0b11 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});
    // }
    // std.log.info(" =============== ", .{});

    // for (0..10) |_| {
    //     std.log.info("BEFORE TEST 0b01  0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});
    //     usb_epr[tst_reg] = 0b1000_0000_1000_0000 | 0b0000_0000_0001_0000;
    //     std.log.info("AFTER TEST  0b01 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});
    // }

    // std.log.info(" =============== ", .{});

    // for (0..10) |_| {
    //     std.log.info("BEFORE TEST 0b10  0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});
    //     usb_epr[tst_reg] = 0b1000_0000_1000_0000 | 0b0000_0000_0010_0000;
    //     std.log.info("AFTER TEST  0b10 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});
    // }

    std.log.info(" =============== ", .{});

    EPRegs.set_tx_status(tst_reg, StatusTxRx.disabled);
    std.log.info("AFTER TX DISBALED 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_tx_status(tst_reg, StatusTxRx.stall);
    std.log.info("AFTER TX STALL 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_tx_status(tst_reg, StatusTxRx.nak);
    std.log.info("AFTER TX NAK 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_tx_status(tst_reg, StatusTxRx.valid);
    std.log.info("AFTER TX VALID 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_tx_status(tst_reg, StatusTxRx.disabled);
    std.log.info("AFTER TX DISBALED 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_tx_status(tst_reg, StatusTxRx.valid);
    std.log.info("AFTER TX VALID 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_rx_status(tst_reg, StatusTxRx.valid);
    std.log.info("AFTER RX VALID 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_rx_status(tst_reg, StatusTxRx.disabled);
    std.log.info("AFTER RX DISBALED 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_rx_status(tst_reg, StatusTxRx.stall);
    std.log.info("AFTER RX STALL 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_rx_status(tst_reg, StatusTxRx.nak);
    std.log.info("AFTER RX NAK 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_rx_status(tst_reg, StatusTxRx.valid);
    std.log.info("AFTER RX VALID 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_rx_status(tst_reg, StatusTxRx.disabled);
    std.log.info("AFTER RX DISBALED 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_rx_status(tst_reg, StatusTxRx.valid);
    std.log.info("AFTER RX VALID 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_addr(tst_reg, 0b0000);
    std.log.info("AFTER SET ADDR 0000 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});

    EPRegs.set_addr(tst_reg, 0b1111);
    std.log.info("AFTER SET ADDR 1111 0x{X:0>4}", .{@truncate(u16, EPRegs.get(tst_reg))});
}
