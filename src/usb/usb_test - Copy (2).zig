const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;
const peripherals = hal.peripherals;
const zusb = microzig.core.usb;

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

// fn get_next_pma(size: u16) u16 {
//     const USB_EPTABLE_SIZE = 0x20; // effective! buffer size of table, 4 entr CHECK CHECK CHECK
//     var result: u16 = USB_PMASIZE; // start on TOP

//     for (0..USB_NUMBDT) |idx| {
//         var bdt = usb_bdt[idx];

//         if ((bdt.tx.addr) and (bdt.tx.addr < result)) result = bdt.tx.addr;
//         if ((bdt.rx.addr) and (bdt.rx.addr < result)) result = bdt.rx.addr;
//     }

//     if (result < (USB_EPTABLE_SIZE + size)) {
//         return 0;
//     } else {
//         return result - size;
//     }
// }

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
    tx_data_start: u16,
    tx_data_buf: ?*const u8,
    tx_data_len: u16,
    rx_data_buf: ?*u8,
    rx_data_len: u16,
};

pub var EpConfigEntries = [_]EpConfigEntry{
    .{
        .number = 0,
        .ep_type = EP_CONFIG_TYPES.ep_control,
        .tx_max = 32,
        .rx_max = 32,
        .tx_data_buf = null,
        .tx_data_len = 0,
        .tx_data_start = 0,
        .rx_data_buf = null,
        .rx_data_len = 0,
    },
    .{
        .number = 1,
        .ep_type = EP_CONFIG_TYPES.ep_interrupt,
        .tx_max = 16,
        .rx_max = 16,
        .tx_data_buf = null,
        .tx_data_len = 0,
        .tx_data_start = 0,
        .rx_data_buf = null,
        .rx_data_len = 0,
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
            std.log.info("bdt.rx_cnt.data set to 0x{X:0>4}", .{@truncate(u16, bdt.rx_cnt.data)});

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
    dump_bdt_mem();
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
// *****************************************************************************
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
            //var ptrTest = std.mem.bytesAsValue(zusb.SetupPacket, usb_pma[start].data);
            //std.log.info("ptrTest  : 0x{x:0>4} 0x{x:0>4}", .{ @truncate(u16, ptrTest.request_type), @truncate(u16, ptrTest.request) });

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
                bdt.rx_cnt.data &= (0b1111110000000000);
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
    //       peripherals.USB.ISTR.read().EPR[EPn] &= 0x78f; //reset flag CTR_RX
    //     #ifdef SWO_USB_LOG
    //       pFloat = stradd (pFloat,"\r\nReceived in EP[");
    //       pFloat = itoa(EPn ,pFloat,10,0);
    //       pFloat = stradd (pFloat,"]: ");
    //     #endif
}

fn usb_poll() void {
    USB_LP_CAN1_RX0_IRQHandler();

    // while (true) {
    //     var istr = peripherals.USB.ISTR.read();
    //     var ep: usize = istr.EP_ID;
    //     _ = ep;

    //     if (istr.SUSP != 0) {
    //         ISTR.clear_susp();
    //         if (peripherals.USB.DADDR.read().ADD != 0) {
    //             std.log.info("SUSP !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
    //             peripherals.USB.DADDR.modify(.{ .ADD = 0 });
    //             ISTR.clear_susp();
    //             ISTR.clear_wkup();
    //         } else {
    //             std.log.info(" S", .{});
    //         }
    //     } else if (istr.RESET != 0) {
    //         // A "RESET" occurs when there are missing packets from the host. This is the at => {

    //         //,} the beginning of every connection,
    //         // until the host recognizes the device.
    //         std.log.info("RESET !!!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
    //         usb_reset();
    //         //ISTR.clear_reset();
    //     } else if (istr.CTR != 0) {

    //         // // Richtung des letzten Transfers. false bedeutet "IN" transfer (Device->Host), true bedeutet "IN" oder "OUT"
    //         // var dir = istr.DIR; // 0=TX, 1=RX/TX
    //         // // Die Nummer des EPnR-Registers, welches zu diesem Transfer gehört.
    //         // var ep_id = istr.EP_ID;
    //         // var ep_val = EPRegs.get(ep_id);
    //         // std.log.info("CTR !!! ISTR 0b{b:0>16} FNR 0b{b:0>16} DADDR 0x{x:0>4} DIR: {} ep: {} EPVAL: 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw), @truncate(u16, peripherals.USB.DADDR.raw), dir, ep_id, @truncate(u16, ep_val) });

    //         // // Stelle sicher, dass das EPnR Register existiert
    //         // // Lösche im EPnR-Register die RX/TX-Flags, falls sie gesetzt sind. Falls die Hardware zwischen Abfragen und Löschen
    //         // // eines der Bits setzt, wird dies nicht gelöscht und im nächsten Schleifendurchlauf behandelt.
    //         // EPRegs.set_rx_status(ep_id, StatusTxRx.disabled);
    //         // EPRegs.set_tx_status(ep_id, StatusTxRx.disabled);

    //         USB_EPHandler(peripherals.USB.ISTR.raw);
    //         ISTR.clear_ctr();
    //     } else if (istr.SOF != 0) {
    //         if (peripherals.USB.EP0R.read().STAT_TX == EPRegs.EP_TX_STALL) {
    //             // send NAK
    //             std.log.info("STALL -> NAK !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
    //             EPRegs.set_tx_status(0, StatusTxRx.nak);
    //         }
    //         ISTR.clear_sof();

    //         // todo SEND DATA

    //     } else if (istr.ESOF != 0) {
    //         //std.log.info("ESOF !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
    //         ISTR.clear_esof();
    //     } else if (istr.PMAOVR != 0) {
    //         std.log.info("PMAOVR !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
    //         ISTR.clear_pmaovr();
    //     } else if (istr.WKUP != 0) {
    //         std.log.info("WKUP !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
    //         ISTR.clear_wkup();
    //     } else if (istr.ERR != 0) {
    //         std.log.info("ERR !!! ISTR 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
    //         ISTR.clear_err();
    //     } else {
    //         peripherals.USB.ISTR.write_raw(0);
    //         // std.log.info("GOT ISTR RESET !!!! 0b{b:0>8}", .{peripherals.USB.ISTR.raw});
    //         break;
    //     }
    // }

    // // if (istr.RESET != 0) {
    // //    peripherals.USB.ISTR.modify(.{ .RESET = 0b0 });
    // //    std.log.info("GOT ISTR RESET !!!!", .{});

    // //    // BDT table is placed at bottom of PMA memory
    // //    //peripherals.USB.BTABLE.BT.BTABLE = map_mem_addr(&usb_bdt[0]);
    // //    peripherals.USB.BTABLE.write_raw( map_mem_addr(@ptrToInt(usb_bdt)));

    // //    for (0..USB_NUMBDT) |idx| {
    // //        usb_ep_deconfig(idx);
    // //    }
    // // }
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

    // for now alloc RX buffers on stack!
    var rx_buf0 = std.mem.zeroes([64]u8);
    var rx_buf1 = std.mem.zeroes([64]u8);
    //    var rx_buf2 = std.mem.zeroes([64]u8);

    EpConfigEntries[0].rx_data_buf = &rx_buf0[0];
    EpConfigEntries[0].rx_data_len = 64;

    EpConfigEntries[1].rx_data_buf = &rx_buf1[0];
    EpConfigEntries[1].rx_data_len = 64;

    // EpConfigEntries[2].rx_addr = rx_buf0;
    // EpConfigEntries[3].rx_cnt = 64;

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
    std.log.info("======= dump_pma_mem_range from {} len {} ", .{ start, length });

    for (start..start + length) |idx| {
        std.log.info(" PTR 0x{X:0>8} IDX 0x{X:0>4} VALUE 0x{X:0>4} ", .{ @ptrToInt(&usb_pma[idx].data), idx * 2, usb_pma[idx].data });
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

var device_address: u7 = 0;

pub fn USBLIB_Reset() void {

    // *********** WARNING **********
    // We DO NOT CHANGE BTABLE!! So we asume that buffer table start from address 0!!!

    // start on top
    var addr: u16 = USB_PMASIZE;

    for (0..EPCOUNT) |idx| {
        if (idx < EpConfigEntries.len) {
            EpConfigEntries[idx].tx_data_buf = null;
            EpConfigEntries[idx].tx_data_len = 0;
            EpConfigEntries[idx].tx_data_start = 0;

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
            EPRegs.set_tx_status(idx, StatusTxRx.disabled); // DISABLED OR NAK? check
            EPRegs.set_rx_status(idx, StatusTxRx.disabled);
        }
    }

    peripherals.USB.CNTR.modify(.{
        .CTRM = 0b1,
        .RESETM = 0b1,
        .SUSPM = 0b1,
        .ERRM = 0b1,
        //.SOFM = 0b1,
        // .WKUPM = 0b1,
    });

    peripherals.USB.ISTR.write_raw(0);
    peripherals.USB.BTABLE.write_raw(map_mem_addr(@ptrToInt(usb_bdt)));
    peripherals.USB.DADDR.modify(.{ .EF = 0b1, .ADD = 0 });

    std.log.info("RESET COMPLETE DUMP REGS", .{});
    dump_ep_regs();
    dump_bdt_mem();
}

// copy from PMA recv to EpConfigEntries[ep_num].rx_data_buf
pub fn USBLIB_Pma2EPBuf2(ep_num: u4) void {
    var bdt = usb_bdt[ep_num];
    var count: u16 = bdt.rx_cnt.data & 0x3FF;

    EpConfigEntries[ep_num].rx_data_len = count;

    var w_idx = bdt.rx_addr.data / 2;

    var dst_ptr = @ptrCast([*]u8, EpConfigEntries[ep_num].rx_data_buf);

    // buffer to PMA, factor out this code
    for (0..count / 2) |idx| {
        var val = usb_pma[w_idx + idx].data;

        var bl = @truncate(u8, val);
        var bh = @truncate(u8, std.math.shr(u16, val, 8));

        dst_ptr[2 * idx + 0] = bh;
        dst_ptr[2 * idx + 1] = bl;
    }

    dump_pma_mem_range(w_idx, count / 2);
}

// copy TX data from EpConfigEntries[ep_num].tx_data_buf tp PMA(tx) buffer
pub fn USBLIB_EPBuf2Pma(ep_num: u4) void {
    var start = EpConfigEntries[ep_num].tx_data_start;

    std.log.info("USBLIB_EPBuf2Pma({}) ptr: {any} start: {} len: {} ", .{ ep_num, EpConfigEntries[ep_num].tx_data_buf, EpConfigEntries[ep_num].tx_data_start, EpConfigEntries[ep_num].tx_data_len });

    var count: u16 = std.math.min(EpConfigEntries[ep_num].tx_data_len, EpConfigEntries[ep_num].tx_max);
    var src_ptr = @ptrCast([*]const u8, EpConfigEntries[ep_num].tx_data_buf);

    var bdt = usb_bdt[ep_num];
    var w_idx = bdt.tx_addr.data / 2;

    // buffer to PMA, factor out this code
    var bts = [2]u8{ 0, 0 };

    for (0..(2 * count + 1) / 2) |idx| {
        if (idx < count) {
            bts[idx % 2] = src_ptr[start];
            //std.log.info("COPY GOT {} 0x{x:0>2} ", .{idx , src_ptr[start]});
            start += 1;
        } else {
            bts[idx % 2] = 0;
        }
        if ((idx % 2) == 1) {
            const val = std.mem.bytesToValue(u16, &bts);
            //std.log.info("--> VAL 0x{x:0>2} bts = {any}", .{val , bts});
            usb_pma[w_idx + idx / 2].data = val;
        }
    }

    // update description table
    usb_bdt[ep_num].tx_cnt.data = count;

    // update remaining data
    EpConfigEntries[ep_num].tx_data_len -= count;
    EpConfigEntries[ep_num].tx_data_start = start;

    dump_pma_mem_range(w_idx, count / 2);
}

pub fn USBLIB_SendData(ep_num: u4, data: ?*const u8, length: u16) void {
    EpConfigEntries[ep_num].tx_data_len = length;
    EpConfigEntries[ep_num].tx_data_buf = data;
    EpConfigEntries[ep_num].tx_data_start = 0;

    if (length > 0) {
        USBLIB_EPBuf2Pma(ep_num);
    } else {
        usb_bdt[ep_num].tx_cnt.data = 0;
    }

    EPRegs.set_tx_status(ep_num, StatusTxRx.valid);

    std.log.info("end of USBLIB_SendData DUMP ISTR: 0b{b:0>16} EPReg(0): 0b{b:0>16} ", .{ peripherals.USB.ISTR.raw, EPRegs.get(0) });
    dump_ep_regs();
    dump_bdt_mem();
}

//comptime const ISTRType = @TypeOf(microzig.chip.types.USB.ISTR);

pub fn USBLIB_EPHandler(status: u32) void {
    var device_configured: u16 = 0;
    var device_status: u8 = 0;

    const ep_num: u4 = @intCast(u4, status & ISTR.USB_ISTR_EP_ID_MASK);
    var ep = EPRegs.get(ep_num);

    std.log.info("USBLIB_EPHandler({}) ISTR: 0b{b:0>16} EP:0b{b:0>16} FNR:0b{b:0>16}", .{ ep_num, @truncate(u16, status), @truncate(u16, ep), @truncate(u16, peripherals.USB.FNR.raw) });
    // dump_bdt_mem();
    //dump_pma_mem_range(0, 256);

    EPRegs.clear_ctr_rx(ep_num);
    EPRegs.clear_ctr_tx(ep_num);
    std.log.info("USBLIB_EPHandler({}) CLEARED RX/TX: EP 0b{b:0>16}", .{@truncate(u16, EPRegs.get(ep_num))});

    if ((ep & EPRegs.EP_CTR_RX_MASK) != 0) { //something received

        USBLIB_Pma2EPBuf2(ep_num);
        if (ep_num == 0) { //If control endpoint
            if ((ep & EPRegs.EP_STAT_SETUP_MASK) != 0) {
                var bdt = usb_bdt[ep_num];

                var start = bdt.rx_addr.data / 2;
                var length = (bdt.rx_cnt.data & 0x3FF);
                _ = length;

                var setup_packet = .{
                    .request_type = @truncate(u8, usb_pma[start].data),
                    .request = @truncate(u8, std.math.shr(u16, usb_pma[start].data, 8)),
                    .value = usb_pma[start + 1].data,
                    .index = usb_pma[start + 2].data,
                    .length = usb_pma[start + 3].data,
                };
                //std.log.info("setup_packet  : 0x{x:0>4} 0x{x:0>4}", .{ @truncate(u16, setup_packet.request_type), @truncate(u16, setup_packet.request) });

                switch (setup_packet.request) {
                    USB_REQUEST_SET_ADDRESS => {
                        USBLIB_SendData(0, null, 0);
                        device_address = @truncate(u7, setup_packet.value);
                        std.log.info("USB_REQUEST_SET_ADDRESS  : device_address 0x{x:0>2}", .{device_address});
                    },

                    USB_REQUEST_GET_DESCRIPTOR => {
                        // AF SHOULD LENGTH TAKEN INTO ACCOUNT
                        USBLIB_GetDescriptor(setup_packet.value);
                    },
                    USB_REQUEST_GET_STATUS => {
                        USBLIB_SendData(0, &device_status, 2);
                    },
                    USB_REQUEST_GET_CONFIGURATION => {
                        USBLIB_SendData(0, @ptrCast(?*u8, &device_configured), 1);
                    },

                    USB_REQUEST_SET_CONFIGURATION => {
                        device_configured = 1;
                        USBLIB_SendData(0, null, 0);
                    },

                    USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE => {
                        //TODO
                    },

                    USB_DEVICE_CDC_REQUEST_SET_LINE_CODING => {
                        //0x20
                        USBLIB_SendData(0, null, 0);
                    },

                    USB_DEVICE_CDC_REQUEST_GET_LINE_CODING => {
                        //0x21
                        // AF TODO  USBLIB_SendData(EPn, &lineCoding, sizeof(lineCoding));
                    },

                    USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE => {
                        //0x22
                        //LineState = SetupPacket->wValue;
                        //USBLIB_SendData(0, 0, 0);
                        //uUSBLIB_LineStateHandler(SetupPacket->wValue);
                    },
                    else => unreachable,
                }
            }
        } else { // Got data from another EP
            // Call user function
            uUSBLIB_DataReceivedHandler(@ptrCast(*const u8, EpConfigEntries[ep_num].tx_data_buf), EpConfigEntries[ep_num].tx_data_len);
        }

        //usb_epr[ep_num] = usb_epr[ep_num] & 0x78f;
        //EPRegs.set_rx_status(ep_num, StatusTxRx.valid);

        // AFX
        std.log.info("ENDOF RX ISTR: 0b{b:0>16} EPREGS: 0b{b:0>16}", .{ peripherals.USB.ISTR.raw, EPRegs.get(0) });
        dump_ep_regs();
        dump_bdt_mem();
    }

    if ((ep & EPRegs.EP_CTR_TX_MASK) != 0) { //something transmitted
        if (device_address != 0) {
            peripherals.USB.DADDR.modify(.{ .ADD = device_address, .EF = 0b1 });
            device_address = 0;
        }

        if (EpConfigEntries[ep_num].tx_data_len > 0) { //Have to transmit something?
            USBLIB_EPBuf2Pma(ep_num);
            EPRegs.set_tx_status(ep_num, StatusTxRx.valid);
        } else {
            // uUSBLIB_DataTransmitedHandler(ep_num, EpConfigEntries[ep_num]);
        }

        // AF TODO 0b 1000 0111 0000 1111
        // CLEAR CR_TX
        //usb_epr[ep_num] = usb_epr[ep_num] & 0x870f;

        std.log.info("ENDOF TX ISTR: 0b{b:0>16} EPREGS: 0b{b:0>16} ", .{ peripherals.USB.ISTR.raw, EPRegs.get(0) });
        // can receive again: TODO CHECK ONLY FOR SETUP PACKETS?
        EPRegs.set_rx_status(ep_num, StatusTxRx.valid);

        dump_ep_regs();
        dump_bdt_mem();
    }
}

pub fn USB_LP_CAN1_RX0_IRQHandler() void {
    if (peripherals.USB.ISTR.read().RESET != 0) {
        // Reset
        std.log.info("USB_LP_CAN1_RX0_IRQHandler RESET () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_reset();
        USBLIB_Reset();
        return;
    }
    if (peripherals.USB.ISTR.read().CTR != 0) { //Handle data on EP
        std.log.info("USB_LP_CAN1_RX0_IRQHandler EPHandler () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        USBLIB_EPHandler(peripherals.USB.ISTR.raw);
        ISTR.clear_ctr();
        return;
    }
    if (peripherals.USB.ISTR.read().PMAOVR != 0) {
        //std.log.info("USB_LP_CAN1_RX0_IRQHandler PMAVR () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_pmaovr();
        // Handle PMAOVR status
        return;
    }
    if (peripherals.USB.ISTR.read().SUSP != 0) {
        //std.log.info("USB_LP_CAN1_RX0_IRQHandler SUSP () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_susp();
        if (peripherals.USB.DADDR.read().ADD != 0) {
            peripherals.USB.DADDR.modify(.{ .ADD = 0 });
            // force reset
            peripherals.USB.CNTR.modify(.{ .FRES = 0b1 });
        }
        return;
    }
    if (peripherals.USB.ISTR.read().ERR != 0) {
        std.log.info("USB_LP_CAN1_RX0_IRQHandler ERR () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_err();
        // Handle Error
        return;
    }
    if (peripherals.USB.ISTR.read().WKUP != 0) {
        //std.log.info("IRQ WKUP () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_wkup();
        // Handle Wakeup
        return;
    }
    if (peripherals.USB.ISTR.read().SOF != 0) {
        //std.log.info("USB_LP_CAN1_RX0_IRQHandler SOF () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_sof();
        // Handle SOF
        return;
    }
    if (peripherals.USB.ISTR.read().ESOF != 0) {
        //std.log.info("USB_LP_CAN1_RX0_IRQHandler ESOF () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_esof();
        // Handle ESOF
        return;
    }
    peripherals.USB.ISTR.write_raw(0);
}

// USB Standard Request Codes
const USB_REQUEST_GET_STATUS = 0;
const USB_REQUEST_CLEAR_FEATURE = 1;
const USB_REQUEST_SET_FEATURE = 3;
const USB_REQUEST_SET_ADDRESS = 5;
const USB_REQUEST_GET_DESCRIPTOR = 6;
const USB_REQUEST_SET_DESCRIPTOR = 7;
const USB_REQUEST_GET_CONFIGURATION = 8;
const USB_REQUEST_SET_CONFIGURATION = 9;
const USB_REQUEST_GET_INTERFACE = 10;
const USB_REQUEST_SET_INTERFACE = 11;
const USB_REQUEST_SYNC_FRAME = 12;

// USB Descriptor Types
const USB_DEVICE_DESC_TYPE = 1;
const USB_CFG_DESC_TYPE = 2;
const USB_STR_DESC_TYPE = 3;
const USB_IFACE_DESC_TYPE = 4;
const USB_EP_DESC_TYPE = 5;
const USB_DEVICE_QR_DESC_TYPE = 6;
const USB_OSPEED_CFG_DESC_TYPE = 7;
const USB_IFACE_PWR_DESC_TYPE = 8;

// USB Device Classes
const USB_RESERVED = 0x00;
const USB_AUDIO = 0x01;
const USB_COMM = 0x02;
const USB_HID = 0x03;
const USB_MONITOR = 0x04;
const USB_PHYSIC = 0x05;
const USB_POWER = 0x06;
const USB_PRINTER = 0x07;
const USB_STORAGE = 0x08;
const USB_HUB = 0x09;
const USB_VENDOR_SPEC = 0xFF;
// Interface Class SubClass Codes
const USB_ACM_COMM = 0x02;

const CDC_DATA_IFACE = 0x0A;
const CS_INTERFACE = 0x24;
const CS_ENDPOINT = 0x25;

const LANG_US = 0x0409;

pub fn USBLIB_GetDescriptor(desc_type_and_index: u16) void {
    var bts = std.mem.toBytes(desc_type_and_index);
    const desc_index = bts[0]; // high part = index
    const desc_type = bts[1]; // low part is type

    std.log.info("USBLIB_GetDescriptor( type = {} index = {} ) ", .{ desc_type, desc_index });

    switch (desc_type) {
        USB_DEVICE_DESC_TYPE => USBLIB_SendData(0, @ptrCast(?*const u8, &USB_DEVICE_DESC), @sizeOf(@TypeOf(USB_DEVICE_DESC))),
        USB_CFG_DESC_TYPE => USBLIB_SendData(0, @ptrCast(?*const u8, &USBD_CDC_CFG_DESCRIPTOR), @sizeOf(@TypeOf(USBD_CDC_CFG_DESCRIPTOR))),
        USB_STR_DESC_TYPE => {
            if (desc_index < USBStringTable.len) {
                var data = USBStringTable[desc_index];
                USBLIB_SendData(0, @ptrCast(?*const u8, data), @intCast(u16, data.len));
            } else {
                std.log.err("USBLIB_GetDescriptor : unknown index! (index = {} ) ", .{desc_index});
                USBLIB_SendData(0, null, 0);
            }
        },
        else => {
            std.log.err("USBLIB_GetDescriptor : unknown type! (type = {} ) ", .{desc_type});
            USBLIB_SendData(0, null, 0);
        },
    }
}

pub fn CreateUsbLang(lang_id: u16) []const u8 {
    const result = [_]u8{ 0x04, 0x03, @truncate(u8, lang_id), lang_id >> 8 };
    return &result;
}

pub fn CreateUsbString(in_str: []const u8) []const u8 {
    const result_len = in_str.len * 2 + 2;
    comptime var result = std.mem.zeroes([result_len]u8);
    result[0] = result_len;
    result[1] = 0x03;

    inline for (0..in_str.len) |i| {
        result[2 * i + 2] = in_str[i];
        result[2 * i + 3] = 0;
    }
    return &result;
}

pub const USB_CDC_CONFIG_DESC_SIZ = 67;
pub const CDC_CMD_EP = 0x81; // EP2 for CDC commands
pub const CDC_IN_EP = 0x82; // EP1 for data IN
pub const CDC_OUT_EP = 0x02; // EP; for data OUT

pub const USB_DEVICE_CDC_REQUEST_SEND_ENCAPSULATED_COMMAND = (0x00); // The CDC class request code for SEND_ENCAPSULATED_COMMAND.
pub const USB_DEVICE_CDC_REQUEST_GET_ENCAPSULATED_RESPONSE = (0x01); // The CDC class request code for GET_ENCAPSULATED_RESPONSE.
pub const USB_DEVICE_CDC_REQUEST_SET_COMM_FEATURE = (0x02); // The CDC class request code for SET_COMM_FEATURE.
pub const USB_DEVICE_CDC_REQUEST_GET_COMM_FEATURE = (0x03); // The CDC class request code for GET_COMM_FEATURE.
pub const USB_DEVICE_CDC_REQUEST_CLEAR_COMM_FEATURE = (0x04); // The CDC class request code for CLEAR_COMM_FEATURE.
pub const USB_DEVICE_CDC_REQUEST_SET_AUX_LINE_STATE = (0x10); // The CDC class request code for SET_AUX_LINE_STATE.
pub const USB_DEVICE_CDC_REQUEST_SET_HOOK_STATE = (0x11); // The CDC class request code for SET_HOOK_STATE.
pub const USB_DEVICE_CDC_REQUEST_PULSE_SETUP = (0x12); // The CDC class request code for PULSE_SETUP.
pub const USB_DEVICE_CDC_REQUEST_SEND_PULSE = (0x13); // The CDC class request code for SEND_PULSE.
pub const USB_DEVICE_CDC_REQUEST_SET_PULSE_TIME = (0x14); // The CDC class request code for SET_PULSE_TIME.
pub const USB_DEVICE_CDC_REQUEST_RING_AUX_JACK = (0x15); // The CDC class request code for RING_AUX_JACK.
pub const USB_DEVICE_CDC_REQUEST_SET_LINE_CODING = (0x20); // The CDC class request code for SET_LINE_CODING.
pub const USB_DEVICE_CDC_REQUEST_GET_LINE_CODING = (0x21); // The CDC class request code for GET_LINE_CODING.
pub const USB_DEVICE_CDC_REQUEST_SET_CONTROL_LINE_STATE = (0x22); // The CDC class request code for SET_CONTROL_LINE_STATE.
pub const USB_DEVICE_CDC_REQUEST_SEND_BREAK = (0x23); // The CDC class request code for SEND_BREAK.
pub const USB_DEVICE_CDC_REQUEST_SET_RINGER_PARAMS = (0x30); // The CDC class request code for SET_RINGER_PARAMS.
pub const USB_DEVICE_CDC_REQUEST_GET_RINGER_PARAMS = (0x31); // The CDC class request code for GET_RINGER_PARAMS.
pub const USB_DEVICE_CDC_REQUEST_SET_OPERATION_PARAM = (0x32); // The CDC class request code for SET_OPERATION_PARAM.
pub const USB_DEVICE_CDC_REQUEST_GET_OPERATION_PARAM = (0x33); // The CDC class request code for GET_OPERATION_PARAM.
pub const USB_DEVICE_CDC_REQUEST_SET_LINE_PARAMS = (0x34); // The CDC class request code for SET_LINE_PARAMS.
pub const USB_DEVICE_CDC_REQUEST_GET_LINE_PARAMS = (0x35); // The CDC class request code for GET_LINE_PARAMS.
pub const USB_DEVICE_CDC_REQUEST_DIAL_DIGITS = (0x36); // The CDC class request code for DIAL_DIGITS.
pub const USB_DEVICE_CDC_REQUEST_SET_UNIT_PARAMETER = (0x37); // The CDC class request code for SET_UNIT_PARAMETER.
pub const USB_DEVICE_CDC_REQUEST_GET_UNIT_PARAMETER = (0x38); // The CDC class request code for GET_UNIT_PARAMETER.
pub const USB_DEVICE_CDC_REQUEST_CLEAR_UNIT_PARAMETER = (0x39); // The CDC class request code for CLEAR_UNIT_PARAMETER.
pub const USB_DEVICE_CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS = (0x40); // The CDC class request code for SET_ETHERNET_MULTICAST_FILTERS.
pub const USB_DEVICE_CDC_REQUEST_SET_ETHERNET_POW_PATTER_FILTER = (0x41); // The CDC class request code for SET_ETHERNET_POW_PATTER_FILTER.
pub const USB_DEVICE_CDC_REQUEST_GET_ETHERNET_POW_PATTER_FILTER = (0x42); // The CDC class request code for GET_ETHERNET_POW_PATTER_FILTER.
pub const USB_DEVICE_CDC_REQUEST_SET_ETHERNET_PACKET_FILTER = (0x43); // The CDC class request code for SET_ETHERNET_PACKET_FILTER.
pub const USB_DEVICE_CDC_REQUEST_GET_ETHERNET_STATISTIC = (0x44); // The CDC class request code for GET_ETHERNET_STATISTIC.
pub const USB_DEVICE_CDC_REQUEST_SET_ATM_DATA_FORMAT = (0x50); // The CDC class request code for SET_ATM_DATA_FORMAT.
pub const USB_DEVICE_CDC_REQUEST_GET_ATM_DEVICE_STATISTICS = (0x51); // The CDC class request code for GET_ATM_DEVICE_STATISTICS.
pub const USB_DEVICE_CDC_REQUEST_SET_ATM_DEFAULT_VC = (0x52); // The CDC class request code for SET_ATM_DEFAULT_VC.
pub const USB_DEVICE_CDC_REQUEST_GET_ATM_VC_STATISTICS = (0x53); // The CDC class request code for GET_ATM_VC_STATISTICS.
pub const USB_DEVICE_CDC_REQUEST_MDLM_SPECIFIC_REQUESTS_MASK = (0x7F); // The CDC class request code for MDLM_SPECIFIC_REQUESTS_MASK.

pub fn USBLIB_Transmit(data: *const u8, length: u16) void {
    //    if (LineState.L) {
    USBLIB_SendData(2, data, length);
    //    }
}

pub const USBStringTable = [_][]const u8{
    CreateUsbLang(LANG_US), // 0 = LANG
    CreateUsbString("ziglang.org"), // 1 = vendor ID string
    CreateUsbString("ZIG test"), // 2 = product ID string
    CreateUsbString("0123-4567-89"), // 3 = serial string
    CreateUsbString("CDC Device"), // 4 = CDC device string
    CreateUsbString("CDC Data"), // 4 = CDC data string
};

pub const DEVICE_VENDOR_ID = 0x25AE;
pub const DEVICE_PRODUCT_ID = 0x24AB;

pub const USB_DEVICE_DESC = [_]u8{
    18, //    bLength
    USB_DEVICE_DESC_TYPE, //    bDescriptorType
    0x00, //    bcdUSB
    0x02, //    bcdUSB
    USB_COMM, //    bDeviceClass
    0, //    bDeviceSubClass
    0, //    bDeviceProtocol
    8, //    bMaxPacketSize0
    (DEVICE_VENDOR_ID & 0xFF), //    idVendor
    (DEVICE_VENDOR_ID >> 8), //    idVendor
    (DEVICE_PRODUCT_ID & 0xFF), //    idProduct
    (DEVICE_PRODUCT_ID >> 8), //    idProduct
    0x00, //    bcdDevice
    0x01, //    bcdDevice
    1, //    iManufacturer
    2, //    iProduct
    3, //    iSerialNumber
    1, //    bNumConfigurations
};

pub const USBD_CDC_CFG_DESCRIPTOR = [_]u8{
    // Configuration Descriptor
    0x09, // bLength: Configuration Descriptor size
    0x02, // bDescriptorType: Configuration
    67, // wTotalLength:no of returned bytes
    0x00,
    0x02, // bNumInterfaces: 2 interface
    0x01, // bConfigurationValue: Configuration value
    0x00, // iConfiguration: Index of string descriptor describing the configuration
    0x80, // bmAttributes - Bus powered
    0x32, // MaxPower 100 mA

    // Interface Descriptor
    0x09, // bLength: Interface Descriptor size
    0x04, // bDescriptorType: Interface
    0x00, // bInterfaceNumber: Number of Interface
    0x00, // bAlternateSetting: Alternate setting
    0x01, // bNumEndpoints: One endpoints used
    0x02, // bInterfaceClass: Communication Interface Class
    0x02, // bInterfaceSubClass: Abstract Control Model
    0x01, // bInterfaceProtocol: Common AT commands
    0x00, // iInterface:

    // Header Functional Descriptor
    0x05, // bLength: Endpoint Descriptor size
    0x24, // bDescriptorType: CS_INTERFACE
    0x00, // bDescriptorSubtype: Header Func Desc
    0x10, // bcdCDC: spec release number
    0x01,

    // Call Management Functional Descriptor
    0x05, // bFunctionLength
    0x24, // bDescriptorType: CS_INTERFACE
    0x01, // bDescriptorSubtype: Call Management Func Desc
    0x00, // bmCapabilities: D0+D1
    0x01, // bDataInterface: 1

    // ACM Functional Descriptor
    0x04, // bFunctionLength
    0x24, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: Abstract Control Management desc
    0x02, // bmCapabilities

    // Union Functional Descriptor
    0x05, // bFunctionLength
    0x24, // bDescriptorType: CS_INTERFACE
    0x06, // bDescriptorSubtype: Union func desc
    0x00, // bMasterInterface: Communication class interface
    0x01, // bSlaveInterface0: Data Class Interface

    // Endpoint 2 Descriptor
    0x07, // bLength: Endpoint Descriptor size
    0x05, // bDescriptorType: Endpoint
    0x81, // bEndpointAddress IN1
    0x03, // bmAttributes: Interrupt
    0x08, // wMaxPacketSize LO:
    0x00, // wMaxPacketSize HI:
    0x10, // bInterval:
    // ---------------------------------------------------------------------------

    // Data class interface descriptor
    0x09, // bLength: Endpoint Descriptor size
    0x04, // bDescriptorType:
    0x01, // bInterfaceNumber: Number of Interface
    0x00, // bAlternateSetting: Alternate setting
    0x02, // bNumEndpoints: Two endpoints used
    0x0A, // bInterfaceClass: CDC
    0x02, // bInterfaceSubClass:
    0x00, // bInterfaceProtocol:
    0x00, // iInterface:

    // Endpoint IN2 Descriptor
    0x07, // bLength: Endpoint Descriptor size
    0x05, // bDescriptorType: Endpoint
    0x82, // bEndpointAddress IN2
    0x02, // bmAttributes: Bulk
    64, // wMaxPacketSize:
    0x00,
    0x00, // bInterval: ignore for Bulk transfer

    // Endpoint OUT3 Descriptor
    0x07, // bLength: Endpoint Descriptor size
    0x05, // bDescriptorType: Endpoint
    0x03, // bEndpointAddress
    0x02, // bmAttributes: Bulk
    64, // wMaxPacketSize:
    0,
    0x00, // bInterval: ignore for Bulk transfer
};

pub fn uUSBLIB_DataReceivedHandler(data: *const u8, length: u16) void {
    USBLIB_Transmit(data, length);
}
