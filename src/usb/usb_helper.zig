const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;
const peripherals = hal.peripherals;
const zusb = microzig.core.usb;


const USBM_PIN = hal.parse_pin("PA11");
const USBD_PIN = hal.parse_pin("PA12");

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

    pub const EP_CTR_RX = 15;
    pub const EP_CTR_RX_MASK = (0b1 << EP_CTR_RX);

    pub const EP_DTOG_RX = 14;
    pub const EP_DTOG_RX_MASK = (0b1 << EP_DTOG_RX);

    pub const EP_STAT_RX = 12;
    pub const EP_STAT_RX_MASK = (0b11 << EP_STAT_RX);

    pub const EP_STAT_SETUP = 11;
    pub const EP_STAT_SETUP_MASK = (0b1 << EP_STAT_SETUP);

    pub const EP_FIELD = 9;
    pub const EP_FIELD_MASK = (0b11 << EP_FIELD);

    pub const EP_KIND = 8;
    pub const EP_KIND_MASK = (0b1 << EP_KIND);

    pub const EP_CTR_TX = 7;
    pub const EP_CTR_TX_MASK = (0b1 << EP_CTR_TX);

    pub const EP_DTOG_TX = 6;
    pub const EP_DTOG_TX_MASK = (0b1 << EP_DTOG_TX);

    pub const EP_STAT_TX = 4;
    pub const EP_STAT_TX_MASK = (0b11 << EP_STAT_TX);

    pub const EP_EA = 0;
    pub const EP_EA_MASK = (0b1111 << EP_EA);

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


    pub fn and_mask( ep_num: usize, mask : u16) void {
        var val = usb_epr[ep_num] & mask;
        usb_epr[ep_num] = val;
    }

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
    std.log.info("FNR    0x{X:0>4}", .{@truncate(u16, peripherals.USB.FNR.raw)});
    std.log.info("BTABLE 0x{X:0>4}", .{@truncate(u16, peripherals.USB.BTABLE.raw)});
    std.log.info("CNTR   0x{X:0>4}", .{@truncate(u16, peripherals.USB.CNTR.raw)});
    std.log.info("ISTR   0x{X:0>4}", .{@truncate(u16, peripherals.USB.ISTR.raw)});
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

pub const USB_PMASIZE = 0x200; // size in BYTES! -> 256 u16
pub const USB_PMRRECS = USB_PMASIZE / 2; // number of PMA record 256 u16

pub const USB_NUMBDT = 8;
pub const USB_PMAADDR = 0x40006000;
pub const usb_pma = @intToPtr([*]volatile PmaElem, USB_PMAADDR);
pub const usb_bdt = @intToPtr([*]volatile BufferDescriptorTableEntry, USB_PMAADDR); // place BufferDescriptorTableEntry[USB_NUMBDT] at start of PMA memory
pub const usb_epr = @ptrCast([*]volatile u32, peripherals.USB);

pub const EPCOUNT = 8;

pub const EP_CONFIG_TYPES = enum(u2) {
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

pub const MyDeviceDescription = [18]u8{
    0x12, // bLength
    0x01, // bDescriptorType: Device
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


pub fn usb_init() void {

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

pub fn usb_connect() void {

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
