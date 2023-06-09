const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;
const peripherals = hal.peripherals;

const LED_PIN = hal.parse_pin(board.pin_map.LED);

pub const std_options = struct {
    pub const log_level = .debug;
    pub const logFn = hal.usart.log;
};

const USBM_PIN = hal.parse_pin("PA11");
const USBD_PIN = hal.parse_pin("PA12");

var num_interrupts: u32 = 0;

//
//
//
//
//
//
//

//extern var stm32_usb_buf_start: anyopaque;

const USB_PMASIZE = 0x200; // size in BYTES! -> 256 u16
const USB_PMRRECS = USB_PMASIZE / 2; // number of PMA record 256 u16

const USB_NUMBDT = 8;
const USB_PMAADDR = 0x40006000;
pub const usb_pma = @intToPtr([*]volatile PmaElem, USB_PMAADDR);
pub const usb_bdt = @intToPtr([*]volatile BufferDescriptorTableEntry, USB_PMAADDR); // place BufferDescriptorTableEntry[USB_NUMBDT] at start of PMA memory
pub const usb_epr = @ptrCast([*]volatile u32, peripherals.USB);

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
    pub const USB_ISTR_ESOF_MASK = 0x1 << USB_ISTR_ESOF_POS;
    pub const USB_ISTR_ESOF = USB_ISTR_ESOF_MASK;
    pub const USB_ISTR_SOF_POS = 9;
    pub const USB_ISTR_SOF_MASK = 0x1 << USB_ISTR_SOF_POS;
    pub const USB_ISTR_SOF = USB_ISTR_SOF_MASK;
    pub const USB_ISTR_RESET_POS = 10;
    pub const USB_ISTR_RESET_MASK = 0x1 << USB_ISTR_RESET_POS;
    pub const USB_ISTR_RESET = USB_ISTR_RESET_MASK;
    pub const USB_ISTR_SUSP_POS = 11;
    pub const USB_ISTR_SUSP_MASK = 0b1 << USB_ISTR_SUSP_POS;
    pub const USB_ISTR_SUSP = USB_ISTR_SUSP_MASK;
    pub const USB_ISTR_WKUP_POS = 12;
    pub const USB_ISTR_WKUP_MASK = 0b1 << USB_ISTR_WKUP_POS;
    pub const USB_ISTR_WKUP = USB_ISTR_WKUP_MASK;
    pub const USB_ISTR_ERR_POS = 13;
    pub const USB_ISTR_ERR_MASK = 0b1 << USB_ISTR_ERR_POS;
    pub const USB_ISTR_ERR = USB_ISTR_ERR_MASK;
    pub const USB_ISTR_PMAOVR_POS = 14;
    pub const USB_ISTR_PMAOVR_MASK = 0b1 << USB_ISTR_PMAOVR_POS;
    pub const USB_ISTR_PMAOVR = USB_ISTR_PMAOVR_MASK;

    // get the register value
    pub fn get() u32 {
        return peripherals.USB.ISTR.raw;
    }

    pub fn clear_pmaovr() void {
        peripherals.USB.ISTR.raw = (0xFFFFFFFF & !USB_ISTR_PMAOVR_MASK);
    }

    pub fn clear_err() void {
        peripherals.USB.ISTR.raw = (0xFFFFFFFF & !USB_ISTR_ERR_MASK);
    }

    pub fn clear_wkup() void {
        peripherals.USB.ISTR.raw = (0xFFFFFFFF & !USB_ISTR_WKUP_MASK);
    }

    pub fn clear_susp() void {
        peripherals.USB.ISTR.raw = (0xFFFFFFFF & !USB_ISTR_SUSP_MASK);
    }

    pub fn clear_reset() void {
        peripherals.USB.ISTR.raw = (0xFFFFFFFF & !USB_ISTR_RESET_MASK);
    }

    pub fn clear_sof() void {
        peripherals.USB.ISTR.raw = (0xFFFFFFFF & !USB_ISTR_SOF_MASK);
    }

    pub fn clear_esof() void {
        peripherals.USB.ISTR.raw = (0xFFFFFFFF & !USB_ISTR_ESOF_MASK);
    }
};

pub const EPRegs = struct {

    // get the register value
    pub fn get(ep_num: usize) u32 {
        // EP0R
        return usb_epr[ep_num];
    }

    const EP_CTR_RX = 15;
    const EP_CTR_RX_MASK = (0b1 << EP_CTR_RX);

    const EP_DTOG_RX = 14;
    const EP_DTOG_RX_MASK = (0b1 << EP_DTOG_RX);

    const EP_STAT_RX = 12;
    const EP_STAT_RX_MASK = (0b11 << EP_STAT_RX);

    const EP_STAT_SETUP = 11;
    const EP_STAT_SETUP_MASK = (0b1 << EP_STAT_SETUP);

    const EP_FIELD = 9;
    const EP_FIELD_MASK = (0b11 << EP_FIELD_MASK);

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

    const EP_TX_DIS = 0x00000000; // EndPoint TX DISabled
    const EP_TX_STALL = 0x00000010; // EndPoint TX STALLed
    const EP_TX_NAK = 0x00000020; // EndPoint TX NAKed
    const EP_TX_VALID = 0x00000030; // EndPoint TX VALID
    const EPTX_DTOG1 = 0x00000010; // EndPoint TX Data TOGgle bit1
    const EPTX_DTOG2 = 0x00000020; // EndPoint TX Data TOGgle bit2

    const EP_RX_DIS = 0x00000000; // EndPoint RX DISabled
    const EP_RX_STALL = 0x00001000; // EndPoint RX STALLed
    const EP_RX_NAK = 0x00002000; // EndPoint RX NAKed
    const EP_RX_VALID = 0x00003000; // EndPoint RX VALID
    const EPRX_DTOG1 = 0x00001000; // EndPoint RX Data TOGgle bit1
    const EPRX_DTOG2 = 0x00002000; // EndPoint RX Data TOGgle bit1

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
    pub fn setEPnR(ep_num: u8, mask: u32, data: u32, old: u32) void {
        var wr0: u32 = rc_w0 & (~mask | data);
        var wr1: u32 = (mask & toggle) & (old ^ data);
        var wr2: u32 = rw & ((old & ~mask) | data);

        usb_epr[ep_num].raw = wr0 | wr1 | wr2;
    }

    pub fn reset(ep_num: usize) void {
        const mask = EP_STAT_RX_MASK | EP_FIELD_MASK | EP_EA_MASK | EP_KIND_MASK | EP_STAT_TX_MASK |
            EP_RX_NAK | EP_TX_NAK |
            0;

        setEPnR(ep_num, mask, get(ep_num));
    }

    // static_cast<uint16_t> (USB_EP_RX_NAK | USB_EP_TX_NAK
    // | (static_cast<uint16_t> (m_type) << USB_EP_T_FIELD_Pos)
    // | (uint16_t { m_address } << USB_EPADDR_FIELD_Pos)));

    // pub fn tx_stall(ep_num: usize) void {
    //    usb_epr[ep_num] = !(1 << CTR_RX);
    // }

    // // Clear the CTR_RX bit (15) in an endpoint register
    // pub fn clr_ctrx(ep_num: usize) void {
    //    usb_epr[ep_num] = !(1 << CTR_RX);
    // }

    //const USB_CLEAR_RX_EP_CTR = USB;
    //) (USB_SET_ENDPOINT((USBx),(bEpNum),USB_GET_ENDPOINT((USBx),(bEpNum)) & 0x7FFF & USB_EPREG_MASK))

    // Set EP_KIND bit in an endpoint register
    // const USB_SET_EP_KIND = USB;
    //) (USB_SET_ENDPOINT((USBx),(bEpNum), \
    // 			(USB_EP_CTR_RX | USB_EP_CTR_TX | ((USB_GET_ENDPOINT((USBx),(bEpNum)) | USB_EP_KIND) & USB_EPREG_MASK))))
    pub fn set_kind(ep_num: usize) void {
        _ = ep_num;
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

    peripherals.USB.CNTR.modify(.{
        .FRES = 0b0,
    });

    // tstartup = 1us
    hal.systick.delay_ms(1);

    peripherals.USB.CNTR.modify(.{
        .CTRM = 0b1,
        .RESETM = 0b1,
        .ERRM = 0b1,
        .SOFM = 0b1,
        .SUSPM = 0b1,
        .WKUPM = 0b1,
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

fn usb_ep_deconfig(ep_num: usize) void {
    var bdt = usb_bdt[ep_num];
    bdt.tx_addr.data = 0;
    bdt.rx_cnt.data = 0;
    bdt.tx_addr.data = 0;
    bdt.tx_cnt.data = 0;

    EPRegs.reset(ep_num);
}

fn usb_init_device() void {

    // This function is called when the host resets the device ("RESET"), possibly also when booting.
    // It should set the hardware and software to a defined initial state so that the driver on the host side of a
    // clean state.

    // set address of Buffer Description Table (PMA) address
    peripherals.USB.BTABLE.raw = map_mem_addr(@ptrToInt(usb_bdt));

    // new address will be zero
    peripherals.USB.DADDR.modify(.{ .EF = 0b1, .ADD = 0 });

    // Initalize all endpoints
    // 	for (EPBuffer* ep : m_epBuffers)
    // 		if (ep)
    // 			ep->onReset ();
    // }

}

fn usb_poll() void {
    while (true) {
        var istr = peripherals.USB.ISTR.read();
        var ep: usize = istr.EP_ID;
        _ = ep;

        if (istr.RESET != 0) {
            // A "RESET" occurs when there are missing packets from the host. This is the case at the beginning of every connection,
            // until the host recognizes the device.
            std.log.info("GOT ISTR RESET !!!!", .{});

            ISTR.clear_reset();

            for (0..USB_NUMBDT) |idx| {
                usb_ep_deconfig(idx);
            }
        } else if (istr.RESET != 0) {
            // get direction
            var dir: bool = (istr.DIR == 0b1);
            std.log.info("GOT CTR !!!!", .{});
            _ = dir;
            ISTR.clear_ctr();
        } else {
            break;
        }
    }

    // if (istr.RESET != 0) {
    //    peripherals.USB.ISTR.modify(.{ .RESET = 0b0 });
    //    std.log.info("GOT ISTR RESET !!!!", .{});

    //    // BDT table is placed at bottom of PMA memory
    //    //peripherals.USB.BTABLE.BT.BTABLE = map_mem_addr(&usb_bdt[0]);
    //    peripherals.USB.BTABLE.raw = map_mem_addr(@ptrToInt(usb_bdt));

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
    uart.apply(9600);

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

    const mpa = map_mem_addr(@ptrToInt(&usb_bdt[1]));
    const epr2 = usb_epr;

    std.log.info("TEST: {} 0x{X:0>8} 0x{X:0>8} ", .{ mpa, @ptrToInt(usb_epr), @ptrToInt(epr2) });

    std.log.info("TEST111: {} 0x{X:0>8} 0x{X:0>8} ", .{ mpa, peripherals.USB.EP0R.raw, EPRegs.get(7) });
    peripherals.USB.EP7R.raw = 0xFFFFFFFF;
    std.log.info("TEST222: {} 0x{X:0>8} 0x{X:0>8} ", .{ mpa, peripherals.USB.EP0R.raw, EPRegs.get(7) });

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
