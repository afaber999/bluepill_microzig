const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;

pub const peripherals = microzig.chip.peripherals;

const LED_PIN = hal.parse_pin(board.pin_map.LED);
const USB_DM = hal.parse_pin(board.pin_map.USB_DM);
const USB_DP = hal.parse_pin(board.pin_map.USB_DP);
//const USB_ENABLE = hal.parse_pin(board.pin_map.USB_ENABLE);

const usbh = @import("usb_helper.zig");
const test1 = @import("usb_test_1.zig");

pub const ISTR = usbh.ISTR;
pub const StatusTxRx = usbh.StatusTxRx;
pub const EPRegs = usbh.EPRegs;
pub const dump_regs = usbh.dump_regs;
pub const dump_bdt_mem = usbh.dump_bdt_mem;
pub const dump_ep_regs = usbh.dump_ep_regs;
pub const dump_pma_mem = usbh.dump_pma_mem;
pub const dump_pma_mem_range = usbh.dump_pma_mem_range;

pub const USB_PMASIZE = usbh.USB_PMASIZE;
pub const USB_PMRRECS = usbh.USB_PMRRECS;

pub const USB_NUMBDT = 8;
pub const USB_PMAADDR = 0x40006000;
pub const usb_pma = usbh.usb_pma;
pub const usb_bdt = usbh.usb_bdt;
pub const usb_epr = usbh.usb_epr;

const EP_CONFIG_TYPES = usbh.EP_CONFIG_TYPES;


fn usb_reset() void {

    // start on top
    var addr: u16 = USB_PMASIZE;

    for (0..usbh.EPCOUNT) |idx| {
        if (idx < usbh.EpConfigEntries.len) {
            const epc = usbh.EpConfigEntries[idx];

            addr = addr - epc.tx_max;
            usb_bdt[idx].tx_addr.data = addr;
            usb_bdt[idx].tx_cnt.data = 0;

            addr -= epc.rx_max;
            usb_bdt[idx].rx_addr.data = addr;

            // todo assert values in range
            if (epc.rx_max > 62) {
                usb_bdt[idx].rx_cnt.data = 0x8000 | (epc.rx_max / 32) << 10;
            } else {
                usb_bdt[idx].rx_cnt.data = (epc.rx_max / 2) << 10;
            }

            EPRegs.set_addr(epc.number, @intCast(u4, epc.number));
            const ep_tp = epc.ep_type;

            EPRegs.set_type(epc.number, ep_tp);
            EPRegs.set_rx_status(idx, StatusTxRx.valid);
            EPRegs.set_tx_status(idx, StatusTxRx.nak);
        } else {
            usb_bdt[idx].tx_addr.data = 0;
            usb_bdt[idx].tx_cnt.data = 0;
            usb_bdt[idx].rx_addr.data = 0;
            usb_bdt[idx].rx_cnt.data = 0;
            EPRegs.set_tx_status(idx, StatusTxRx.disabled);
            EPRegs.set_rx_status(idx, StatusTxRx.disabled);
        }
    }

    peripherals.USB.CNTR.modify(.{
        .CTRM = 0b1,
        .RESETM = 0b1,
        //.SUSPM = 0b1,
        //.ERRM = 0b1,
        //.SOFM = 0b1,
        // .WKUPM = 0b1,
    });

    //peripherals.USB.ISTR.write_raw(0);
    peripherals.USB.BTABLE.write_raw(0); // ALWAYS AT ZERO, DESIGN DECISION

    std.log.info("RESET COMPLETE DUMP REGS", .{});
    dump_regs();
    dump_ep_regs();
    dump_bdt_mem();

    peripherals.USB.DADDR.modify(.{ .EF = 0b1, .ADD = 0 });
}

// *****************************************************************************
// USB Endpoint Interrupt Handler
// *****************************************************************************
fn USB_EPHandler(status: u32) void {
    var ep_num = status & 0x0F;
    var ep = EPRegs.get(ep_num);
    std.log.info("USB_EPHandler({}) status: 0b{b:0>16} ep 0b{b:0>16}  FNR 0b{b:0>16}", .{ ep_num, @truncate(u16, status), @truncate(u16, ep), @truncate(u16, peripherals.USB.FNR.raw) });
    dump_bdt_mem();
    dump_pma_mem_range(0, 256);

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

                var xmit_len = std.math.min(@intCast(u16, usbh.MyDeviceDescription.len), wlength);

                // send MyDeviceDescription
                var w_idx = bdt.tx_addr.data / 2;

                // buffer to PMA, factor out this code
                for (0..xmit_len / 2) |idx| {
                    var val = @intCast(u16, usbh.MyDeviceDescription[2 * idx + 0]) + (std.math.shl(u16, @intCast(u16, usbh.MyDeviceDescription[2 * idx + 1]), 8));
                    usb_pma[w_idx + idx].data = val;
                }
                // debug

                dump_pma_mem_range(w_idx, xmit_len / 2);

                // update descriptor lengths and assign
                //bdt.rx_cnt.data &= (0b1111110000000000);
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
}

fn usb_poll() void {
    if (peripherals.USB.ISTR.read().RESET != 0) {
        // Reset
        std.log.info("POLL RESET () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        ISTR.clear_reset();
        usb_reset();
        return;
    }
    if (peripherals.USB.ISTR.read().CTR != 0) { //Handle data on EP
        std.log.info("POLL EPHandler () status: 0b{b:0>16} FNR 0b{b:0>16}", .{ @truncate(u16, peripherals.USB.ISTR.raw), @truncate(u16, peripherals.USB.FNR.raw) });
        USB_EPHandler(peripherals.USB.ISTR.raw);
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

pub fn main() void {
    std.log.info("STARTING USB_TEST TEST 0 B", .{});

    hal.gpio.set_output(LED_PIN, hal.gpio.OutputMode.pushpull, hal.gpio.OutputSpeed.output_10MHz);

    usbh.usb_init();
    usbh.usb_connect();
    std.log.info("USB ENABLED", .{});

    var loop_idx: u32 = 0;

    while (true) {
        if ((loop_idx % 3000) == 0) {
            std.log.info("loop_idx {}", .{loop_idx});
        }

        usb_poll();

        loop_idx += 1;
        hal.systick.delay_ms(1);
    }
}
