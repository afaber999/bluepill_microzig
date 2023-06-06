const std = @import("std");
const microzig = @import("microzig");
const board = microzig.board;
const hal = microzig.hal;
const dbg = hal.semihosting;

pub const peripherals = microzig.chip.peripherals;

const LED_PIN = hal.parse_pin(board.pin_map.LED);
const USB_DM = hal.parse_pin(board.pin_map.USB_DM);
const USB_DP = hal.parse_pin(board.pin_map.USB_DP);
//const USB_ENABLE = hal.parse_pin(board.pin_map.USB_ENABLE);

pub fn usb_enable(enable: bool) void {
    if (enable) {

        //hal.gpio.set_output(USB_DP, hal.gpio.OutputMode.alternate_opendrain, hal.gpio.OutputSpeed.output_50MHz);
        //hal.gpio.set_output(USB_DM, hal.gpio.OutputMode.alternate_opendrain, hal.gpio.OutputSpeed.output_50MHz);

        // USB pullup (for bluepill devices with incorrect pull up resistor)
        hal.gpio.set_output(USB_DP, hal.gpio.OutputMode.opendrain, hal.gpio.OutputSpeed.output_50MHz);
        hal.gpio.write(USB_DP, hal.gpio.State.high);
        hal.systick.delay_ms(10);

        // ENABLE USB
        peripherals.RCC.APB1ENR.modify(.{ .USBEN = 0b1 });
        peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b1 });
        peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b0 });

        peripherals.USB.CNTR.modify(.{
            .CTRM = 0b1,
            .RESETM = 0b1,
            .ERRM = 0b1,
            .SUSPM = 0b1,
            .WKUPM = 0b1,
            .PDWN = 0b1,
        });
    } else {
        peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b1 });
        peripherals.RCC.APB1ENR.modify(.{ .USBEN = 0b0 });
    }
}

pub fn usb_init() void {
    usb_enable(true);

    peripherals.USB.ISTR.raw = 0x00000000;
    peripherals.USB.BTABLE.raw = 0x00000000;

    //GPIO_SET(GPIOA, 1 << USB_ENABLE);
}

pub fn usb_connect(connect: bool) u8 {
    if (connect) {
        // set DP high?
        hal.gpio.write(USB_DP, hal.gpio.State.high);
        // 0b0010
        hal.gpio.set_output(USB_DP, hal.gpio.OutputMode.pushpull, hal.gpio.OutputSpeed.output_2MHz);
    } else {
        // set DP floating input
        hal.gpio.set_input(USB_DP, hal.gpio.InputMode.floating);
    }
    const usbd_lane_unk = 0;

    return usbd_lane_unk;
}

fn usb_enable(enable: bool) void {
    if (enable) {
        // PA11 = USB-  PA12 = USB+
        // assume both pins on same port
        hal.gpio.enable_port(USB_DM);

        peripherals.RCC.APB2ENR.modify(.{ .IOPAEN = 0b1 });
        peripherals.RCC.APB1ENR.modify(.{ .USBEN = 0b1 });

        // reset device
        peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b1 });
        peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b0 });

        // set control bits
        peripherals.USB.CNTR.modify(.{
            .CTRM = 0b1,
            .RESETM = 0b1,
            .ERRM = 0b1,
            .SOFM = 0b1,
            .SUSPM = 0b1,
            .WKUPM = 0b1,
        });
    } else {
        if (peripherals.RCC.APB1ENR.read().USBEN == 0b1) {
            // set reset and disable
            peripherals.RCC.APB1RSTR.modify(.{ .USBRST = 0b1 });
            peripherals.RCC.APB1ENR.modify(.{ .USBEN = 0b0 });
            // disconnecting DP if configured
            // connect(0);
        }
    }

    //     } else if (RCC->APB1ENR & RCC_APB1ENR_USBEN) {
    //         RCC->APB1RSTR |= RCC_APB1RSTR_USBRST;
    //         RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;
    //         /* disconnecting DP if configured */
    //         connect(0);
    //     }
}

pub fn setup_usb() void {
    usb_enable(true);

    // enable USB device
    peripherals.RCC.APB1ENR.modify(.{ .USBEN = 0b1 });
}

pub fn main() !void {
    hal.rcc.setup_high_performance(8_000_000);

    // hal.rcc.dbg_show();

    // num 1 = USART2
    var uart = hal.usart.num(2);
    uart.apply(9600);

    hal.systick.apply();

    //hal.time.init();
    hal.usart.init_logger(uart);

    hal.gpio.set_output(LED_PIN, hal.gpio.OutputMode.pushpull, hal.gpio.OutputSpeed.output_10MHz);

    //uart.init(9600);

    usb_init();

    // main loop
    var loop_idx: u32 = 0;

    while (true) {
        uart.put_hex(loop_idx);
        uart.put_char('\r');
        uart.put_char('\n');
        loop_idx += 1;

        hal.systick.delay_ms(5000);
    }
}
