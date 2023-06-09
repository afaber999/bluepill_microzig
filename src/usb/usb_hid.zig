const std = @import("std");
const microzig = @import("microzig");
const hal = microzig.hal;
const board = microzig.board;
const usb = hal.usb;

// const baud_rate = 115200;
// const uart_tx_pin = gpio.num(0);
// const uart_rx_pin = gpio.num(1);

// First we define two callbacks that will be used by the endpoints we define next...
fn ep1_in_callback(dc: *usb.DeviceConfiguration, data: []const u8) void {
    _ = data;
    // The host has collected the data we repeated onto
    // EP1! Set up to receive more data on EP1.
    usb.Usb.callbacks.usb_start_rx(
        dc.endpoints[2], // EP1_OUT_CFG,
        64,
    );
}

fn ep1_out_callback(dc: *usb.DeviceConfiguration, data: []const u8) void {
    // We've gotten data from the host on our custom
    // EP1! Set up EP1 to repeat it.
    usb.Usb.callbacks.usb_start_tx(
        dc.endpoints[3], // EP1_IN_CFG,
        data,
    );
}

// The endpoints EP0_IN and EP0_OUT are already defined but you can
// add your own endpoints to...
pub var EP1_OUT_CFG: usb.EndpointConfiguration = .{
    .descriptor = &usb.EndpointDescriptor{
        .length = @intCast(u8, @sizeOf(usb.EndpointDescriptor)),
        .descriptor_type = usb.DescType.Endpoint,
        .endpoint_address = usb.Dir.Out.endpoint(1),
        .attributes = @enumToInt(usb.TransferType.Interrupt),
        .max_packet_size = 64,
        .interval = 0,
    },
    .endpoint_control_index = 2,
    .buffer_control_index = 3,
    .data_buffer_index = 2,
    .next_pid_1 = false,
    // The callback will be executed if we got an interrupt on EP1_OUT
    .callback = ep1_out_callback,
};

pub var EP1_IN_CFG: usb.EndpointConfiguration = .{
    .descriptor = &usb.EndpointDescriptor{
        .length = @intCast(u8, @sizeOf(usb.EndpointDescriptor)),
        .descriptor_type = usb.DescType.Endpoint,
        .endpoint_address = usb.Dir.In.endpoint(1),
        .attributes = @enumToInt(usb.TransferType.Interrupt),
        .max_packet_size = 64,
        .interval = 0,
    },
    .endpoint_control_index = 1,
    .buffer_control_index = 2,
    .data_buffer_index = 3,
    .next_pid_1 = false,
    // The callback will be executed if we got an interrupt on EP1_IN
    .callback = ep1_in_callback,
};

// This is our device configuration
pub var DEVICE_CONFIGURATION: usb.DeviceConfiguration = .{
    .device_descriptor = &.{
        .length = @intCast(u8, @sizeOf(usb.DeviceDescriptor)),
        .descriptor_type = usb.DescType.Device,
        .bcd_usb = 0x0200,
        .device_class = 0,
        .device_subclass = 0,
        .device_protocol = 0,
        .max_packet_size0 = 64,
        .vendor = 0xCafe,
        .product = 1,
        .bcd_device = 0x0100,
        // Those are indices to the descriptor strings
        // Make sure to provide enough string descriptors!
        .manufacturer_s = 1,
        .product_s = 2,
        .serial_s = 3,
        .num_configurations = 1,
    },
    .interface_descriptor = &.{
        .length = @intCast(u8, @sizeOf(usb.InterfaceDescriptor)),
        .descriptor_type = usb.DescType.Interface,
        .interface_number = 0,
        .alternate_setting = 0,
        // We have two endpoints (EP0 IN/OUT don't count)
        .num_endpoints = 2,
        .interface_class = 3,
        .interface_subclass = 0,
        .interface_protocol = 0,
        .interface_s = 0,
    },
    .config_descriptor = &.{
        .length = @intCast(u8, @sizeOf(usb.ConfigurationDescriptor)),
        .descriptor_type = usb.DescType.Config,
        .total_length = @intCast(u8, @sizeOf(usb.ConfigurationDescriptor) + @sizeOf(usb.InterfaceDescriptor) + @sizeOf(usb.EndpointDescriptor) + @sizeOf(usb.EndpointDescriptor)),
        .num_interfaces = 1,
        .configuration_value = 1,
        .configuration_s = 0,
        .attributes = 0xc0,
        .max_power = 0x32,
    },
    .lang_descriptor = "\x04\x03\x09\x04", // length || string descriptor (0x03) || Engl (0x0409)
    .descriptor_strings = &.{
        // ugly unicode :|
        //"R\x00a\x00s\x00p\x00b\x00e\x00r\x00r\x00y\x00 \x00P\x00i\x00",
        &usb.utf8ToUtf16Le("Raspberry Pi"),
        //"P\x00i\x00c\x00o\x00 \x00T\x00e\x00s\x00t\x00 \x00D\x00e\x00v\x00i\x00c\x00e\x00",
        &usb.utf8ToUtf16Le("Pico Test Device"),
        //"c\x00a\x00f\x00e\x00b\x00a\x00b\x00e\x00",
        &usb.utf8ToUtf16Le("cafebabe"),
    },
    .hid = .{
        .hid_descriptor = &.{
            .bcd_hid = 0x0111,
            .country_code = 0,
            .num_descriptors = 1,
            .report_length = 34,
        },
        .report_descriptor = &usb.hid.ReportDescriptorFidoU2f,
    },
    // Here we pass all endpoints to the config
    // Dont forget to pass EP0_[IN|OUT] in the order seen below!
    .endpoints = .{
        &usb.EP0_OUT_CFG,
        &usb.EP0_IN_CFG,
        &EP1_OUT_CFG,
        &EP1_IN_CFG,
    },
};

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("panic: {s}", .{message});
    @breakpoint();
    while (true) {}
}

pub const std_options = struct {
    pub const log_level = .debug;
    pub const logFn = hal.usart.log;
};

const LED_PIN = hal.parse_pin(board.pin_map.LED);

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
    hal.gpio.write(LED_PIN, hal.gpio.State.high);

    // First we initialize the USB clock
    hal.usb.Usb.init_clk();

    // Then initialize the USB device using the configuration defined above
    hal.usb.Usb.init_device(&DEVICE_CONFIGURATION) catch unreachable;
    //var old: u64 = time.get_time_since_boot().to_us();
    var old: u64 = 0;
    var new: u64 = 0;

    var loop_idx: u32 = 0;
    while (true) {
        std.log.info("Loop index from logging as err ... 0x{X:0>8}", .{loop_idx});

        // You can now poll for USB events
        hal.usb.Usb.task(
            true, // debug output over UART [Y/n]
        ) catch unreachable;

        // AF new = time.get_time_since_boot().to_us();
        if (new - old > 500000) {
            old = new;

            // AF led.toggle();
        }
        loop_idx = loop_idx +% 1;
    }
}
