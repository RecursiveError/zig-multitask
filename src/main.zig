const std = @import("std");
const stm32f1xx = @import("stm32f1xx.zig");
const cortex_m3 = @import("cortexm3.zig");
const OS_base = @import("base_OS.zig");
const peripherals = stm32f1xx.devices.STM32F103.peripherals;
const CPU_CLOCK = 8_000_000;

const GPIOC = peripherals.GPIOC;
const RCC = peripherals.RCC;
const GPIOA = peripherals.GPIOA;
const UART = peripherals.USART1;

//global time variable
var millis: u64 = 0;
var OS_inst: OS_base.Create_OS(5) = undefined;

fn delay(time: u32) void {
    const end = millis + time;
    //wait for millis
    while (millis < end) {
        std.mem.doNotOptimizeAway(end); //keep "end" in memory
    }
}

const sys_print = struct {
    fn write_bytes(_: void, byte: []const u8) anyerror!usize {
        for (byte) |b| {
            _ = OS_base.syscall(0, b, 0, 0);
        }

        return byte.len;
    }
    fn print(comptime fmt: []const u8, args: anytype) !void {
        var write = std.io.GenericWriter(void, anyerror, write_bytes){ .context = {} };
        try write.print(fmt, args);
    }
};

var t1_handler: *OS_base.Task = undefined;
fn task1() noreturn {
    var buffer: [25]u8 = undefined;
    var tick: usize = 0;
    while (true) {
        const msg = std.fmt.bufPrint(&buffer, "hello task1 {d}\n", .{tick}) catch unreachable;
        for (msg) |c| {
            GPIOC.ODR.ODR13 = 1;
            delay(15);
            GPIOC.ODR.ODR13 = 0;
            delay(15);
            _ = OS_base.syscall(0, @as(u8, c), 0, 0);
        }
        tick += 1;
        OS_inst.task_sleep(500);
    }
}

var t2_handler: *OS_base.Task = undefined;
fn task2() noreturn {
    const msg = "Hello Task2\n";
    while (true) {
        for (msg) |c| {
            GPIOC.ODR.ODR13 = 1;
            delay(15);
            GPIOC.ODR.ODR13 = 0;
            delay(15);
            _ = OS_base.syscall(0, @as(u8, c), 0, 0);
        }
        OS_inst.task_sleep(2500);
        _ = OS_inst.create_task(task3, 2000) catch continue;
    }
}

fn task3() noreturn {
    sys_print.print("hello task{d}\n", .{3}) catch {};
    OS_inst.exit();
}

var idle_stack: [200]u32 = undefined;
fn idle() noreturn {
    while (true) {
        OS_inst.yield();
    }
}

var buf: [16000]u8 = undefined;
export fn main() noreturn {
    cortex_m3.disableInterrupts();
    init_gpio();
    init_kernel();
    init_UART();
    var idle_t = OS_base.Task.init(idle, &idle_stack);

    var fba = std.heap.FixedBufferAllocator.init(&buf);
    OS_inst = OS_base.Create_OS(5).init(idle_t.create_task(), &millis, fba.allocator());

    t1_handler = OS_inst.create_task(task1, 1200) catch unreachable;
    t2_handler = OS_inst.create_task(task2, 600) catch unreachable;

    cortex_m3.enableInterrupts();
    OS_inst.start_scheduler();
}

fn init_gpio() void {
    RCC.APB2ENR.IOPCEN = 1; //enable GPIOC clock
    GPIOC.CRH.MODE13 = 2; //output with max clock 2Mhz
    GPIOC.CRH.CNF13 = 0; //push pull
}

fn init_UART() void {
    RCC.APB2ENR.USART1EN = 1; //enable USART1 clock
    RCC.APB2ENR.IOPAEN = 1;
    GPIOA.CRH.MODE9 = 1;
    GPIOA.CRH.CNF9 = 0b10;

    //GPIOA.CRH.MODE10 = 1;
    //GPIOA.CRH.CNF10 = 0b10;

    GPIOA.CRH.MODE12 = 1;
    GPIOA.CRH.CNF12 = 0b10;

    //CLOCK = 8Mhz
    //BRR = 115200
    //8000000/(16*115200);
    //H_BRR = 4 = 0x004
    //L_BRR = 5 = 0x5

    UART.BRR.DIV_Fraction = 0x5;
    UART.BRR.DIV_Mantissa = 0x4;
    UART.CR1.TE = 1;
    UART.CR1.RE = 1;
    UART.CR3.RTSE = 1;
    UART.CR1.UE = 1;
}

fn uart_transmite_blocking(data: []const u8) void {
    for (data) |c| {
        while (UART.SR.TC != 1) {}
        UART.DR.DR = @intCast(c);
        while (UART.SR.TXE != 1) {}
    }
}

fn uart_read_blocking(data: []u8, timeout: u64) usize {
    var bytes: usize = 0;
    for (data) |*c| {
        const deadline = millis + timeout;
        while (UART.SR.RXNE != 1) {
            std.mem.doNotOptimizeAway(deadline);
            if (millis > deadline) return bytes;
        }
        c.* = @intCast(UART.DR.DR);
        bytes += 1;
    }
    return bytes;
}

fn init_kernel() void {
    //Enable System Tick
    cortex_m3.SCB.set_IRQPrio(15, 1);
    cortex_m3.SCB.set_IRQPrio(14, 14);
    cortex_m3.SCB.set_IRQPrio(11, 13);
    cortex_m3.SCB.enable_IRQ(15);
    cortex_m3.SCB.enable_IRQ(14);
    cortex_m3.SCB.enable_IRQ(11);
    init_SysTick(CPU_CLOCK / 1000); //1ms per tick
}

fn init_SysTick(tick: u32) void {
    const sysTick = cortex_m3.SysTick;
    sysTick.CSR.* = 0; //disable SysTemTick
    sysTick.RVR.* = tick & 0x00FFFFFF; //set reload value
    sysTick.CVR.* = 0; //clear corrent value
    sysTick.CSR.* = 0b111; // enable Clock as the same as CPU(HSI) | enable IRQ on underflow | enable timer
}

//update millis every tick IRQ
export fn SystemTick_IRQ() callconv(.C) void {
    millis += 1;
}
