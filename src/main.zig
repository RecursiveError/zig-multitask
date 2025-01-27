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
var OS_inst: OS_base.Create_OS(4) = undefined;

fn delay(time: u32) void {
    const end = millis + time;
    //wait for millis
    while (millis < end) {
        std.mem.doNotOptimizeAway(end); //keep "end" in memory
    }
}

var t1_stack: [200]u32 = undefined;
fn task1() noreturn {
    const msg = "Hello Task1\n";
    while (true) {
        for (msg) |c| {
            GPIOC.ODR.ODR13 = 1;
            delay(15);
            GPIOC.ODR.ODR13 = 0;
            delay(15);
            const ret = OS_base.syscall(0, @as(u8, c), 0, 0);
            std.mem.doNotOptimizeAway(ret);
        }
        OS_inst.task_sleep(1000);
    }
}

var t2_stack: [200]u32 = undefined;
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
        OS_inst.task_sleep(200);
    }
}

var idle_stack: [200]u32 = undefined;
fn idle() noreturn {
    while (true) {
        OS_inst.yield();
    }
}

export fn main() noreturn {
    cortex_m3.disableInterrupts();
    init_gpio();
    init_kernel();
    init_UART();
    var idle_t = OS_base.Task.init(idle, &idle_stack);
    var t1 = OS_base.Task.init(task1, &t1_stack);
    var t2 = OS_base.Task.init(task2, &t2_stack);
    OS_inst = OS_base.Create_OS(4).init(idle_t.create_task(), &millis);

    OS_inst.add_task(t1.create_task()) catch unreachable;
    OS_inst.add_task(t2.create_task()) catch unreachable;
    cortex_m3.enableInterrupts();
    OS_inst.start_scheduler();
    while (true) {
        asm volatile ("wfe");
    }
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
    cortex_m3.SCB.set_IRQPrio(15, 4);
    cortex_m3.SCB.set_IRQPrio(14, 1);
    cortex_m3.SCB.set_IRQPrio(11, 1);
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
