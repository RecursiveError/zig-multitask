const std = @import("std");
const alloc = std.mem.Allocator;
const cortex_m = @import("cortexm3.zig");
const stm32f1xx = @import("stm32f1xx.zig");

const peripherals = stm32f1xx.devices.STM32F103.peripherals;
const UART = peripherals.USART1;
const TaskCallBack = *const fn () noreturn;

pub const TaskState = union(enum) {
    active: void,
    @"suspend": void,
    delete: void,
    sleep: u64,
};
pub const Task = struct {
    stack: []u32,
    Task_fn: TaskCallBack,
    stack_pointer: *u32 = undefined,
    state: TaskState = .{ .active = {} },

    pub fn init(func: TaskCallBack, stack: []u32) Task {
        return .{
            .stack = stack,
            .Task_fn = func,
        };
    }

    pub fn create_task(self: *Task) *Task {
        var index = self.stack.len - 1;
        self.stack[0] = 0xC0DAF0F0;
        self.stack[index] = 0x01000000;
        index -= 1;
        self.stack[index] = @intCast(@intFromPtr(self.Task_fn));
        index -= 1;
        self.stack[index] = 0xFFFFFFF9;
        index -= 1;
        for (0..13) |_| {
            self.stack[index] = 0;
            index -= 1;
        }
        self.stack_pointer = &self.stack[index + 1];
        return self;
    }
};

var next_Task: *Task = undefined;
var corrent_task: *Task = undefined;
pub fn Create_OS(pool_size: comptime_int) type {
    return struct {
        const Self = @This();
        task_pool: std.fifo.LinearFifo(*Task, .{ .Static = pool_size }),
        idle_task: *Task,
        tick_var: *u64,
        OS_mem: alloc,

        pub fn init(
            idle: *Task,
            tick: *u64,
            mem: alloc,
        ) @This() {
            return .{
                .task_pool = std.fifo.LinearFifo(*Task, .{ .Static = pool_size }).init(),
                .idle_task = idle,
                .tick_var = tick,
                .OS_mem = mem,
            };
        }

        pub fn create_task(self: *Self, task_fn: TaskCallBack, stack_mem: usize) !*Task {
            if (self.task_pool.writableLength() < 2) return error.TaskPoolFull;
            var new_task: *Task = try self.OS_mem.create(Task);
            new_task.stack = self.OS_mem.alloc(u32, stack_mem) catch |err| {
                self.OS_mem.destroy(new_task);
                return err;
            };
            new_task.Task_fn = task_fn;
            new_task.state = .{ .active = {} };
            self.task_pool.writeItem(new_task.create_task()) catch unreachable;
            return new_task;
        }

        pub fn start_scheduler(self: *Self) noreturn {
            const stack_addr = self.idle_task.stack_pointer;
            corrent_task = self.idle_task;

            //start PSP for thread mode
            asm volatile ("MSR CONTROL, %[value]"
                :
                : [value] "r" (2),
                : "memory"
            );

            //set PSP fpr the task addrs
            asm volatile ("MSR PSP, %[addr]"
                :
                : [addr] "r" (stack_addr),
                : "memory"
            );

            //call pendSVS_IRQ
            cortex_m.SCB.ICSR.* |= 1 << 28;
            cortex_m._ISB();
            cortex_m._DSB();
            while (true) {
                asm volatile ("wfe");
            }
        }

        pub fn yield(self: *Self) void {
            cortex_m.disableInterrupts();

            const task_qtd = self.task_pool.readableLength();
            if (corrent_task != self.idle_task) {
                self.task_pool.writeItem(corrent_task) catch unreachable;
            }
            var next = self.idle_task;
            for (0..task_qtd) |_| {
                const task = self.task_pool.readItem().?;
                switch (task.state) {
                    .active => {
                        next = task;
                        break;
                    },
                    .sleep => |deadline| {
                        if (self.tick_var.* > deadline) {
                            task.state = .{ .active = {} };
                            next = task;
                            break;
                        }
                    },
                    .delete => {
                        self.OS_mem.free(task.stack);
                        self.OS_mem.destroy(task);
                        continue;
                    },
                    else => {},
                }
                self.task_pool.writeItem(task) catch unreachable;
            }
            next_Task = next;
            cortex_m.SCB.ICSR.* |= 1 << 28;
            cortex_m.enableInterrupts();
            cortex_m._ISB();
            cortex_m._DSB();
        }

        pub fn task_sleep(self: *Self, tick: u64) void {
            const daedline = self.tick_var.* + tick;
            corrent_task.state = .{ .sleep = daedline };
            self.yield();
        }

        pub fn exit(self: *Self) noreturn {
            corrent_task.state = .{ .delete = {} }; //just mark task for delete, task will be delete in next yield run
            self.yield();
            while (true) {}
        }

        pub fn suspend_task(self: *Self) void {
            corrent_task.state = .{ .@"suspend" = {} };
            self.yield();
        }

        pub fn resume_task(self: *Self, ts: *Task) void {
            const read_size = self.task_pool.readableLength();
            for (0..read_size) |_| {
                const task = self.task_pool.readItem().?;
                if (task == ts) {
                    ts.state = .{ .active = {} };
                }
                self.task_pool.writeItem(task) catch unreachable;
            }
        }

        fn add_task(self: *Self, task: *Task) !void {
            if (self.task_pool.writableLength() < 2) return error.TaskPoolFull;
            self.task_pool.writeItem(task) catch unreachable;
        }
    };
}

var boot = true;
pub export fn pendSV_IRQ() callconv(.C) void {
    cortex_m.disableInterrupts();
    if (boot) {
        boot = false;
        //bootstrap first Task
        const point = corrent_task.stack_pointer;

        //just load current task
        asm volatile (
            \\MSR PSP, %[addr]
            \\MRS R0, PSP
            \\LDMIA R0!, {R4-R11}
            \\MSR PSP, R0
            :
            : [addr] "r" (point),
            : "PSP", "R0"
        );
    } else {
        var __temp: u32 = 0;

        //save current_task context and update the task stack pointer
        asm volatile (
            \\MRS R0, PSP
            \\STMDB R0!, {R4-R11}
            \\MSR PSP, R0
            \\MRS %[stack_p], PSP
            : [stack_p] "=r" (__temp),
            :
            : "PSP", "R0"
        );
        corrent_task.stack_pointer = @ptrFromInt(__temp);

        //load next context task and return
        corrent_task = next_Task;
        const point = corrent_task.stack_pointer;
        asm volatile (
            \\MSR PSP, %[addr]
            \\MRS R0, PSP
            \\LDMIA R0!, {R4-R11}
            \\MSR PSP, R0
            :
            : [addr] "r" (point),
            : "PSP", "R0"
        );
    }
    //clear pendSV IRQ
    cortex_m.SCB.ICSR.* |= 1 << 27;
    cortex_m.enableInterrupts();
}

pub fn syscall(comptime sys_id: u8, arg0: u32, arg1: u32, arg2: u32) usize {
    const sysret = asm volatile (
        \\MOV R0, %[arg0]
        \\MOV R1, %[arg1]
        \\MOV R2, %[arg2]
        \\svc %[sys_id]
        \\MOV %[ret], R0
        : [ret] "=r" (-> usize),
        : [arg0] "r" (arg0),
          [arg1] "r" (arg1),
          [arg2] "r" (arg2),
          [sys_id] "i" (sys_id),
        : "R0", "R1", "R3"
    );
    return sysret;
}

pub export fn SVcall_IRQ() callconv(.C) void {
    cortex_m.disableInterrupts();
    var stack: u32 = undefined;

    //this code only uses syscalls from thread mode so no need to chech for MSP
    asm volatile (
        \\MRS %[stack], PSP 
        : [stack] "=r" (stack),
    );

    const stackP: [*]u32 = @ptrFromInt(stack);
    const r0 = stackP[0];
    const num: [*]u8 = @ptrFromInt(stackP[6] - 2);

    switch (num[0]) {
        0 => {
            uart_transmite_byte(@intCast(r0));
            stackP[0] = 0;
        },
        else => {},
    }
    cortex_m.enableInterrupts();
}

//just a test for syscall
fn uart_transmite_byte(c: u8) void {
    while (UART.SR.TC != 1) {}
    UART.DR.DR = @intCast(c);
    while (UART.SR.TXE != 1) {}
}

fn uart_transmite_blocking(data: []const u8) void {
    for (data) |c| {
        while (UART.SR.TC != 1) {}
        UART.DR.DR = @intCast(c);
        while (UART.SR.TXE != 1) {}
    }
}
