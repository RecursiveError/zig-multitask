a simple example of a multitasking system for ARM Cortex-M3 made in Zig for SMT32F103

Zig: v0.14.0

This example contains:

- context switching between tasks using PendSV

- a basic syscall with SVCall

multitask functions:

- `create_task`: creates a task and adds it to the task pool
- `yield`: switch to next task
- `task_sleep`: suspends the current task for a period of time
- `exit`: removes the current task from the task pool
- `suspend_task`: suspends a current task until it is resumed by another task
- `resume_task`: resume a suspended task