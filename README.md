# OSS Memory Manager
# Notes

structures.h contains all data-structures and contains the #define 
variables. There are many shared memory segments and the program relies on 
all of them to run.

IMPORTANT NOTE(S): 
Inside of the log file, there are a few different messages that can appear.
The first of them is a memory request message, which looks like this:

    MemoryManager: Process 1182 requesting write of address 6576 at time 0:14000000

or

    MemoryManager: Process 1182 requesting read of address 6576 at time 0:14000000

From here, there are two messages that can appear, either a 'found' message if the 
page is in main memory, or a 'moving' message which indicates the page is being moved
to main memory now.

    MemoryManager: Address 6576 is not in a frame, pagefault
    MemoryManager: Empty Frame Found. Writing page to empty frame

or

    MemoryManager: Address 22557 found in frame 197, giving data to Process 1193 at time 5:938000000

the main memory is also periodically logged in table form, which looks 
like this:

    Current memory layout at time 6:8000000 is: 
    Frame Number Occupied DirtyBit
    Frame 0      Yes        1
    Frame 1      Yes        1
    Frame 2      Yes        1
    Frame 3      Yes        1
    Frame 4      Yes        1
    Frame 5      Yes        1
    ......      ....        ..
   Frame 252      Yes        1
   Frame 253      Yes        1
   Frame 254      Yes        0

And finally, when a process exits, a line like this is printed:
    MemoryManager: Process 16041 has exited, and had an effective memory access time of 7.5257ms

# OSS and user_proc (Test Process): Overview

OSS is the Operating System Simulator. It acts as a simulated OS and
memory manager implemented via a simulated main memory and page tables.
It has many message queues and shared objects that control 
the communication between this program and the child processes. It starts 
by creating the queues and spawning the first child. That child will note
it's PID similar to my last project, but this time the PIDS are obtained without
the parent's intervention in a *Nix/POSIX manner (probably what I should have done previously).
From here, the child will begin requesting memory segments randomly via a message queue
and the parent (OSS) will check the main memory to see if the page the child is looking
for is stored in any frames. If it is, it pretends to either read or write to that frame
and lets the child know it's request was granted. If the page is not in any frames, then using 
FIFO, it is swapped with another frame. The parent will continue to spawn new processes
until there are 18 spawned children at the same time. When all
children are done, they clean up and notify OSS. When a child requests a memory piece,
it waits in a blocked state until it can be granted.  One all children are done, OSS 
cleans up and prints the final running statistics.

user_proc is the binary that actually runs as a service that our OSS
manages. It pretends to do work that requires memory. It periodically 
sends a message back to OSS containing a memory address, and a conditional 
action for that address, read or write. It continues until it reaches termination,
and waits in a blocked state after it's request if it can't be granted immediately


# Getting Started (Compiling)

To compile both the OSS and user_proc programs, simple use of 'make'
will compile both for your use. 'make clean' will remove the created object files and the
executable binary that was compiled using the object files
