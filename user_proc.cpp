/***********************************************************************
user_proc code for project 6:

Author: Tyler Martin
Date: 12/15/2021

edited
i::

-user_proc is the binary that actually runs as a service that our OSS
 manages. It pretends to do work that requires memory. It periodically 
 sends a message back to OSS containing a memory address, and a conditional 
 action for that address, read or write. It continues until it reaches termination,
 and waits in a blocked state after it's request if it can't be granted immediately

Testing steps:
Program accesses the shared memory structs and message queues and obtains
its PID. It then requests memory at random times either reading or writing, and waits
until it gets a message back and repeats until exit. It cleans up on termination.
***********************************************************************/

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <random>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <time.h>
#include <ios>
#include <string>
#include <vector>
#include <tuple>
#include "string.h"
#include "sys/msg.h"
#include "structures.h"

//other

//key for address location
#define SHM_KEY 0x98273
#define PARENT_KEY 0x94512
#define TURN_KEY 0x95712
#define KEY 0x00043 //child-to-parent key
#define ResourceMatrices_KEY 0x46583
#define SCHEDULER_KEY 0x46513
#define ResourceQueue_KEY 0x71623

//perror header
std::string exe;

//message object
struct message msg;

//int to hold which processID we take up
int ProcessID = 0;

//float to hold effective access time stats
float dirty = 0;
float totalreq = 0;
float effectiveAccessTime=0;

/***********************************************************************
 create shared message queues 

 Queues: 
        Queue for child-to-parent communication
        Queue for parent check-up
        Queue for any general purpose communication that may be needed later
        Queue for waiting for responses from OSS
***********************************************************************/
int msgid = msgget(KEY, 0666 | IPC_CREAT); 

int parent_msgid = msgget(PARENT_KEY, 0666 | IPC_CREAT);

int resource_msgid = msgget(ResourceQueue_KEY, 0666 | IPC_CREAT); 

int returning_msgid = msgget(SCHEDULER_KEY, 0666 | IPC_CREAT); 

/***********************************************************************
 create shared message memory segments 

 Objects:
        Clock Object for time management
        Turn Management Semaphore for starting children
        The Shared Semaphore for Zombie collection
***********************************************************************/
int shmid = shmget(SHM_KEY, sizeof(struct Clock), 0600|IPC_CREAT);
struct Clock *currentClockObject = (static_cast<Clock *>(shmat(shmid, NULL, 0)));

int Turn_shmid = shmget(TURN_KEY, sizeof(struct TurnSemaphore), 0600|IPC_CREAT);
struct TurnSemaphore *currentTurnSemaphore = (static_cast<TurnSemaphore *>(shmat(Turn_shmid, NULL, 0)));

/***********************************************************************
Function: determineProcessIDandInstantiate()

Description: Function that retrieves our PID 
***********************************************************************/
int determineProcessIDandInstantiate(){
    //get our PID
    pid_t tmp_pid_c = getpid();
    return tmp_pid_c;
}

/***********************************************************************
Function: int doWork()

Description: Function that executes when child is put into the running
             state as indicated by the shared semaphore that controls
             the running PID. it requests random memory segments either
             as read or write, and waits for confirmation. Termination of 
             the children is handled on a check of whether or not the child
             has requested 1000 or more memory pieces (since the instructions
             say to do this every 1000 +- 100, I just have it hard set to check every
             1 over 1000)
***********************************************************************/
void doWork(int PID){
    
    //keep track of whether or not we terminated
    bool terminated = false;
    int i = 0;
    int fulfilled = 0;

    //boolean to control reading and writing
    bool reading = 1;

    //update global pid value
    ProcessID = PID;
    float previous = currentClockObject->getFractionalStamp();

    //random seeds and generators
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> requestTimeRange(0.0, 250000000.0);
    std::uniform_real_distribution<double> exit(0.0, 10.0);
    std::uniform_real_distribution<double> memreq(0.0, 31.0);
    std::uniform_real_distribution<double> offset(0.0, 1023.0);

    //get time to new request
    double requestTime = requestTimeRange(mt);
    requestTime /= 1000000000;

    //keep track of requests made
    int requestsMade=0;

    //Get process Start Time
    double startingStamp = currentClockObject->getFractionalStamp();
    double previousStamp = currentClockObject->getFractionalStamp();

    //set variable for blocked time logging
    std::string blockedStart="0";

    while(!terminated && !currentTurnSemaphore->getTurn(-2)){
        //determine if we are reading or writing this turn (preference given to reading)
        if(exit(mt) <= 3)
            reading = 0;
        else
            reading = 1;
        
        //generate random address
        int address = (memreq(mt) * 1024) + offset(mt);

        //make our request
        std::stringstream ss; 
        ss << PID << " " << address << " ";
        if(reading == 1)
            ss << "R";
        else 
            ss << "W";

        //convert request to string and then to char array
        std::string blocked = ss.str();
        std::string clean = "                          "; 
        clean.copy(msg.text, 200);
        blocked.copy(msg.text, 200);

        //send a message with a memory request and frame request, and Id a nd r/w (  PID address r/w  )
        if(msgsnd(parent_msgid, &msg, strlen(msg.text)+1, 0) == -1){
            std::cout << "Can't send request!\n";
            perror(&exe[0]);
        }

        //wait for a message to be received
        if (msgrcv(returning_msgid, &msg, sizeof(msg), PID ,0) == -1){
            std::cout << "stuck waiting for callback" << std::endl;
            perror(&exe[0]);
        }

        //get dirty or clean
        if(msg.text[0] == 'd')
            dirty++;
        totalreq++;

        //kill after 1000 requests
        if (requestsMade > 1000)
            terminated = true;
        
        requestsMade++;

        //save a bit of time and return if time
        if(terminated || currentTurnSemaphore->getTurn(-2)){
            return;
        }
    }
}

int main(int argc, char *argv[]){

    //Get the perror header
    std::ifstream("/proc/self/comm") >> exe;
    exe.append(": Error");

    //check all memory allocations
    if(shmid == -1) {
        std::cout <<"Pre-Program Memory allocation error. Program must exit.\n";
        errno = 11;
        perror(&exe[0]);
    }

    //actually do work when allowed
    doWork(determineProcessIDandInstantiate());

    //close memory
    shmdt((void *) currentTurnSemaphore);
    shmdt((void *) currentClockObject);

    //calc effective access time
    effectiveAccessTime = static_cast<float>(10.0 + static_cast<float>(static_cast<float>(dirty/totalreq) * 14000000.0));

    //build exit message
    std::stringstream ss; 
    ss << ProcessID << " has exited, and had an effective memory access time of " << static_cast<float>(effectiveAccessTime / 1000000) << "ms";
    std::string blocked = ss.str();
    std::string clean = "                          "; 
    clean.copy(msg.text, 200);
    blocked.copy(msg.text, 200);

    //send an exit message
    if(msgsnd(resource_msgid, &msg, strlen(msg.text)+1, 0) == -1){
        std::cout << "Can't send request!\n";
        perror(&exe[0]);
    }

    exit(EXIT_SUCCESS);
}
