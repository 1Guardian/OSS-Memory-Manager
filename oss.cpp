/***********************************************************************
OSS code for project 6:

Author: Tyler Martin
Date: 12/15/2021

- OSS is the Operating System Simulator. It acts as a simulated OS and
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

Testing steps:
Program allocates shared memory and message queues. It then creates two 
additional threads and begins forking children while managing 
memory requests from each request until it runs out of processes
or time elapsing. It logs all events while it executes and prints general
statistics at the end of the program's execution.
***********************************************************************/
#include <stdio.h>
#include <sys/ipc.h>
#include <signal.h>
#include <sys/shm.h>
#include <bits/stdc++.h>
#include <sys/types.h>
#include <algorithm>
#include <regex>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <chrono>
#include <random>
#include <vector>
#include <thread>
#include <tuple>
#include "sys/msg.h"
#include "string.h"
#include "structures.h"

//key for address location
#define SHM_KEY 0x98273
#define PARENT_KEY 0x94512
#define PCT_KEY 0x92612
#define TURN_KEY 0x95712
#define KEY 0x00043 //child-to-parent key
#define CLOCK_KEY 0x05912
#define PCB_KEY 0x46781
#define ResourceMatrices_KEY 0x46583
#define SCHEDULER_KEY 0x46513
#define FINAL_STATS_KEY 0x396561
#define ResourceQueue_KEY 0x71623
#define MaxClaimsQueue_KEY 0x19238

//sentinel to control whether or not to 
//print out the resource allocation matrix
bool verbose = false;

//number of running children
int runningChildren=0;

//argv[0] perror header
std::string exe;

//message object
struct message msg;

//logfile name and max exec time
std::string logfile = "logfile";
int maxExecutionTime = 2;

//frame table
struct FrameTable frames;

//PCT
std::vector<struct PCB> PCT;

//stat trackers
int ittertrack = 0;
std::vector<int> averageAccesses;
int faults = 0;
int accesses = 0;
int tenns = 0;
int fourteenns = 0;

/***********************************************************************
 create shared message queues 

 Queues: 
        Queue for child-to-parent communication
        Queue for parent check-up
        Queue for all-to-resource matrices communication
        Queue for all-to-maximum resource communication
        Queue for PCB access
        Queue for talking to scheduler
***********************************************************************/
int msgid = msgget(KEY, 0666 | IPC_CREAT); 

int parent_msgid = msgget(PARENT_KEY, 0666 | IPC_CREAT);

int resource_msgid = msgget(ResourceQueue_KEY, 0666 | IPC_CREAT); 

int maximum_msgid = msgget(MaxClaimsQueue_KEY, 0666 | IPC_CREAT); 

int PCB_msgid = msgget(PCB_KEY, 0666 | IPC_CREAT); 

int returning_msgid = msgget(SCHEDULER_KEY, 0666 | IPC_CREAT); 

/***********************************************************************
 create shared message memory segments 

 Objects:
        Clock Object for time management
        Turn Management Semaphore for starting children
        The Resource Manager Object
        The PIDS vector (made shared even though only runsim wil ever use it
                         so that all threads see it)
***********************************************************************/
int shmid = shmget(SHM_KEY, sizeof(struct Clock), 0600|IPC_CREAT);
struct Clock *currentClockObject = (static_cast<Clock *>(shmat(shmid, NULL, 0)));

int Turn_shmid = shmget(TURN_KEY, sizeof(struct TurnSemaphore), 0600|IPC_CREAT);
struct TurnSemaphore *currentTurnSemaphore = (static_cast<TurnSemaphore *>(shmat(Turn_shmid, NULL, 0)));

int PIDSVec_shmid = shmget(/*key defined here cause it isn't needed by any other process*/0x571254, sizeof(struct PIDSVector), 0600|IPC_CREAT);
struct PIDSVector *PIDSVec = (static_cast<PIDSVector *>(shmat(PIDSVec_shmid, NULL, 0)));

/***********************************************************************
Function: docommand(char*)

Description: Function to call the grandchildren 
             (user_proc). Uses execl to open a bash
             instance and then simply run the 
             program by providing the name of the bin
             and the relative path appended to the 
             beginning of the command.
***********************************************************************/
void docommand(char* command){

    //add relative path to bin
    std::string relativeCommand = "./";
    relativeCommand.append(command);

    //execute bin
    execl("/bin/bash", "bash", "-c", &relativeCommand[0], NULL);
}

/***********************************************************************
Function: deleteMemory(char*)

Description: Function to delete all shared memory
***********************************************************************/
void deleteMemory(){

    //delete each memory/queue share
    shmctl(shmid, IPC_RMID, NULL);
    msgctl(msgid, IPC_RMID, NULL);
    shmctl(PIDSVec_shmid, IPC_RMID, NULL);
    msgctl(parent_msgid, IPC_RMID, NULL);
    msgctl(resource_msgid, IPC_RMID, NULL);
    msgctl(PCB_msgid, IPC_RMID, NULL);
    msgctl(maximum_msgid, IPC_RMID, NULL);
    shmctl(Turn_shmid, IPC_RMID, NULL);
    msgctl(returning_msgid, IPC_RMID, NULL);
}

/***********************************************************************
Function: threadReturn()

Description: Function that a thread will inhabit with
             the sole purpose and intent of keeping 
             track of which child processes are still
             running
***********************************************************************/
void threadReturn()
{
    while(1){
        if(waitpid(-1, NULL, WNOHANG) > 0){
            PIDSVec->PIDS.pop_back();
        }
    }
}

/***********************************************************************
Function: cleanUp(bool)

Description: Function that receives control of the OSS program
             when the main scheduler is finished. It's job is to
             cleanup and delegate the deletion of shared memory, and make
             sure that no zombie processes are found.
             (Functionally a copy of threadkill)
***********************************************************************/
void cleanUp(bool show = true)
{
    //wait for children that are still de-coupling
    while(wait(NULL) > 0);

    //kill all children
    for(int i=0; i < PIDSVec->PIDS.size(); i++)
        kill(std::get<0>(PIDSVec->PIDS[i]), SIGTERM);

    //log any last minute returns to the log file
    std::ofstream outdata;
    outdata.open(logfile, std::ios_base::app);
    struct msqid_ds Return_msgid_stats;
    if( msgctl(resource_msgid, IPC_STAT, &Return_msgid_stats) == -1)
        perror(&exe[0]);
    for(int i =0; i < Return_msgid_stats.msg_qnum; i++){

        //get process ids
        if (msgrcv(resource_msgid, &msg, sizeof(msg), 0 ,0) == -1){
            perror(&exe[0]);
        }

        //print that process finished
        outdata << "MemoryManager: Process "<< std::string(msg.text) << std::endl;

    }
    outdata.close();
    //clear memory 
    deleteMemory();
    exit(1);
}

/***********************************************************************
Function: siginthandler(int)

Description: Function that receives control of
             termination when program has received 
             a termination signal
***********************************************************************/
void siginthandler(int param)
{
    //kill all children
    for(int i=0; i < PIDSVec->PIDS.size(); i++)
        kill(std::get<0>(PIDSVec->PIDS[i]), SIGTERM);

    //clear memory
    deleteMemory();
    exit(1);
}

/***********************************************************************
Function: threadKill(std::chrono::steady_clock::time_point)

Description: Function that receives control of
             termination when program has received 
             a termination signal from the time 
             contained in config.h being ellapsed.
***********************************************************************/
void threadKill(std::chrono::steady_clock::time_point start, int max_run_time)
{
    //don't execute until time has ellapsed.
    while((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < 2));

    //signal both scheduler and all children that exit time has arrived
    currentTurnSemaphore->setTurn(-2);

    //finish cleaning up and exit
    //cleanUp();
}

/***********************************************************************
Function: printStats()

Description: Prints the final running statistics of the Resource Manager

***********************************************************************/
void printStats(){

    int avgprocess=0;
    float pagefaultsper =0;

    //get average
    for(int i =0; i < averageAccesses.size(); i++){
        avgprocess += averageAccesses[i];
    }
    avgprocess = avgprocess / (static_cast<int>(currentClockObject->getFractionalStamp()));

    //get fault average
    pagefaultsper = (static_cast<float>(faults))/(static_cast<float>(accesses));

    std::cout << std::endl << "Operating System Simulator (Memory Manager) End of Run Statistics: " << std::endl;
    std::cout << "==========================================================================" << std::endl;
    std::cout << "Average number of memory accesses per second: " << avgprocess << std::endl;
    std::cout << "Average Number of page faults per memory access: " << pagefaultsper << std::endl;
    std::cout << "Average memory access speed: " << static_cast<float>((((10.0*static_cast<float>(tenns)) + (14.0*static_cast<float>(fourteenns)))/(static_cast<float>(tenns)+static_cast<float>(fourteenns)))) << "ns" << std::endl;
    cleanUp(false);
}

/***********************************************************************
Function: childProcess(std::string, pid_t, int)

Description: Function that executes docommand and 
             waits for child process to finish.
***********************************************************************/
void childProcess(std::string currentCommand, pid_t wpid, int status){

    //run docommand
    docommand(&currentCommand[0]);
}

/***********************************************************************
Function: getPIDFromPCT(int)

Description: Function to get index of PCB that matches PID
***********************************************************************/
int getPIDFromPCT(int PID){

    for(int i =0; i < PCT.size(); i++){
        if (PCT[i].getPID(PID))
            return i;
    }

    return -1;
}

/***********************************************************************
Function: initTable(int, int)

Description: Function that initializes the process control table
             for the new process which has just confirmed it's creation
***********************************************************************/
void initTable(int child_pid, int count){

    //since initialization passed, setup a PCT PCB for the new child
    //ask for permission
    if (msgrcv(PCB_msgid, &msg, sizeof(msg), 0 ,0) == -1){
        std::cout << "stuck waiting for permission to look at PCB" << std::endl;
        perror(&exe[0]);
    }

    //Since initialization passed, push pid to stack
    PIDSVec->PIDS.push_back(std::make_tuple (child_pid, count));

    //add child entry to the PCT
    struct PCB newEntry(child_pid);
    PCT.push_back(newEntry);

    //cede access to the PCB back
    if(msgsnd(PCB_msgid, &msg, strlen(msg.text)+1, 0) == -1){
        std::cout << "Can't cede control back to worker from PCB thread!\n";
        perror(&exe[0]);
    }
}

/***********************************************************************
Function: scheduleNewProcess(pid_t*, pid_t*, int*, int*)

Description: Function that executes the scheduling initilization duties
             creates children as well as assigns pids and
             waits for confimation from children
***********************************************************************/
void scheduleNewProcess(pid_t* child_pid, pid_t* wpid, int* status, int* count){

    //keep going until we hit max allowed processes (40 default)
    if(*count < 18){ 

        //don't let more than 20 processes run at any time
        if(PIDSVec->PIDS.size() < 18){

            //check to see if process is child or parent
            if((*child_pid = fork()) < 0){
                perror(&exe[0]);
            }
            if (*child_pid  == 0) {
                //start child process
                childProcess("user_proc", *wpid, *status);
            }

            //init the table entry
            initTable(*child_pid, *count);

            //update count
            *count = *count + 1;
            
        }
    }
}

/***********************************************************************
Function: handleRequests(pid_t*, pid_t*, int*)

Description: Function that actually handles all memory requests. It
             serves as the module that manages the page tables of the 
             children, while also managing the main memory frames.
***********************************************************************/
void handleRequests(pid_t* child_pid, pid_t* wpid, int* instatus){
    //hold status && PID
    int count = 0;

    //more modern use of C++11
    //random seeds and generators
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> blockedRand(1.0, 1000.0);
    std::uniform_real_distribution<double> timeToNewProcess(0.0, 500000.0);

    //variables to write to output file
    std::ofstream outdata;
    outdata.open(logfile);

    //initial starting time for use during averages and timing the spawning of children
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    //struct to hold all of our ctl params for our queues
    struct msqid_ds Parent_msgid_stats;
    struct msqid_ds Return_msgid_stats;

    //sentinel to save on execution time
    bool timerElapsed = false;

    //line tracker
    int lineCheck =0;

    //tracker for printing entire table
    int previousIntStamp = 0;

    //Print out Program Header
    std::cout << "CS 4760 Project 6 (Memory Manager)" << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << "File Being Logged To: " << logfile << std::endl;
    std::cout << "Resource Count: " << 20 << std::endl;
    std::cout << "Process Count: " << 18 << std::endl;
    fputs("====================================\n\nBeginning Run: .", stdout);
    fflush(stdout);


    //simple constant loop to keep our process in the context of this loop
    while(!currentTurnSemaphore->getTurn(-2)){  
        
        //if it's time to end, return
        if (PIDSVec->PIDS.empty() && timerElapsed || PIDSVec->PIDS.size() == 1 && timerElapsed){
            outdata << "ResourceManager: All Requests Handled. Exiting." << std::endl;

            //signal any zombies that it's time to exit
            currentTurnSemaphore->setTurn(-2);
            return;
        }

        //log any newly returned processes
        if( msgctl(resource_msgid, IPC_STAT, &Return_msgid_stats) == -1)
            perror(&exe[0]);

        if(Return_msgid_stats.msg_qnum > 0){
            int returns = Return_msgid_stats.msg_qnum;
            for(int i =0; i < returns; i++){

                //get process ids
                if (msgrcv(resource_msgid, &msg, sizeof(msg), 0 ,0) == -1){
                    perror(&exe[0]);
                }

                //print that process finished
                outdata << "MemoryManager: Process "<< std::string(msg.text) << std::endl;

            }
        }


        //schedule new process if possible 
        //condition that prevents even entering is if time has elapsed
        if((std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() <= MAX_INITIALIZATION_TIME)){
            scheduleNewProcess(child_pid, wpid, instatus, &count);
        }
        else{
            //if timer is in excess of 2 seconds, stop spawning processes
            timerElapsed = true;
        }   

        //======================================================
        //
        // NEW INCOMING MEMORY REQUESTS
        //
        //======================================================
        //check the queue to see if we have a request waiting or if we can tend to something else
        if( msgctl(parent_msgid, IPC_STAT, &Parent_msgid_stats) == -1)
            perror(&exe[0]);

        if(Parent_msgid_stats.msg_qnum > 0){

            //print activity notification
            if(lineCheck % 200 == 0){
                fputs("...", stdout);
                fflush(stdout);
            }

            //go through all incoming memory requests
            int msgNum = Parent_msgid_stats.msg_qnum;
            for(int i =0; i < msgNum; i++){

                //update ittertrack
                ittertrack++;

                //update accesses
                accesses++;

                //print the table if we ellapsed 1 second of our logical clock
                if(static_cast<int>(currentClockObject->getFractionalStamp()) > previousIntStamp){

                    //update the stamp
                    previousIntStamp = static_cast<int>(currentClockObject->getFractionalStamp());

                    //print the memory table
                    lineCheck++;
                    outdata << "Current memory layout at time " << currentClockObject->getTimeStamp() << " is: " << std::endl;
                    lineCheck++;
                    outdata << std::setw(9) << "Frame Number" << std::setw(9) << "Occupied" << std::setw(9) << "DirtyBit" << std::endl;
                    for(int i =0; i <255; i++){
                        lineCheck++;
                        outdata << std::setw(9) << "Frame " << i << std::setw(9) << frames.isFrameInUse(i) << std::setw(9) << frames.GetDirtyBit(i) << std::endl;
                    }

                    //update the stat for number of memory accesses per second
                    averageAccesses.push_back(ittertrack);
                    ittertrack = 0;

                }

                //Wait for a request to come through msg queue
                //FIXME: might want to change the message channel the 
                //request comes through later (for tidyness)
                if (msgrcv(parent_msgid, &msg, sizeof(msg), 0 ,0) == -1){
                    perror(&exe[0]);
                }

                //get request
                std::string currentRequest(msg.text);

                //make a string stream out of it and read request
                std::stringstream ss(currentRequest);
                std::vector<std::string> broken;

                //break request and fulfill it
                while(ss >> currentRequest){
                    broken.push_back(currentRequest);
                }

                //get all desired information
                int desiredPCT = getPIDFromPCT(stoi(broken[0]));
                int frame = stoi(broken[1]) / 1024;
                int offset = stoi(broken[1]) % 1024;
                std::string mode = broken[2];

                //check if the information is already in memory
                if(PCT[desiredPCT].getValueInFrame(frame) != -1){ //frame is already in the main memory

                    //add the time
                    if(frames.GetDirtyBit(PCT[desiredPCT].getValueInFrame(frame)) == 0){
                        currentClockObject->setTimeStamp(10000000);
                        tenns++;
                    }
                    else{
                        currentClockObject->setTimeStamp(14000000);
                        fourteenns++;
                    }
                    
                    //get mode 
                    if(mode == "R"){
                        //signal the process
                        msg.identifier = stoi(broken[0]);
                        std::string("clean").copy(msg.text, 200);
                        if(msgsnd(returning_msgid, &msg, strlen(msg.text)+1, 0) == -1){
                            std::cout << "Can't send signal!\n";
                            perror(&exe[0]);
                        }

                        //set dirty bit back
                        frames.SetDirtyBit(PCT[desiredPCT].getValueInFrame(frame), 0);

                        //write out
                        lineCheck++;
                        outdata << "MemoryManager: Process " << broken[0] << " requesting read of address " << broken[1] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                        lineCheck++;
                        outdata << "MemoryManager: Address " << broken[1] << " found in frame " << PCT[desiredPCT].getValueInFrame(frame) << ", giving data to Process " << broken[0] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                    }
                    else{
                        //signal the process
                        msg.identifier = stoi(broken[0]);
                        std::string("clean").copy(msg.text, 200);
                        if(msgsnd(returning_msgid, &msg, strlen(msg.text)+1, 0) == -1){
                            std::cout << "Can't send signal!\n";
                            perror(&exe[0]);
                        }

                        //set dirty bit back
                        frames.SetDirtyBit(PCT[desiredPCT].getValueInFrame(frame), 0);

                        //write out
                        lineCheck++;
                        outdata << "MemoryManager: Process " << broken[0] << " requesting write of address " << broken[1] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                        lineCheck++;
                        outdata << "MemoryManager: Address " << broken[1] << " is in frame " << PCT[desiredPCT].getValueInFrame(frame) << ", writing data to frame " << broken[0] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                    }

                }
                else{

                    //do stuff with the information
                    if(frames.getFrame(stoi(broken[0]), offset) != -1){
                        
                        //add the time
                        if(frames.GetDirtyBit(frames.getFrame(stoi(broken[0]), offset)) == 0){
                            currentClockObject->setTimeStamp(10000000);
                            tenns++;
                        }
                        else{
                            currentClockObject->setTimeStamp(14000000);
                            fourteenns++;
                        }

                        if(mode == "R"){
                            //signal the process
                            msg.identifier = stoi(broken[0]);
                            std::string("clean").copy(msg.text, 200);
                            if(msgsnd(returning_msgid, &msg, strlen(msg.text)+1, 0) == -1){
                                std::cout << "Can't send signal!\n";
                                perror(&exe[0]);
                            }

                            //set dirty bit back
                            frames.SetDirtyBit(PCT[desiredPCT].getValueInFrame(frame), 0);

                            //write out
                            lineCheck++;
                            outdata << "MemoryManager: Process " << broken[0] << " requesting read of address " << broken[1] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                            lineCheck++;
                            outdata << "MemoryManager: Address " << broken[1] << " is in frame " << PCT[desiredPCT].getValueInFrame(frame) << ", reading data from frame " << broken[0] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                        }
                        else{
                            //signal the process
                            msg.identifier = stoi(broken[0]);
                            std::string("clean").copy(msg.text, 200);
                            if(msgsnd(returning_msgid, &msg, strlen(msg.text)+1, 0) == -1){
                                std::cout << "Can't send signal!\n";
                                perror(&exe[0]);
                            }

                            //set dirty bit back
                            frames.SetDirtyBit(PCT[desiredPCT].getValueInFrame(frame), 0);

                            //write out
                            lineCheck++;
                            outdata << "MemoryManager: Process " << broken[0] << " requesting write of address " << broken[1] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                            lineCheck++;
                            outdata << "MemoryManager: Address " << broken[1] << " is in frame " << PCT[desiredPCT].getValueInFrame(frame) << ", writing data to frame " << broken[0] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                        }
                    }
                    else{

                        //update the page fault stat
                        faults++;
                        
                        //add to the main mem frame
                        std::string returned = frames.setFrame(0, stoi(broken[0]), offset);
                        std::string holder;

                        //check the returned value
                        std::stringstream st(returned); 
                        std::vector<int> returnbroken;

                        //break request and fulfill it
                        while(st >> holder){
                            returnbroken.push_back(stoi(holder));
                        }

                        //add the time
                        if(frames.GetDirtyBit(frames.getFrame(stoi(broken[0]), offset)) == 0){
                            currentClockObject->setTimeStamp(10);
                            tenns++;
                        }
                        else{
                            currentClockObject->setTimeStamp(14000000);
                            fourteenns++;
                        }
                            

                        if(returnbroken[0] != -1){

                            //update the PCB of the mem value that was kicked
                            int slot = PCT[getPIDFromPCT(returnbroken[0])].findFrame(returnbroken[2]);

                            if(slot != -1)
                                PCT[getPIDFromPCT(returnbroken[0])].setFrame(-1, slot);
                        }

                        //add the data to the PCB frame
                        PCT[desiredPCT].setFrame(returnbroken[2], frame);

                        if(mode == "R"){
                            //signal the process
                            msg.identifier = stoi(broken[0]);
                            std::string("dirty").copy(msg.text, 200);
                            if(msgsnd(returning_msgid, &msg, strlen(msg.text)+1, 0) == -1){
                                std::cout << "Can't send signal!\n";
                                perror(&exe[0]);
                            }

                            //write out
                            lineCheck++;
                            outdata << "MemoryManager: Process " << broken[0] << " requesting read of address " << broken[1] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                            lineCheck++;
                            outdata << "MemoryManager: Address " << broken[1] << " is not in a frame, pagefault" << std::endl;
                            if(returnbroken[1] == -1){
                                lineCheck++;
                                outdata << "MemoryManager: Empty Frame Found. Writing page to empty frame" << std::endl;
                            }
                            else{
                                lineCheck++;
                                outdata << "MemoryManager: Clearing frame "<< returnbroken[2] << " and swapping in address " << broken[1] << std::endl;
                                lineCheck++;
                                outdata << "MemoryManager: Dirty bit of frame "<< returnbroken[2] << " set, adding additional time to the clock " << std::endl;
                                lineCheck++;
                                outdata << "MemoryManager: Indicating to process "<< broken[0] << " that write has happened to address " << broken[1] << std::endl;
                            }
                        }
                        else{
                            //signal the process
                            msg.identifier = stoi(broken[0]);
                            std::string("dirty").copy(msg.text, 200);
                            if(msgsnd(returning_msgid, &msg, strlen(msg.text)+1, 0) == -1){
                                std::cout << "Can't send signal!\n";
                                perror(&exe[0]);
                            }

                            //write out
                            lineCheck++;
                            outdata << "MemoryManager: Process " << broken[0] << " requesting write of address " << broken[1] << " at time " << currentClockObject->getTimeStamp() << std::endl;
                            lineCheck++;
                            outdata << "MemoryManager: Address " << broken[1] << " is not in a frame, pagefault" << std::endl;
                            lineCheck++;
                            if(returnbroken[1] == -1){
                                lineCheck++;
                                outdata << "MemoryManager: Empty Frame Found. Writing page to empty frame" << std::endl;
                            }
                            else{
                                outdata << "MemoryManager: Clearing frame "<< returnbroken[2] << " and swapping in address " << broken[1] << std::endl;
                                lineCheck++;
                                outdata << "MemoryManager: Dirty bit of frame "<< returnbroken[2] << " set, adding additional time to the clock " << std::endl;
                                lineCheck++;
                                outdata << "MemoryManager: Indicating to process "<< broken[0] << " that read has happened from address " << broken[1] << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }

    //signal all children so that they end
    int msgNum = Parent_msgid_stats.msg_qnum;
    for(int i =0; i < PCT.size(); i++){

        //signal the process so that we can clean up
        msg.identifier = PCT[i].returnPID();
        if(msgsnd(returning_msgid, &msg, strlen(msg.text)+1, 0) == -1){
            std::cout << "Can't send signal!\n";
            perror(&exe[0]);
        }
    }
    outdata.close();
}

/***********************************************************************
Function: main(int, char*)

Description: Main function. Starts by checking parameters and
             allocated memory. It then ensures that the correct 
             pthreads are created and dispatched. It then inits 
             the queue and then starts the scheduler. 
***********************************************************************/
int main(int argc, char *argv[])
{
    //Handle Command Line Switches
    int opt;
    while ((opt = getopt(argc, argv, ":")) != -1) {
        switch (opt) {

        // an unknown argument is supplied
        case '?':
            std::cout << "Invalid option supplied. Terminating." << std::endl;
            cleanUp();
            break;
        }
    }
    
    //Get the perror header
    std::ifstream("/proc/self/comm") >> exe;
    exe.append(": Error");

    //capture sigint 
    signal(SIGINT, siginthandler);

    //stream holders
    std::string currentCommand;

    //forking variables
    pid_t child_pid = 0, wpid = 0;
    int max_run_time = 30;
    int status = 0;

    //check sharedmem
    if(shmid == -1 || Turn_shmid == -1 || PIDSVec_shmid == -1){
        std::cout << "Shared Memory Error!\n";
        perror(&exe[0]);
    }

    //initialize the PCB queue
    if(msgsnd(PCB_msgid, &msg, strlen(msg.text)+1, 0) == -1){
        std::cout << "Message Failed to send!\n";
        perror(&exe[0]);
    }

    //initialize the Resource queue
    if(msgsnd(resource_msgid, &msg, strlen(msg.text)+1, 0) == -1){
        std::cout << "Message Failed to send!\n";
        perror(&exe[0]);
    }

    //initialize the Maximum queue
    if(msgsnd(maximum_msgid, &msg, strlen(msg.text)+1, 0) == -1){
        perror(&exe[0]);
    }

    //make our time-watching thread and logging start-time
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::thread timeWatcher (threadKill, start, max_run_time);

    //childwatcher
    std::thread childrenWatcher (threadReturn);

    //call parent function
    handleRequests(&child_pid, &wpid, &status);

    //print activity notification
    std::cout << " Done!" << std::endl;

    //clean up sharedmemory, make sure no zombies are present, and exit
    printStats();
}
