#include <string>
#include <vector>
#include <tuple>
#include <random>
#include <iostream>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <algorithm>

//other stuff 
//more stuff 
//stuff here more stuff  
// other stuff 
//Bakery maximum process amount
#define PROCESS_COUNT 20

//Maximum time to run the program
#define MAX_RUN_TIME 100

//Maximum time to allow child creation
#define MAX_INITIALIZATION_TIME 5

//Maximum child processes to allow
#define MAX_PROCESSES 40

// Number of processes
const int P = 18;
  
// Number of resources
const int R = 20;

//Message Queue Struct
struct message{
    long identifier =1;
    char text[200];
};

//Clock Object 
struct Clock
{
    private:
       unsigned int seconds;
       unsigned int nanoseconds;

    public:

        /***********************************************************************
        Function: Clock->getTimeStamp()

        Description: Function to get a Time Stamp at the requested
                     point in time.
        ***********************************************************************/
        std::string getTimeStamp(){
            return (std::to_string(seconds) + ":" + std::to_string(nanoseconds));
        }

        /***********************************************************************
        Function: Clock->getFractionalStamp()

        Description: Function to get the clock as a measurement of only the 
                     ellapsed factional seconds
        ***********************************************************************/
        double getFractionalStamp(){
            return (seconds + (nanoseconds/1000000000));
        }

        /***********************************************************************
        Function: Clock->setTimeStamp(unsigned int nanoseconds)

        Description: Function to set a Time Stamp at the requested
                     point in time.
        ***********************************************************************/
        void setTimeStamp(unsigned int newnanoseconds){

            //add the nanoseconds
            nanoseconds += newnanoseconds;

            //rollover into seconds
            if(nanoseconds >= 1000000000){
                seconds += 1;
                nanoseconds -= 1000000000;
            }
        }
};

//Semaphore for Turn Taking
struct TurnSemaphore{

    private:
        int turn =0;
        int kill =-1;

    public:
        /***********************************************************************
        Function: TurnSemaphore->setTurn(int)

        Description: Function to set which child's turn it is
        ***********************************************************************/
        void setTurn(int PID){

            //update the semaphore
            turn = PID;
        }

        /***********************************************************************
        Function: TurnSemaphore->setKill(int)

        Description: Function to set which child's turn to exit it is
        ***********************************************************************/
        void setKill(int PID){

            //update the semaphore
            kill = PID;
        }

        /***********************************************************************
        Function: TurnSemaphore->getKill(int)

        Description: Function to get which child's turn to exit it is
        ***********************************************************************/
        bool getKill(int PID){
            //return true if PID matches turn
            return (PID == kill);
        }

        /***********************************************************************
        Function: TurnSemaphore->returnTurn(int)

        Description: Function to set turn to 0
        ***********************************************************************/
        void returnTurn(){
            turn = 0;
        }

        /***********************************************************************
        Function: TurnSemaphore->getTurn(int)

        Description: Function to set which child's turn it is
        ***********************************************************************/
        bool getTurn(int PID){
            //return true if PID matches turn
            return (PID == turn);
        }
};

//vector for storing PIDS
struct PIDSVector{
    public:
        std::vector<std::tuple<int,int>> PIDS;

        /***********************************************************************
        Function: PIDSVector->getMask()

        Description: Function to get a pid mask value from the vector
        ***********************************************************************/
        int getMask(int PID){
            //find the desired PID mask and return it
            for(int i=0; i < PIDS.size(); i++){
                if(std::get<0>(PIDS[i]) == PID){
                    return std::get<1>(PIDS[i]);
                }                
            }
        }
};

//Semaphore for SIGINT Handling
struct SigSemaphore{

    private:
        bool kill = false;
        bool received = false;

    public:
        /***********************************************************************
        Function: SigSemaphore->signal()

        Description: Function to set whether signint has been passed
        ***********************************************************************/
        void signal(){

            //update the semaphore
            kill = true;
        }

        /***********************************************************************
        Function: SigSemaphore->signalhandshake()

        Description: Function to set whether signint has been passed
        ***********************************************************************/
        void signalhandshake(){

            //update the semaphore
            received = true;
        }

        /***********************************************************************
        Function: SigSemaphore->check()

        Description: Function to get whether signint has been passed
        ***********************************************************************/
        bool check(){
            return kill;
        }

        /***********************************************************************
        Function: SigSemaphore->handshake()

        Description: Function to get whether signint has been passed
        ***********************************************************************/
        bool handshake(){

            return received;
        }
};

struct PCB {
    private: 
        int PID;
        std::vector<int> pageTable;

    public:

        //constructor to set the pageTable to -1's
        PCB(int newPID){
            //init frame table
            for(int i =0; i < 32; i++){
                pageTable.push_back(-1);
            }
            //set pid of table
            PID = newPID;
        }

        /***********************************************************************
        Function: PCB->returnPID()

        Description: Function to return the pid of the desired PCB
        ***********************************************************************/
        int returnPID(){
            return PID;
        }

        /***********************************************************************
        Function: PCB->getPID()

        Description: Function to return T/F for whether this object matches the 
                     PID we're looking for
        ***********************************************************************/
        bool getPID(int desired_PID){
            return (PID == desired_PID);
        }

        /***********************************************************************
        Function: PCB->setFrame()

        Description: Function to set a value in the frametable
        ***********************************************************************/
        void setFrame(int value, int frame){
            pageTable[frame] = value;
        }

        /***********************************************************************
        Function: PCB->getFrame()

        Description: Function to check a value in the frame table 
        ***********************************************************************/
        bool getFrame(int value, int frame){
            return (pageTable[frame] == value);
        }

        /***********************************************************************
        Function: PCB->findFrame()

        Description: Function to get a value from the frame table (returns -1)
                     if it doesn't exist
        ***********************************************************************/
        int findFrame(int value){
            for(int i =0; i < 32; i++){
                if (pageTable[i] == value){
                    return i;
                }
            }
            return -1;
        }

        /***********************************************************************
        Function: PCB->getValueInFrame()

        Description: Function to get an index of a value from the frame table 
        ***********************************************************************/
        int getValueInFrame(int frame){
            return pageTable[frame];
        }

};

struct FrameTable{
    private: 
        std::vector<std::tuple<int,int>> frames;
        int dirty[255] = {0};
        int lastUsed = 0;

    public:
        //constructor to set the pageTable to -1's
        FrameTable(){
            for(int i =0; i < 255; i++){
                frames.push_back(std::make_tuple(-1,-1));
            }
        }
        /***********************************************************************
        Function: FrameTable->setFrame()

        Description: Function to set a value in the frametable
        ***********************************************************************/
        std::string setFrame(int frame, int pid, int value){

            //set frame if emptyframe present
            for(int i =0; i < 255; i++){
                if(std::get<0>(frames[i]) == -1){
                    frames[i] = std::make_tuple(pid,value);
                    dirty[i] = 1;
                    return "-1 -1 " + std::to_string(i);
                }
            }

            //no empty frame present, set old frame to new frame
            std::stringstream ss;
            ss << std::get<0>(frames[lastUsed]) << " " << std::get<1>(frames[lastUsed]) << " " << lastUsed;
            frames[lastUsed] = std::make_tuple(pid,value);
            dirty[lastUsed] = 1;
            lastUsed++; 

            //rollover if needed
            if(lastUsed == 255){
                lastUsed = 0;
            }

            //return the values that got booted
            return ss.str();
        }

        /***********************************************************************
        Function: FrameTable->getFrame()

        Description: Function to get a value from the frame table (returns -1)
                     if it doesn't exist
        ***********************************************************************/
        int getFrame(int pid, int value){
            for(int i =0; i < 255; i++){
                if(pid == std::get<0>(frames[i]) && pid == std::get<1>(frames[i]))
                    return i;
            }
            return -1;
        }

        /***********************************************************************
        Function: FrameTable->isFrameInUse()

        Description: Function to check if a frame is in use (returns yes or no)
        ***********************************************************************/
        std::string isFrameInUse(int frame){
            if(std::get<0>(frames[frame]) != -1)
                return "Yes";
            else
                return "No";
        }

        /***********************************************************************
        Function: FrameTable->GetDirtyBit()

        Description: returns the dirty bit value of the desired frame
        ***********************************************************************/
        int GetDirtyBit(int frame){
            return dirty[frame];
        }

        /***********************************************************************
        Function: FrameTable->SetDirtyBit()

        Description: sets the dirty bit value of the desired frame
        ***********************************************************************/
        void SetDirtyBit(int frame, int val){
            dirty[frame] = val;
            return;
        }
};
