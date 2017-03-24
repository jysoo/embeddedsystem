#include "mbed.h"
#include "rtos.h"
#include <string>
#include <map>
#include <vector>

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
DigitalIn I1(I1pin);
DigitalIn I2(I2pin);
DigitalIn I3(I3pin);

InterruptIn CHAinterrupt(CHA);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

#define BUFLNTH 49
//new data consisting of the note and the sring
struct note_ele {
    string note;
    int8_t  beat;
};

vector<note_ele> vec_notes(16); //holds the pattern of notes received from input

float rev;                      //extracted number of revolutions from regex
float vel;                      //extracted number of velocity from regex
RawSerial pc(USBTX, USBRX);
char buffer[BUFLNTH];           //stores characters in this buffer from serial
string input(buffer);           //input is the string representation of the buffer
int i=0;                        //iterates buffer elements
bool Tbool=false;               //will turn off/on infitite while loop of play_repeat fuction
bool Rbool=false;               //will turn off/on infinite while loop of motor_run thread

//Initiliasing the following parameters needed for the motor_run tread
float initial_count;            //the targeted number of rotations
float count_big=0;              //keeps track of full rotations
float count_small=0;            //keeps track of fraction of a rotation
int APF;                        //60 degrees accuracy position
int APF_initial;                //initial 60 degrees accuracy position
float duty_cycle =1;
Timer time1; 
float current_count=0;          //current number of rotations
float current_count_old=0;      //previous value of the current number of rotations 
Timer music_time;               //this timer keeps track if the duration of the beat is finished


float revs_K =10;
float revs_Kp=15;
float revs_Kd=-0.9*100;
float revs_tf=0;
float revs_I =0;
float revs_Ki =1;

float vel_tf=0;
float vel_K =1;
float vel_error =0;
float vel_error_old =0;
float vel_Kp =1;
float targetvel =8;

float final_tf = 0;

float v=0;
float time_read;
float dy=0;
float dist_error=0;
float dist_timer_read=0;
Timer dist_timer;
int8_t  current_control =0;

int8_t orState = 0;                 //Rotot offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;

float time_automatic_tuning =0; 
Timer automatic_tuning;
float past_v;
float past_time=0;

double max_double(double a, double b){
    if(a>b){
        return a;
        }else{
            
            return b;
            }
    
    }

//Set a given drive state
void motorOut(int8_t driveState){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L = duty_cycle;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = duty_cycle;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = duty_cycle;
    if (driveOut & 0x20) L3H = 0;
    }

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}

//Returns inimum between two given numbers
inline float min(float a, float b){
    if(a<b){
        return a;
     }else{
        return b;
    }
}

//Returns maximum between two numbers
inline float max(float a, float b){
    if(a>b){
        return a;
     }else{
        return b;
    }
}

//Mapping of notes to the corresponding period
int note_to_period(string note){
    //map<string,int> dict{{"A^",1200},{"A",1140},{"A#",1070},{"B^",1070},{"B",1010},{"B#",955},{"C^",1010},{"C",955},{"C#",901},{"D^",901},{"D",851},{"D#",803},{"E^",803},{"E",758},{"E#",715},{"F^",758},{"F",715},{"F#",676},{"G^",676},{"G",638},{"G#",602}};
   // dict = {'A^':1200,'A':1140,'A#':1070,'B^':1070,'B':1010,'B#':955,'C^':1010,'C':955,'C#':901,'D^':901,'D':851,'D#':803,'E^':803,'E':758,'E#':715,'F^':758,'F':715,'F#':676,'G^':676,'G':638,'G#':602}
    //unordered_map<string, int> dict;
    map<string,int> dict;
    
    dict["A^"]=1200;
    dict["A"]=1140;
    dict["A#"]=1070;
    dict["B^"]=1070;
    dict["B"]=1010;
    dict["B#"]=955;
    dict["C^"]=1010;
    dict["C"]=955;
    dict["C#"]=901;
    dict["D^"]=901;
    dict["D"]=851;
    dict["D#"]=803;
    dict["E^"]=803;
    dict["E"]=758;
    dict["E#"]=715;
    dict["F^"]=758;
    dict["F"]=715;
    dict["F#"]=676;
    dict["G^"]=676;
    dict["G"]=638;
    dict["G#"]=602;
    return dict[note];
}
    
//Spin the motor at full speed and play the duration of a specific note
void play_motor(int beat){
        music_time.start();
        while (music_time.read()<0.4*beat) {
        intState = readRotorState();
        if (intState != intStateOld) {
            intStateOld = intState;
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
        }
        music_time.reset();
    
}
//A function that sets the periods by calling note_to_period function
//and runs the motor with the duration of each note
void tune(string note, int8_t beat){
    int per = note_to_period(note);     //get the mapping between note to period                         
    L1L.period_us(per);                 //Set the pitch  of the sound produced
    L2L.period_us(per);
    L3L.period_us(per);
    play_motor( beat);                  //run the motor and play the music
}
//a function which will repeat when it reaches the end of the (input) sequence of notes
void play_repeat(){
    int8_t j=0;                         //Tbool becomes false
    while (Tbool){                      //if a new serial input is received and infite loop breaks
        tune(vec_notes[j].note,vec_notes[j].beat);
        if (j==vec_notes.size()-1){ 
                j=0;
        }else
            j++;
    }
}

//-----------------------------------------------------------------------------    
// This function empties the buffer and the vec_notes vector when there is a new commnand
void emptybuffer(void){
    for (int8_t  k=0;k<BUFLNTH;k++) {
    buffer[k]='\0';
    }
    i=0;
    vec_notes.clear();
}
// A serial input interrupt to receive the input commmand
void callback(){                                           
    buffer[i]=pc.getc();    // Put the character received into the buffer, at place "i"
    i++;                    // Increment "i" to make ready for next interrupt
    Tbool=false;            // If there is a new serial input interrupt play_repeat
    Rbool=false;            // If there is a new serial input interrupt motor_run thread
} 
 
//------------------------------------------------------------------------------
//Extract the number of revolutions from  a serial input
//If the input does not follow the Regex syntax, 0 is returned
float extract_rev(string input){
    size_t found1=input.find("R"); //checks if an "R" character exists
    size_t found2=input.find("-"); //checks if an "-" character exists
    size_t found3=input.find("."); //checks if an "." character exists
    string number;
    string fract; 
    float rev;
    if(found1!=string::npos){
        if(found3!=string::npos){
            if((found3-found1-1<=4&&found2!=string::npos)||(found3-found1-1<=3&&found2==string::npos)&&(found3-found1-1!=0)){
                number=input.substr(found1+1,found3-found1-1);
                if(input.length()-found3<=3&&input.length()-found3!=1){
                    fract=input.substr(found3, string::npos-found3);
                    rev=atof((number.append(fract)).c_str());
                    return rev;
                }
                else{
                    return 0;           //wrong syntax
                }
            }
            else{
                return 0;               //wrong syntax
            }
                
        }
        else{
            if((input.length()-found1-1<=4&&found2!=string::npos)||(input.length()-found1-1<=3&&found2==string::npos)&&(input.length()-found1-1!=0)){
                    number=input.substr(found1+1,string::npos-found1-1);
                    rev=atof(number.c_str());
                    return rev;
            }
            else{
                    return 0;           //wrong syntax
            }
        }
    }
    else{
        return 0;                       //wrong syntax
    }
}

//-----------------------------------------------------------------------------  
//Extract the velocity from a serial input
//If the input does not follow the Regex syntax, 0 is returned  
float extract_vel(string input){
    size_t found1=input.find("V");    //checks if an "V" character exists
    size_t found2=input.find("-");    //checks if an "-" character exists
    size_t found3=input.find(".");    //checks if an "." character exists
    string number;
    string fract; 
    float vel;
    if(found1!=string::npos){
        if(found3!=string::npos){
            if(found3-found1-1<=3&&found3-found1-1!=0&&found2==string::npos){
                number=input.substr(found1+1,found3-found1-1);
                if(input.length()-found3<=4&&input.length()-found3!=1){
                    fract=input.substr(found3, string::npos-found3);
                    vel=atof((number.append(fract)).c_str());
                    return vel;
                }
                else{
                    return 0;       //wrong syntax
                }
            }
            else{
                return 0;           //wrong syntax
            }
                
        }
        else{
            if(input.length()-found1-1<=3&&input.length()-found1-1!=0&&found2==string::npos){
                    number=input.substr(found1+1,string::npos-found1-1);
                    vel=atof(number.c_str());
                    return vel;
            }
            else{
                    return 0;      //wrong syntax
            }
        }
    }
    else{
        return 0;                  //wrong syntax
    }   
    
}
//-----------------------------------------------------------------------------
//This function 
void slice(string &input,int8_t k){
    string note;
    int8_t  beat;
    duty_cycle=0.5;                           //Set duty_cyle =0.5
    note=input.substr(0,k);                   //Extract a note from input
    beat=input[k] - '0';                      //Transform char to integer
    note_ele newNote;                         //Create a new note_ele type variable
    newNote.note = note;                         
    newNote.beat = beat;
    vec_notes.push_back(newNote);             //Append the extracted note to vec_notes vector
    input=input.substr(k+1,input.length()-k); //Remove the extracted note from the input
}

//-----------------------------------------------------------------------------
//Finite State machine to extract correctly an input pattern of notes
int8_t  parse_note (string &input){
int8_t  state=1;

if (input.length()==1)
    return -1;

    for (int i=0;i<input.length();i++){
        switch (state){                 
            case 1: if ( (input[i]=='A')|| (input[i]=='B') || (input[i]=='C') || (input[i]=='D')|| (input[i]=='E') || (input[i]=='F')|| (input[i]=='G'))
                        state=2;           //if one of the above have been received move to state 2
                    else return -1;        //wrong syntax
                    break;
            case 2: if ( (input[i]=='^')|| (input[i]=='#'))
                        state=3;            //if one of the above have been received move to state 3
                    else if (((input[i] - '0')<9)&&((input[i] - '0')>0)){
                        state=1;            //if one of the above have been received move to state 1
                        slice(input,i);
                        i=-1;   
                    }
                    else return -1;         //wrong syntax
                    break;
            case 3: if (((input[i] - '0')<9)&&((input[i] - '0')>0)){
                    state=1;                //if one of the above have been received move to state 1
                    slice(input,i);
                    i=-1;
                    }
                    else return -1;         //wrong syntax
                    break;
        }
        
    }
        return 0;                          //wrong syntax
}
void automatic_tuning_function(){

    float max_a=0;
    float a=0;
    
    automatic_tuning.start();
    duty_cycle =1;
    
     while (time_automatic_tuning<5) {

        intState = readRotorState();
        APF = I1 + 2*I2 + 4*I3;

        if (intState != intStateOld) {
            
            time_automatic_tuning+= automatic_tuning.read();
            dy=current_count-current_count_old;
            v=dy/ (time_automatic_tuning-past_time);
            a= (v-past_v)/(time_automatic_tuning-past_time);
            current_count_old=current_count; 
            vel_error =(targetvel-v);
            vel_error_old =vel_error;
            past_v = v;
            automatic_tuning.reset();
            automatic_tuning.start();
            
            past_time=time_automatic_tuning;
            current_count = count_big+count_small/117;
            duty_cycle =1;
            if(a<500 && a>-500){
                max_a=max_double(a,max_a);
            }   
            
            if(APF==APF_initial){
                    count_big++;
                    count_small=0;
              }
            intStateOld = intState;
            
            motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        }
        
        //  printf("max_a = %f,v =%f ,a =%f,time = %f, dc=%f\n", max_a,v,a,time_automatic_tuning,duty_cycle);   
    }
    duty_cycle = 0;
    //printf("out \n");
    //printf("max_a =%f \n",max_a );
    revs_Kd=-(45000/max_a);
    //printf("Kd =%f \n", revs_Kd);
}
    
//------------------------------------------------------------------------------
void motor_run(){

        while (Rbool) {
        current_count = count_big+count_small/117;
        dist_timer_read=dist_timer.read();
    
        dy=current_count-current_count_old;
        v=dy/ dist_timer_read;
        vel_error =(targetvel -v);
        vel_error_old =vel_error;
        current_count_old=current_count;
        dist_timer.reset();
        dist_timer.start();
        
        dist_error = (initial_count-current_count);
        revs_I+=revs_Ki*dist_error*dist_timer_read;
        if(revs_I>2){revs_I=2;}
        else if(revs_I<-2){revs_I=-2;}

        revs_tf=revs_K*( revs_Kp* dist_error + revs_Kd*v);
        vel_tf = vel_K*(vel_Kp*vel_error);

        if(current_control ==1){
            final_tf =revs_tf;
        }else if(current_control ==2){
            final_tf=vel_tf;    
        }else if(current_control ==3){
            final_tf = min(revs_tf,vel_tf);
            }

        duty_cycle =  min(1 , final_tf );
        
        if(duty_cycle < 0){
            duty_cycle =0.1;
            }

        intState = readRotorState();
        APF = I1 + 2*I2 + 4*I3;

        if (intState != intStateOld) {
            if(APF==APF_initial){
                    count_big++;
                    count_small=0;
                }else{
                    //printf("%f ", current_count);

                    }
            intStateOld = intState;
            if(initial_count>current_count){
                motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
            }
        }else if(duty_cycle >0.75){
                motorOut((intState-orState+lead+6)%6);
            }
        Thread::wait(10);
    }
}

void CHAinterruptcall(){
    count_small++;
}
//Main
int main() {
    int8_t foundR; 
    int8_t foundV; 
    int8_t foundT; 
    int8_t foundA; 
    string inputV;   
    string inputR;  
    
    Thread t1(osPriorityHigh,512);

    RawSerial pc(SERIAL_TX, SERIAL_RX);
    
    //Initialise the serial port
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    APF_initial = I1 + 2*I2 + 4*I3;
   
    CHAinterrupt.rise(&CHAinterruptcall);
    pc.attach(&callback);
    while(1){
        Thread::wait(100);
        //printf("cc = %f,v =%f,cu_cont =%i,r_tf =%f ,v_tf= %f,f_tf=%f,dc=%f \n", current_count,v, current_control, revs_tf,vel_tf,final_tf,duty_cycle);
       if(buffer[0] != '\0'){
            wait(0.5);//until it receives the whole serial input
            input=buffer; //transform buffer to a string
            emptybuffer();
            pc.printf("Buffer input : %s\n\r ",input);
            foundR=input.find("R");
            foundV=input.find("V");
            foundT=input.find("T");
            foundA=input.find("A");
            if (foundA!=-1){
                motorHome();
                automatic_tuning_function();
                printf("Kd = %f \n", revs_Kd);
               
                }
            if (foundV!=-1){ //Start extracting the velocity
                inputV=input.substr(foundV,input.length()-foundV);
                vel=extract_vel(inputV); //here call velocity function thread
                if (vel==0) //Error in syntax
                    pc.printf("Error input V \n\r");
                else{       //Syntax ok
                    pc.printf("Finished V \n\r");
                    }
            }

            pc.printf("%f \n\r" ,vel);
            if (foundR!=-1){ 
                inputR=input.substr(foundR,foundV);
                rev=extract_rev(inputR); //here call revolution function thread
                if (rev==0) //Error in syntax
                    pc.printf("Error input R \n\r");
                else{       //Syntax ok
                    pc.printf("Finished R \n\r");
                    }
            }
            pc.printf("%f \n\r" ,rev);
            //check in which direction must perform the revolutions
            if (rev<0){
                lead=-2;
                rev=-rev;
            }else{
                lead=2;
            }
            motorHome();
            
            if((rev!=0)&&(vel!=0)){ 
                //Initiliase the paramters to start performing a specific number of revolutions without
                //exeeding a specific velocity
                Rbool=true;
                count_big=0;
                count_small=0;
                current_count=0;
                current_count_old=0;
                initial_count=rev;
                targetvel=vel;
                rev=0;
                vel=0;
                current_control =3;
                dist_timer.start();
                t1.start(callback(motor_run));
                }
            else if(rev!=0){
                //Initiliase the paramters to start performing a specific number of revolutions
                pc.printf(" Rrrrrr \n\r");
                Rbool=true;
                count_big=0;
                count_small=0;
                current_count=0;
                current_count_old=0;
                initial_count=rev;
                rev=0;
                current_control =1;
                dist_timer.start();
                t1.start(callback(motor_run));
                }
            else if(vel!=0){
                //Initiliase the paramters to start performing a specific velocity
                Rbool=true;
                count_big=0;
                count_small=0;
                current_count=0;
                current_count_old=0;
                targetvel=vel;
                vel=0;
                current_control =2;
                dist_timer.start();
                t1.start(callback(motor_run));
                }
                
            if (foundT!=-1){
                
                input=input.substr(foundT+1,input.length()-1);
                 if (parse_note(input)==-1) //Syntax Error
                    pc.printf("Error input T \n\r");
                else{                       //Syntax ok
                     pc.printf("Finished T \n\r");
                     //Repeatedly play the input pattern
                     Tbool=true;
                     play_repeat();
                     }
            }
        if ((foundT==-1)&&(foundV==-1)&&(foundR==-1)){
            pc.printf("Invalid input \n\r"); //Invalid syntax
            }
      
        }
    }
}

