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
const int8_t lead = -2;  //2 for forwards, -2 for backwards

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

struct note_ele {
    string note;
    int beat;
};

vector<note_ele> vec_notes(16);

float rev;  //HERE
float vel; //HERE
RawSerial pc(USBTX, USBRX);
char buffer[BUFLNTH]; //HERE
string input(buffer); //HERE
int i=0; //HERE
Thread t3;//HERE
bool Tbool=false;
bool Rbool=false;
bool Vbool=false;

float initial_count=30;
float count_big=0;
float count_small=0;
int APF;
int APF_initial;
float duty_cycle =1;
Timer time1;
float current_count=0;
float current_count_old=0;




int8_t orState = 0;    //Rotot offset at motor state 0
int8_t intState = 0;
int8_t intStateOld = 0;

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
    wait(1.0);

    //Get the rotor state
    return readRotorState();
}


inline float min(float a, float b){
    if(a<b){
        return a;
     }else{
        return b;
    }
}

inline float max(float a, float b){
    if(a>b){
        return a;
     }else{
        return b;
    }
}

int note_to_period(string note){
    map<string,int> dict; 
    dict["A^"]=602   
    dict["A"]=568;
    dict["A#"]=536;
    dict["B^"]=536;
    dict["B"]=506;
    dict["B#"]=477;
    dict["C^"]=506;
    dict["C"]=477;
    dict["C#"]=451;
    dict["D^"]=451;
    dict["D"]=426;
    dict["D#"]=402;
    dict["E^"]=402;
    dict["E"]=379;
    dict["E#"]=358;
    dict["F^"]=379;
    dict["F"]=358;
    dict["F#"]=338;
    dict["G^"]=338;
    dict["G"]=319;
    dict["G#"]=301;
    return dict[note];
}

Timer music_time;
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
void tune(string note, int beat){
    int per = note_to_period(note);
    L1L.period_us(per); 
    L2L.period_us(per);
    L3L.period_us(per);
    play_motor( beat);
    //wait(0.4*beat);
}
void play_repeat(){
    int j=0;
    while (Tbool){
        tune(vec_notes[j].note,vec_notes[j].beat);
        if (j==vec_notes.size()-1){ 
                j=0;
        }else
            j++;
    }
}
//HERE
//-----------------------------------------------------------------------------    

void emptybuffer(void){
    for (int k=0;k<BUFLNTH;k++) {
    buffer[k]='\0';
    }
    i=0;
    vec_notes.clear();
}

//HERE
void callback(){                                           
    buffer[i]=pc.getc();    // Put the character received into the buffer, at place "i"
    i++;                    // Increment "i" to make ready for next interrupt
    Tbool=false;
    Rbool=false;
    Vbool=false;
} 
 //HERE
//------------------------------------------------------------------------------
float extract_rev(string input){
    size_t found1=input.find("R");
    size_t found2=input.find("-");
    size_t found3=input.find(".");
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
                    return 0;
                }
            }
            else{
                return 0;
            }
                
        }
        else{
            if((input.length()-found1-1<=4&&found2!=string::npos)||(input.length()-found1-1<=3&&found2==string::npos)&&(input.length()-found1-1!=0)){
                
                    number=input.substr(found1+1,string::npos-found1-1);
                    rev=atof(number.c_str());
                    return rev;
            }
            else{
                    return 0;
            }
        }
    }
    else{
        return 0;
    }
}

//HERE
//-----------------------------------------------------------------------------    
float extract_vel(string input){
    size_t found1=input.find("V");
    size_t found2=input.find("-");
    size_t found3=input.find(".");
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
                    return 0;
                }
            }
            else{
                return 0;
            }
                
        }
        else{
            if(input.length()-found1-1<=3&&input.length()-found1-1!=0&&found2==string::npos){
                    number=input.substr(found1+1,string::npos-found1-1);
                    vel=atof(number.c_str());
                    return vel;
            }
            else{
                    return 0;
            }
        }
    }
    else{
        return 0;
    }   
    
}
 

//-----------------------------------------------------------------------------
//HERE
void slice(string &input,int k){
    string note;
    int beat;
    duty_cycle=0.5;
    note=input.substr(0,k);
    beat=input[k] - '0';
    note_ele newNote;
    newNote.note = note;
    newNote.beat = beat;
    vec_notes.push_back(newNote);
    input=input.substr(k+1,input.length()-k);
}




//-----------------------------------------------------------------------------
//HERE
int parse_note (string &input){
int state=1;

if (input.length()==1)
    return -1;

    for (int i=0;i<input.length();i++){
        switch (state){
            case 1: if ( (input[i]=='A')|| (input[i]=='B') || (input[i]=='C') || (input[i]=='D')|| (input[i]=='E') || (input[i]=='F')|| (input[i]=='G'))
                        state=2;
                    else return -1;
                    break;
            case 2: if ( (input[i]=='^')|| (input[i]=='#'))
                        state=3;
                    else if (((input[i] - '0')<9)&&((input[i] - '0')>0)){
                        state=1;
                        slice(input,i);
                        i=-1;   
                    }
                    else return -1;
                    break;
            case 3: if (((input[i] - '0')<9)&&((input[i] - '0')>0)){
                    state=1;
                    slice(input,i);
                    i=-1;
                    }
                    else return -1;
                    break;
        }
        
    }
        return 0;
}
    
//------------------------------------------------------------------------------
   

float K =5;
float Kp=10;
float Kd=-0.9;
float tf=1;
float v=0;
float time_read;
float dy=0;
float dist_error=0;
float I =0;
float Ki =1;

void motor_run(){

        while (Rbool) {
        current_count = count_big+count_small/117;
        //duty_cycle = min( 0.9, (initial_count-current_count)/initial_count+0.4 );
        
        dist_error = (initial_count-current_count);
        I+=Ki*dist_error*1;
        if(I>2){I=2;}
        else if(I<-2){I=-2;}

        tf=K*( Kp* dist_error + Kd*v);
        duty_cycle =  min(0.8 , tf );

        if(duty_cycle < 0){
            duty_cycle =0.1;
            }


        intState = readRotorState();
        APF = I1 + 2*I2 + 4*I3;

        if (intState != intStateOld) {


            if(APF==APF_initial){
                    count_big++;
                    count_small=0;

                    //printf(" qwer");
                }else{
                    //printf("%f ", current_count);

                    }
           // count_big++;
            //printf('%x ', count_big);
            intStateOld = intState;
            if(initial_count>current_count){
                //printf("%x  \r", count_big);
                motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
            }
        }else if(duty_cycle >0.75){
                motorOut((intState-orState+lead+6)%6); //+6 to make sur
            }
        Thread::wait(10);
    }
}

void CHAinterruptcall(){
    count_small++;
    //printf("%x  ", count2);

    }
void measure_speed(){
    while(1){
            dy=current_count-current_count_old;
            v=1000*dy/ (0.1);
            current_count_old=current_count;
            wait(0.1);
            Thread::wait(10);
        }

    }
//Main
int main() {
    int foundR;  //HERE
    int foundV; //HERE 
    int foundT;  //HERE 
    string inputV;  //HERE 
    string inputR;  //HERE  
    
    Thread t1(osPriorityHigh,512);
    Thread t2(osPriorityHigh,512);
    RawSerial pc(SERIAL_TX, SERIAL_RX); //HERE
    
    //Initialise the serial port
    pc.printf("Hello\n\r");

    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    APF_initial = I1 + 2*I2 + 4*I3;
   

    //t1.start(callback(motor_run));
    //t2.start(callback(measure_speed));
    CHAinterrupt.rise(&CHAinterruptcall);
    pc.attach(&callback);
    while(1){
        Thread::wait(100);
        printf("cc = %f,v =%f \n", current_count,v);
       if(buffer[0] != '\0'){
            wait(0.5);//until it recieve whole string
            input=buffer; //transform buffer to a string
            emptybuffer();
            pc.printf("Buffer input : %s\n\r ",input);
            foundR=input.find("R");
            foundV=input.find("V");
            foundT=input.find("T");
            if (foundV!=-1){
                inputV=input.substr(foundV,input.length()-foundV);
                vel=extract_vel(inputV); //here call velocity function thread
                if (vel==0)
                    pc.printf("Error input V \n\r");
                else
                    pc.printf("Finished V \n\r");
            }

            pc.printf("%f \n\r" ,vel);
            if (foundR!=-1){
                inputR=input.substr(foundR,foundV);
                rev=extract_rev(inputR); //here call revolution function thread
                if (rev==0)
                    pc.printf("Error input R \n\r");
                else{
                    pc.printf("Finished R \n\r");
                    Rbool=true;
                    count_big=0;
                    count_small=0;
                    duty_cycle =1;
                    current_count=0;
                    current_count_old=0;
                    initial_count=rev;
                    t1.start(callback(motor_run));
                    t2.start(callback(measure_speed));
                    }
            }
            pc.printf("%f \n\r" ,rev);
            if (foundT!=-1){
                
                input=input.substr(foundT+1,input.length()-1);
                 if (parse_note(input)==-1)
                    pc.printf("Error input T \n\r");
                else{
                     pc.printf("Finished T \n\r");
                     Tbool=true;
                     play_repeat();
                     }
            }
        if ((foundT==-1)&&(foundV==-1)&&(foundR==-1)){
            pc.printf("Invalind input \n\r");
            }
      
        }
    }
}

