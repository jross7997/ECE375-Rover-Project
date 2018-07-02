//Abedallah Abedrabbah, Justin Ross, and Zack Salvador
// ECE375 Final Prjects
// This program allows the robot to move on a pre determined track, there are 2 different places for obstacles, 
//if it senses any, it will turn out of the way. At the end of the track, it needs to find
//a free parking spot
//I certify that this program works according to specification
#include <hidef.h>      /* common defines and macros */
#include <mc9s12dg256.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12dg256b"
#include "main_asm.h" /* interface to the assembly module */
//-------------------------------------------------------------------------//
void Update (void);
void stop(void);
void PulseOut(void);
void getDist(void);
void Sensor(void);
void PAC_init(void);
void move_forward(int n);
void move_back(int n);
void turn_right(int n);
void turn_left(int n);
void shortPark(void);
void longPark(void);
void finished (void);
void basiclineSensor(void);
void lineSensor(void);
void generalPark (void);
//-------------------------------------------------------------------------//
int mid,left,right;
int totalcount =0;
int sonarCount =0;
int shortPath = 0;
int firstcount =0;
int longPath =0;
//-------------------------------------------------------------------------//
void interrupt 9 handler1(){
  HILOtimes1();
}
//-------------------------------------------------------------------------//
void Update (void){

			mid = ad0conv(2);     //mid
			mid = mid >>1;
			left = ad0conv(1);   //left
			left = left >>1;
			right = ad0conv(6);    //right
			right = right >>1;	
}
//-------------------------------------------------------------------------//
//Stop
void stop(void){
    set_servo76(4500);   //Sends a 1.5ms pulsewidth to the servos,
   set_servo54(4500);    //which causes them to stop moving 
}
//-----------------------------------------------------------------------------//
//Sends a Pulse Out 
void PulseOut(void){
 int time, flag;       //initialize variables
 DDRT |= 0x08;         // set port 3 as output
 PTT |= 0x08;
 TCTL2 = 0x40;        //set pin to toggle
 TIOS |= 0x08;        //port 3 is now output compare
 TSCR1_TEN = 1;       //enable timer counter
 time = TCNT;
 
 TFLG1 |= 0x08;        //clears the flag
 time += 23880;        //pin is high for 20 micro seconds
 TC3 = time;
 flag = TFLG1 & 0x08;
 while(flag ==0){      //wait until the TFLG1 is thrown
  flag = TFLG1 & 0x08;
}

TFLG1 |= 0x08;           //clear the flag
time = time + 120;       //wait 5 micri seconds
TC3 = time;
 flag = TFLG1 & 0x08;
 while(flag ==0){         //wait until the TFLG1 is thrown
  flag = TFLG1 & 0x08;
  }
}
//----------------------------------------------------------------------------//
//Gets the Distance 
void getDist(void){

 int hitime;
 
 HILO1_init();                   //initialize the high low time                              //checks 5 times if something is nearby
  do{    
    hitime = get_HI_time1(); 
    if (hitime > 300 && hitime < 2000) {		// If Object was detected stop
      sonarCount++;
      if(sonarCount >=1  && totalcount < 1000) {
        turn_left(75);
      } else if(sonarCount >= 1 && totalcount > 1000){
        shortPark();
      } else{
        stop();
      }    
    }
  }while(hitime > 300 && hitime < 2000);
}


//----------------------------------------------------------------------------//
//Concatenates the two sensor methods into one, easy to call method
void Sensor(void){                //concatenates the two methods into one method
  PulseOut();
  getDist();
}

int pcount;
//-------------------------------------------------------------------------//
//Initialized the Pulse Accumulator System
void PAC_init(void) {
DDRT = DDRT & 0x7E;       // Pin 7 & 0 of port T is input for pulses
PBCTL = 0x40;             // PAEN=1, PAMOD=0, PEDGE =0, No Interrupts
PACTL = 0x40;             // PAEN=1, PAMOD=0, PEDGE =0, No Interrupts
}
void servo_init(void){
 servo54_init();          //Pin 5 of Port P is enabled to drive the servo
servo76_init();           //Pin 7 of Port P is enabled to drive the servo
}
//-------------------------------------------------------------------------//
//Move Forward
void move_forward(int n){
   PACN32 = 0;             //makes sure the pulse counter is set to zero
pcount=PACN32;
Sensor();
  while(pcount<n){        // for how ever many pulses the code calls for, 
   // Sensor();
   set_servo76(4850);     //sends a 1.67ms pulse to the left servo
   set_servo54(4850);     //sends a 1.7ms pulse to the right servo
      pcount=PACN32; 	      
    }
    stop();
  }
//-------------------------------------------------------------------------//
void move_back(int n){
  //PAC_init();
 // Sensor();             //starts the PAC system
  PACN32 = 0;             //makes sure the pulse counter is set to zero
pcount=PACN32;
  while(pcount<n){        // for how ever many pulses the code calls for, 
   // Sensor();
   set_servo76(3700);     //sends a 1.67ms pulse to the left servo
   set_servo54(3700);     //sends a 1.7ms pulse to the right servo
      pcount=PACN32; 	      
  }
  
  stop();
  
}

//--------------------------------------------------------------------------//
//Turn Right
void turn_right(int n){
 // PAC_init();             //starts the PAC system
  PACN32 = 0;             //makes sure the pulse counter is set to zero
  pcount=PACN32;
    while(pcount<n){      // for how ever many pulses the code calls for,
      pcount=PACN32;      // the servos will move at opposite rates causing a right turn
      set_servo76(5000);  //sends a 1.67ms pulsewidth to the left servo
      set_servo54(4000);
      Update();
    }               //this combination of the two motors moving in opposite directions
                     //causes a right turn
  stop();      //stops the motors  
}

//---------------------------------------------------------------------------//
//Turn Left
void turn_left(int n){
  //PAC_init();             //starts the PAC system
  PACN32 = 0;             //makes sure the pulse counter is set to zero
  pcount=PACN32;
    while(pcount<n){      // for how ever many pulses the code calls for,
      pcount=PACN32;      // the servos will move at opposite rates causing a left turn
      set_servo76(4000);  //sends a 1.2s pulsewidth to the left servo
      set_servo54(5000);  //sends a 1.67,s pulsewidth to the right servo
      Update();  
    }                     //this combination of the two motors moving in opposite directions
     stop();                     //causes a left turn
 /* set_servo76(4500);      //stops the servos
  set_servo54(4500);*/
}


//-------------------------------------------------------------------------//
int parkCount;
int hitime;
void shortPark(void){
move_back(8);
turn_left(95);
do{ 
    hitime = 0;
    parkCount = 0;
    move_forward(130);    
    turn_right(65);
    PulseOut();
    HILO1_init();
    hitime = get_HI_time1();
    if (hitime > 100 && hitime < 3200) { //was 2250
      turn_left(65);
    } else {
      while(mid<400){
      basiclineSensor();
      }
      parkCount = 1;
    }
  }while(parkCount < 1);
  finished();
}
//-------------------------------------------------------------------------//
int first = 0;
void longPark(void){
first = 0;
do{ 
    int hitime;  
    parkCount = 0;
    
    if(first ==0){
      move_forward(60);
      first++;
    }else{
    move_forward(130);
    }
    turn_left(75);
    PulseOut();
    HILO1_init();
    hitime = get_HI_time1();
    ms_delay(100);
    if (hitime > 100 && hitime < 3000) { //was 2250
      move_forward(5);
      turn_right(65);
    } else {
      while(mid<200){
      basiclineSensor();
      }
      parkCount = 1;
    }
  }while(parkCount < 1);
  finished();

} 
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

void basiclineSensor(void){
    
    Update();
			
			if(mid < 490 && left < 200 && right <490){           //if all white go forawrd
        set_servo76(4825);     //sends a 1.67ms pulse to the left servo
        set_servo54(4825);     //sends a 1.7ms pulse to the right servo
        Update();     
			}else if(mid < 490 && left > 200 && right <490){              //left sensor black, turn right
			 turn_right(1); 
			}else if(mid < 490 && left < 200 && right >490){             //right sensor black, turn left
			 turn_left(1);
			}else {    //park
			  stop();
			}
}
//-------------------------------------------------------------------------//
//-------------------------------------------------------------------------//

//Checks the line sensor
//Checks the line sensor
void lineSensor(void){
    int temp =0;
    Update();
		
		if(shortPath ==1){
			  while(1){
			   shortPark();
			  } 
		}else if(longPath ==1){
			  while(1){
			   longPark();
			  }
		}else if(mid < 490 && left < 200 && right <490){
		if(totalcount <2400){
		  
		      Sensor();
		      totalcount++;
			    set_servo76(4850);     //sends a 1.67ms pulse to the left servo
           set_servo54(4850);     //sends a 1.7ms pulse to the right servo         
		  }else{
		   longPark();
		    //finished();
		  }//Short Path is 1200-1500 Long Path 3200 - 	    
    }else if(mid > 200 && left < 200 && right <490){
			      stop();
			      ms_delay(10);     
		      if(firstcount ==0){ 
		        move_back(10);                   //if mid is black first time           
			      turn_right(60);
			      firstcount++;
			      
			    }else if (sonarCount == 0){
			       finished();                            //first parking condition
		     	}else if(sonarCount > 1 && firstcount ==1){
			      stop();                                     //parks
		     	}
    }else if(mid < 490 && left > 200 && right <490){              //left sensor black, turn right
			 turn_right(1);
    }else if(mid < 490 && left < 200 && right >490){             //right sensor black, turn left
			 turn_left(1);
    }else{
      stop();
    }
}

void finished(void){
 while(1){
  stop();
 }
}

//------------------------------------------------------------------------//
//Runs the course
void main(void) {
ad0_enable();
PAC_init();
servo_init();           //Initializes the servos
SW_enable();
while(1){
if(SW5_down()){
while(1){
 lineSensor();
        }
}
}
}