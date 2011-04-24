/* Sous-vide controller! */

#include "PID_Beta6.h"
#include "OneWire.h"
#include "DallasTemperature.h"

double setpoint, pid_output, temperature, analog_temp, onewire_temp;

#define ONEWIRE_TEMP_PIN A1
#define ANALOG_TEMP_PIN A2
#define HEATER_PIN A0

#define PID_FREQ 100 // 10 Hz
#define HEATER_FREQ 1250

#define P_PARAM 60
#define I_PARAM 0.05
#define D_PARAM 10
#define PROP_BAND 0.8 // degrees

// setup onewire
OneWire oneWire(ONEWIRE_TEMP_PIN);
DallasTemperature sensors(&oneWire);
PID pid(&temperature, &pid_output, &setpoint, P_PARAM, I_PARAM, D_PARAM);

unsigned long serialTime, now, duty_start, last_temp_read = 0;

// initial heater state
int heater_state = LOW;

void setup() {
  //initialize the serial link with processing
  Serial.begin(9600);
  
  // setup the onewire sensor
  sensors.setWaitForConversion(false);
  sensors.begin();
  // initial request
  sensors.requestTemperatures();
  
  // set the reference voltage for the analog sensor
  analogReference(INTERNAL);
  
  // do an initial read
  read_temperature(1);
  
  // set an initial setpoint (room temperature)
  setpoint = 33;

  //turn the PID on
  pid.SetSampleTime(PID_FREQ);
  pid.SetMode(AUTO);
  pid.SetOutputLimits(0, HEATER_FREQ);
  
  // init heater
  pinMode(HEATER_PIN, OUTPUT);  
  
  // and initialize the duty cycle
  last_temp_read = duty_start = millis();
}

// average the two sensors
void read_temperature(int first) {
  read_analog_temp();
  read_digital_temp();
  if (first)
    temperature = (onewire_temp + analog_temp)/2;
  else
    temperature = ((onewire_temp + analog_temp)/2 + 199.0*temperature)/200.0;
}


/* conversion for the AD22100KTZ-ND sensor (not used)
void read_analog_temp() {
  analog_temp = (((double)analogRead(ANALOG_TEMP_PIN))/1024.0*5 - 0.25)/0.0225 - 50.0;
}*/

/* conversion for the LM35 sensor */
void read_analog_temp() {
  analog_temp = map(analogRead(ANALOG_TEMP_PIN), 0, 1023, 0, 1100)/10.0 - 1.5;
}

void read_digital_temp() {
  if (now - last_temp_read < 750) return;
  last_temp_read = now;
  onewire_temp = sensors.getTempCByIndex(0) - 1.0;
  sensors.requestTemperatures();
}

void do_output() {
  // turn on if we are still within our duty cycle
  if ((now - duty_start) > HEATER_FREQ)
    duty_start += HEATER_FREQ;
  
  if (pid_output > (now - duty_start)) {
    digitalWrite(HEATER_PIN, HIGH);
    //Serial.print("HIGH\n");
  } else {
    digitalWrite(HEATER_PIN, LOW);
    //Serial.print("LOW!\n");
  }
}

void loop() {
  now = millis();
  read_temperature(0);
  
  // do the PID loop if it's within the proportional band
  if (setpoint - temperature < PROP_BAND && temperature < setpoint)
    pid.Compute();
  else if (setpoint - temperature > PROP_BAND)
    pid_output = HEATER_FREQ;
  else
    pid_output = 0;
  
  // and do the output
  do_output();
  
  //send-receive with processing if it's time
  if(now > serialTime) {
    //Serial.println(onewire_temp);
    //Serial.println(analog_temp);
    //Serial.println(pid_output);
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }  
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1-4: float setpoint
//  5-8: float input
//  9-12: float output  
//  13-16: float P_Param
//  17-20: float I_Param
//  21-24: float D_Param
void SerialReceive() {
  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  
  while(Serial.available()&&index<25)
  {
    if(index==0) Auto_Man = Serial.read();
    else foo.asBytes[index-1] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==25  && (Auto_Man==0 || Auto_Man==1))
  {
    setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    
    if(Auto_Man==0) pid.SetMode(MANUAL);// * set the controller mode
    else pid.SetMode(AUTO);             //
    
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      pid_output = double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    pid.SetTunings(p, i, d);            //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}



// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(setpoint);   
  Serial.print(" ");
  Serial.print(temperature);   
  Serial.print(" ");
  Serial.print(pid_output);   
  Serial.print(" ");
  Serial.print(pid.GetP_Param());   
  Serial.print(" ");
  Serial.print(pid.GetI_Param());   
  Serial.print(" ");
  Serial.print(pid.GetD_Param());   
  Serial.print(" ");
  if(pid.GetMode()==AUTO) Serial.println("Automatic");
  else Serial.println("Manual");  
}


