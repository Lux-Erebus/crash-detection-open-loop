

/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   If you have other time consuming code running (i.e. a graphical LCD), consider calling update() from an interrupt routine,
   see example file "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/
#include <SPI.h>
//#include <FastLED.h>
#include "Arduino.h"
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

//pins:
#define HX711_dout_1 A0//mcu > HX711 dout pin > pin A1
#define HX711_sck_1 A1//mcu > HX711 sck pin > pin A2
#define HX711_dout_2 A2 //mcu > HX711 dout pin > pin A3
#define HX711_sck_2 A3 //mcu > HX711 sck pin > pin A4
#define CRASH D10

//const int CRASH=14; //gpio pin 
//const int LED_PIN=2;
const int rms_size=10;//size of a window
const int rms_factor=100;//number of windows

//HX711 constructor: 
HX711_ADC LoadCell_1(HX711_dout_1,HX711_sck_1);
HX711_ADC LoadCell_2(HX711_dout_2,HX711_sck_2);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
//const int num_states=5;
//int counter[num_states];
//CRGB led_array[num_states];
//int counter_lim=270;
//int state=50;
int tolerance=10; //accounts for loadcell inaccuracy (not needed with good standard deviation)
const float calval_1=555.89;// calibration value for A1, A2
const float calval_2=-448.55; //calibration values for A3, A4
const int crash_val=250; //set an experimental threshold for crash value
float avg=0; //calculates average of the baseline
float std_dev=0; //standard deviation of baseline
float rms=0;
const int baseline_lgth=1000;
float baseline[baseline_lgth]; //collects signal baseline
int phase=0; //finite state info
int counter=0; //keep track of amount of data collected for baseline
float baseline_rms[rms_factor];
int window_counter=0;
float window[rms_size];
float threshold=0;
int crashed=0;
const float threshold_c=120;



template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order+1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order+1]; // Raw values
    float y[order+1]; // Filtered values

  public:  
    LowPass(float f0, float fs, bool adaptive){
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.
      
      omega0 = 6.28318530718*f0;
      dt = 1.0/fs;
      adapt = adaptive;
      tn1 = -dt;
      for(int k = 0; k < order+1; k++){
        x[k] = 0;
        y[k] = 0;        
      }
      setCoef();
    }

    void setCoef(){
      if(adapt){
        float t = micros()/1.0e6;
        dt = t - tn1;
        tn1 = t;
      }
      
      float alpha = omega0*dt;
      if(order==1){
        a[0] = -(alpha - 2.0)/(alpha+2.0);
        b[0] = alpha/(alpha+2.0);
        b[1] = alpha/(alpha+2.0);        
      }
      if(order==2){
        float alphaSq = alpha*alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2];
        b[0] = alphaSq/D;
        b[1] = 2*b[0];
        b[2] = b[0];
        a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D;
        a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D;      
      }
    }

    float filt(float xn){
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if(adapt){
        setCoef(); // Update coefficients if necessary      
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for(int k = 0; k < order; k++){
        y[0] += a[k]*y[k+1] + b[k]*x[k];
      }
      y[0] += b[order]*x[order];

      // Save the historical values
      for(int k = order; k > 0; k--){
        y[k] = y[k-1];
        x[k] = x[k-1];
      }
  
      // Return the filtered value    
      return y[0];
    }
};

// Filter instance
LowPass<2> lp1(10,90,0);
LowPass<2> lp2(10,90,0);

void setup() {
  Serial.begin(9600); delay(10);
  Serial.println();
  Serial.println("Starting...");
//  FastLED.addLeds<WS2811, LED_PIN>(led_array, num_states);
//  pinMode(LED_PIN, OUTPUT);
  pinMode(CRASH, OUTPUT);
  digitalWrite(CRASH, LOW);
//  pinMode(LED_BUILTIN, OUTPUT);
//  digitalWrite(LED_BUILTIN, LOW); //turn on LED
  phase=0;


  LoadCell_1.begin();
  LoadCell_2.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive


  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell_1.start(stabilizingtime, _tare);
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell_1.setCalFactor(calval_1); // set calibration value (float)
    //Serial.println("Startup is complete");
  
  }
  LoadCell_2.start(stabilizingtime, _tare);
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell_2.setCalFactor(calval_2); // set calibration value (float)
    //Serial.println("Startup is complete");
  
  }
//  for(int i=0;i<num_states;i++)
//  {
//    counter[i]=0;
//    led_array[i]=CRGB(0,0,0);
//  }
//   FastLED.show();
}

void loop() {
  static boolean newDataReady_1 = 0;
  static boolean newDataReady_2 = 0;
  float save1=0;
  float save2=0;
  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady_1 = true;
  if (LoadCell_2.update()) newDataReady_2 = true;
  // get smoothed value from the dataset:
  if(newDataReady_1)
  {
    float i1=LoadCell_1.getData();
    save1=lp1.filt(i1);
    newDataReady_1=0;
  }
  if(newDataReady_2)
  {
    float i2=LoadCell_2.getData();
    save2=lp2.filt(i2);
    float tot=save1+save2;
    float ser=tot*100;  
    int val=int(ser);
    Serial.print(tot);
    Serial.write(13);
    Serial.write(10);
    newDataReady_2=0;
    if(phase==0)
    baseliner(tot);
    if(phase==1)
    analysis();
    if(phase==2)
    crash_detect(tot);
    if(crashed==1)
    {
      digitalWrite(CRASH,HIGH);
      Serial.println("CRASHED");
      delay(500);
      digitalWrite(CRASH, LOW);
      crashed=0;
    }
   
  }
 
  // receive command from serial terminal, send 't' to initiate tare operation:
  // check if last tare operation is complete:
//  if (LoadCell.getTareStatus() == true) {
//    Serial.println("Tare complete");
//  } 
}

//void thresholder(int val2)
//{
//  int threshold;
//  val2=abs(val2);
//  for(int i=0;i<num_states;i++)
//  {
//    threshold=(i+1)*state-tolerance;
//    if(val2>threshold)
//    {
//      led_array[i]=CRGB(50,255,255);
//      counter[i]=0;
//    }
//    else
//    {
//      counter[i]+=1;
//    }
//    if(counter[i]>counter_lim)
//    {
//      counter[i]=0;
//      led_array[i]=CRGB(0,0,0);
//    }
//    
//  }
//  FastLED.show();
//}

bool crashdetection(int val2)
{
 val2=abs(val2);
 if(val2>crash_val)
 return true;
 else
 return false;
}

void baseliner(int val)
{
//  if(counter%45==0)//blink LED every 0.5s
//  digitalWrite(LED_BUILTIN,LOW);
//  if(counter%90==0)
//  digitalWrite(LED_BUILTIN,HIGH);
  if(counter<1000)//gather baseline data for 11s
  baseline[counter]=val;
  counter++;
  if(counter>=1000)
  {
    phase=1;
    counter=0;
  }
}
void analysis()
{
  
//  digitalWrite(LED_BUILTIN, HIGH); // turn off LED during signal analysis
  for(int i=0;i<rms_factor;i++)
  {
    rms=0;
    for(int j=0;j<rms_factor;j++)
    {
      int index=i*rms_size+j;
      rms=rms+baseline[index]*baseline[index];
    }
    baseline_rms[i]=sqrt(rms/rms_size);
  }
  for(int k=0;k<rms_factor;k++)
  {
    avg=avg+baseline_rms[k];
  }
  avg=avg/rms_factor;
  for(int m=0;m<rms_factor; m++)
  {
    std_dev=std_dev+(baseline_rms[m]-avg)*(baseline_rms[m]-avg);
  }
  std_dev=sqrt(std_dev/rms_factor);
  threshold=avg+6*std_dev+threshold_c; //set threshold
  phase=2;
}

void crash_detect(float tot)
{
  window[window_counter]=tot;
  window_counter++;
  if(window_counter==rms_size)//find rms after window collected
  {
    window_counter=0;
    rms=0;
    for(int i=0;i<rms_size;i++)
    {
      rms=rms+window[i]*window[i];
    }
    rms=sqrt(rms/rms_size);
    Serial.println("rms value:");
    Serial.print(rms);
    Serial.println("threshold:");
    Serial.print(threshold);
    if(rms>threshold)
    {
      crashed=1;
//      digitalWrite(LED_BUILTIN,LOW);//turn on LED if detected
    } 
    else
    {
//      digitalWrite(LED_BUILTIN, HIGH); //keep LED off otherwise   
    }
  }
}
