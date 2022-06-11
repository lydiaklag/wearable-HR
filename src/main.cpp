#include "SPI.h"
#include "Wire.h"
#include "max30105.h"
#include "IIRFilter.h"
#include <Arduino.h>


MAX30105 Sensor;

//***Define signal parameters
int samp_freq=25; //for each led
const int Num_Samples = 100;  //it stores 4 sec 

uint32_t gr_buffer[Num_Samples];
uint32_t ir_buffer[Num_Samples];
int filtered_gr_buffer[Num_Samples];
int filtered_ir_buffer[Num_Samples];
int ma_gr_buffer[Num_Samples];
int ma_ir_buffer[Num_Samples];

const int points_pr=4; //ask Maria if that is the number of peaks that I use to compute the HR
float PR[points_pr];
float Pulse_Rate_next=0, Pulse_Rate_previous=70;
int HR;

static int taskCore = 1;
static int taskCore_sp = 0;

//***Butterworth band-pass (0.5Hz-5Hz) 2nd order
const double b[] = {0.175087643672101,0,-0.350175287344202,0,0.175087643672101};
const double a[] = {1,-2.299055356038497,1.967497759984451,-0.874805556449481,0.219653983913695};

IIRFilter f(b, a);
double filtered_gr=0;
double filtered_ir=0;

int Moving_Average_Num = 2;
int Num_Points = 2*Moving_Average_Num+1;  //***5-point moving average filter
int Sum_Points;
void *ax;
int Sampling_Time= 2400; //2400ms = 4s
int flag_unplugged = 0; //0 means unplugged

SemaphoreHandle_t baton;

//regarding SpO2, initialising global variables
const int points_spo2 = 4; 
double SpO2_dc_ir[points_spo2]; //no need to initialise the arrays, that is done inside the SpO2 func
double SpO2_ac_ir[points_spo2];
double SpO2_dc_red[points_spo2];
double SpO2_ac_red[points_spo2];
double Oxy[points_spo2]; 
int ma_ir2_buffer[Num_Samples];
int ma_red_buffer[Num_Samples];
double SpO2_next; //maybe double
double SpO2_previous = 99;
double SpO2;
//declaring functions
void iir_ma_filter_for_hr();
int Find_Peak(int p1, int p2, int *x);
int Find_Mean(int p1, int p2, int *x);
int find_min_negative(int p1, int p2, int *x);
void convert_signal_to_positive(int p1, int p2, int *x, int point);
void ComputeHeartRate();
int readSamples();
void setup();
void loopHR(void * pvParameter);
void loop();
void ComputeSpO2(void * pvParameter);

void setup(){
  
  Serial.begin(9600);
 
  // Initialize sensor
  if (Sensor.begin() == false)
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  baton = xSemaphoreCreateMutex();
  byte ledBrightness = 0xDF; //Options: 0=Off to 255=50mA  --> DF=~44mA
  byte sampleAverage = 2;    //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3;          //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200;      //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;      //Options: 69, 118, 215, 411
  int adcRange = 2048;       //Options: 2048, 4096, 8192, 16384

  Sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  Sensor.setPulseAmplitudeRed(0xFF);  //if the value was 0, here we basically turn off the red LED, so green and IR LEDs are active
  Sensor.setPulseAmplitudeGreen(0);// trial to calculate HR using IR only, need to comment out the SpO2 task 
  //now that I changed the value to 0xFF it means that all the LEDs are on at the same time 
  xTaskCreatePinnedToCore(
  //xTaskCreate(  
    loopHR,
    "loop",
    10000,
    NULL,
    1,
    NULL,
    taskCore
  );   
  
  xTaskCreatePinnedToCore(
  //xTaskCreate(  
    ComputeSpO2,
    "loop",
    10000,
    NULL,
    1,
    NULL,
    taskCore_sp
  );
  
}


void loopHR(void * pvParameter) {
  for(;;){
    xSemaphoreTake( baton, portMAX_DELAY);
    double t_HR = millis();
    Serial.println(); Serial.print("t_HR: "); Serial.print(t_HR); Serial.println();
    int flag=readSamples();
    Serial.println();
    Serial.println("----------------------------------------------------------------------------------------------------");
    if (flag){          //***if sensor reads real data
      
      iir_ma_filter_for_hr();
      //trial for RED LED, next 2 lines
      int neg = find_min_negative(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_ir2_buffer);
      convert_signal_to_positive(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_ir2_buffer, neg);

    //the next 2 lines are for GREEN LED
      // int neg=find_min_negative(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer);
      // convert_signal_to_positive(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer, neg);
      ComputeHeartRate();
      
      Serial.print("NEW DATA--> ");
      Serial.print("HR: ");
      Serial.print(HR);
      Serial.println();
      flag_unplugged = 1; 
    }
    else{                 //***else if sensor is unplugged
      flag_unplugged = 0; 
      HR = 0; 
      Serial.print("HR : unplugged"); 
      Serial.println();
    }
    //not sure about reseting the flag_unplugged to 0, just trial and error 
    //semaphores will fix the sunchronisation issue
    //flag_unplugged = 0;
    xSemaphoreGive( baton );
    delay(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelete(NULL);
  //here I will put the info for the upload to firebase 
  
}

void iir_ma_filter_for_hr(){
  
  for (int i=0;i<Num_Samples;i++){
    
    // filtered_gr = f.filter(double(gr_buffer[i])); //green
    // filtered_gr_buffer[i] = round(filtered_gr);  //green
    filtered_ir = f.filter(double(ir_buffer[i]));
    filtered_ir_buffer[i] = round(filtered_ir);
  }

  for (int i= Moving_Average_Num;i<Num_Samples-Moving_Average_Num;i++){
      Sum_Points= 0;
      for( int k =0; k < Num_Points; k++){   
        Sum_Points = Sum_Points + filtered_ir_buffer[i-Moving_Average_Num+k];
        // Sum_Points = Sum_Points + filtered_gr_buffer[i-Moving_Average_Num+k]; 
      }    
      ma_ir_buffer[i] = Sum_Points/Num_Points;
      // ma_gr_buffer[i] = Sum_Points/Num_Points; 
  }
  
}

int Find_Peak(int p1, int p2, int *x){
  int Peak_Magnitude = 0;
  for (int m = p1; m < p2; m++){
      if(Peak_Magnitude < x[m]){
        Peak_Magnitude = x[m];
      }
  }
  return Peak_Magnitude;
}

int Find_Mean(int p1, int p2, int *x){
  int Mean = 0;
  int c = 0;
  for (int m = p1; m < p2; m++){
      if (x[m]>0){
        Mean = Mean+x[m];
        c++;
      }
  }
  if (c>0){
    return Mean/c;
  }else{
    return 1;
  }
}

int find_min_negative(int p1, int p2, int *x){
  int min_magnitude = 0;
  for (int m = p1; m < p2; m++)
  {
    if (min_magnitude > x[m])
    {
      min_magnitude = x[m];
    }
  }
  return min_magnitude;
}

void convert_signal_to_positive(int p1, int p2, int *x, int point){
  for (int m = p1; m < p2; m++)
  {
     x[m]=x[m]-point;
  }
}

void ComputeHeartRate(){
  
  // int Mean_Magnitude =Find_Mean(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_gr_buffer); //green
  int Mean_Magnitude =Find_Mean(Moving_Average_Num, Num_Samples - Moving_Average_Num, ma_ir_buffer);
  //***detect successive peaks and compute PR
  for (int i = 0; i < points_pr; i++){
     PR[i] = 0;
  }
  int  Peak = 0;
  int Index = 0;
  int p=0;

    
  for (int j = Moving_Average_Num; j < Num_Samples-Moving_Average_Num; j++){
      //***Find first peak
      
      // if(ma_gr_buffer[j] > ma_gr_buffer[j-1] && ma_gr_buffer[j] > ma_gr_buffer[j+1] && ma_gr_buffer[j] > Mean_Magnitude && Peak == 0){
       if(ma_ir_buffer[j] > ma_ir_buffer[j-1] && ma_ir_buffer[j] > ma_ir_buffer[j+1] && ma_ir_buffer[j] > Mean_Magnitude && Peak == 0){ 
        //  Peak = ma_gr_buffer[j]; //gr
        Peak = ma_ir_buffer[j];
         Index = j; 
      }
      
      //***Search for next peak 
      
      if(Peak > 0 ){
      //  if(ma_gr_buffer[j] > ma_gr_buffer[j-1] && ma_gr_buffer[j] > ma_gr_buffer[j+1] && ma_gr_buffer[j] > Mean_Magnitude){
        if(ma_ir_buffer[j] > ma_ir_buffer[j-1] && ma_ir_buffer[j] > ma_ir_buffer[j+1] && ma_ir_buffer[j] > Mean_Magnitude){
        float d=j-Index;
        float pulse=(float)samp_freq*60/d; //bpm for each PEAK interval
        PR[p]=pulse; 
        p++;
        p %= points_pr; //Wrap variable
        Peak = ma_ir_buffer[j];
        // Peak = ma_gr_buffer[j]; //gr
        Index = j;
       } 
      } 
  } 

  float sum=0;
  int c=0;
  for (int i=0; i< points_pr; i++){
    if (PR[i]!=0 && PR[i]!=INFINITY){
       sum=sum+PR[i];
       c=c+1;
    }      
  }

  if (c!=0){
      Pulse_Rate_next=sum/c;
      Serial.print("Pulse Rate next: "); Serial.print(Pulse_Rate_next); Serial.print(", \n");
      //these lines above do not print out anything
// thus c = 0, but why?
    if(Pulse_Rate_next > 40 && Pulse_Rate_next < 200){
      if (Pulse_Rate_next-Pulse_Rate_previous>=5){
        Pulse_Rate_next=Pulse_Rate_previous+1;
      }
      if (Pulse_Rate_next-Pulse_Rate_previous<=-5){
        Pulse_Rate_next=Pulse_Rate_previous-1;
      }
      Pulse_Rate_previous=Pulse_Rate_next;
    }
    else{
      Pulse_Rate_next=Pulse_Rate_previous;
    }
  }else{
    Pulse_Rate_next=Pulse_Rate_previous;
  }

  HR= int(round(Pulse_Rate_next));

}

int readSamples(){
 
  int flag=1;
  
  for (uint32_t i=0;i<Num_Samples;i++){ 
    //read max30105
    // gr_buffer[i] = Sensor.getGreen(); //green
    ir_buffer[i] = Sensor.getIR();
    //Sensor.nextSample();

    // if (gr_buffer[i]<10000){
    //   flag=0;
    // }
    if (ir_buffer[i]<80000){
      flag = 0;
    }
    delay(40);
  }

  return flag;
}

void ComputeSpO2(void * pvParameter){
   //need to initialise all the variables for SpO2, make them global
for(;;){ //we need this huge endless loop, for the FreeRTOS task, requirement
    xSemaphoreTake( baton, portMAX_DELAY);
    xSemaphoreGive( baton ); //this is the second line because it is the shortest task (compared to HR)
    double t_SP = millis();
    Serial.println(); Serial.print("t_SP: "); Serial.print(t_SP); Serial.println();
/*

    if (flag_unplugged = 1) { //calculate the SpO2 only if the finger is plugged, name of the flag is counterintuitive
      int mean_ir = Find_Mean(0, Num_Samples, ma_ir2_buffer);
      int mean_red = Find_Mean(0, Num_Samples, ma_red_buffer);
      
      //***detect successive peaks and mins
      for (int i = 0; i < points_spo2; i++){
        SpO2_dc_ir[i] = 0;
        SpO2_ac_ir[i]=0;
        SpO2_dc_red[i] = 0;
        SpO2_ac_red[i]=0;
        Oxy[i]=0;
      }

      int p_ir=0;
      int p_red=0;
      int peak_ir=0;
      int peak_red=0;
      int m_ir=0;
      int m_red=0;
    
      for (int j = 101; j < 350; j++){
        //***Find peaks and means for IR
        if(ma_ir2_buffer[j] > ma_ir2_buffer[j-1] && ma_ir2_buffer[j] > ma_ir2_buffer[j+1] && ma_ir2_buffer[j] > mean_ir && peak_ir==0){
        
          /*Serial.print("peak  ir: ");
          Serial.print(ma_ir2_buffer[j]);
          Serial.println();
          SpO2_dc_ir[p_ir]=ma_ir2_buffer[j]; 
          p_ir++;
          p_ir %= points_spo2; //Wrap variable
          peak_ir=1;
        }
        
        if(ma_ir2_buffer[j] < ma_ir2_buffer[j-1] && ma_ir2_buffer[j] < ma_ir2_buffer[j+1] && ma_ir2_buffer[j] < mean_ir && peak_ir==1){
          /*Serial.print("min  ir: ");
          Serial.print(ma_ir2_buffer[j]);
          Serial.println();
          SpO2_ac_ir[m_ir]=SpO2_dc_ir[p_ir-1]-ma_ir2_buffer[j]; 
          m_ir++;
          m_ir %= points_spo2; //Wrap variable
          peak_ir=0;
        }

        //***Find peaks and means for RED
        if(ma_red_buffer[j] > ma_red_buffer[j-1] && ma_red_buffer[j] > ma_red_buffer[j+1] && ma_red_buffer[j] > mean_red && peak_red==0){
        
          SpO2_dc_red[p_red]=ma_red_buffer[j]; 
          p_red++;
          p_red %= points_spo2; //Wrap variable
          peak_red=1;
        }

        if(ma_red_buffer[j] < ma_red_buffer[j-1] && ma_red_buffer[j] < ma_red_buffer[j+1] && ma_red_buffer[j] < mean_red && peak_red==1){
        
          SpO2_ac_red[m_red]=SpO2_dc_red[p_red-1]-ma_red_buffer[j]; 
          m_red++;
          m_red %= points_spo2; //Wrap variable
          peak_red=0;
        }
      }
      


      for (int i=1; i< points_spo2; i++){

        if (SpO2_ac_ir[i]!=0 && SpO2_ac_red[i]!=0){
          float R =(float(SpO2_ac_red[i])/float(SpO2_dc_red[i]))/(float(SpO2_ac_ir[i])/float(SpO2_dc_ir[i]));
          Oxy[i]=-45.060*R*R + 30.354*R + 94.845;
          // Serial.print("oxy: ");
          // Serial.print(Oxy[i]);
          // Serial.println();
        }   
            
      }

      float sumSP=0;
      int cSP=0;
      
      for (int i=1; i< points_spo2; i++){

        if (Oxy[i]>84 && Oxy[i]<=100){
          sumSP=sumSP+Oxy[i];
          cSP=cSP+1;
        }   
      }
    
      if(cSP>0){
        SpO2_next=sumSP/cSP;
      }
      
      // Serial.print("SpO2 predicted: ");
      // Serial.print(SpO2_next);
      // Serial.println();

      if(SpO2_next > 84 && SpO2_next <= 100){  
        if (SpO2_next-SpO2_previous<-4){
          SpO2_next=SpO2_previous;
        }
        if (SpO2_next-SpO2_previous<-3){
          SpO2_next=SpO2_previous-1;
        }
        SpO2_previous=SpO2_next;
      }else{
        SpO2_next=SpO2_previous;
      }
      
      SpO2=int(round(SpO2_next));
      if (flag_unplugged == 0 || HR == 0){ //if unplugged show this 
        //SpO2 = 0; //this is probably not needed
        Serial.println();
        Serial.println("SpO2 : unplugged");
      }else {
        Serial.println();
        Serial.print("SpO2 : ");
        Serial.print(SpO2);
        Serial.print(" %");
        Serial.println();
      }
      // Serial.println(); Serial.print("time to run the SpO2 task: "); Serial.print(millis() - t_SP); Serial.println(); // I want to measure how long does it take for SpO2 task to run
      vTaskDelay(Sampling_Time / portTICK_PERIOD_MS);
    }
    // xSemaphoreGive( baton ); //this line not here, as this is the shortest task, we immediately give the semaphore to the other task
    // delay(50);
    */
  }
}

//now both threads are in sync, they begin exactly at the same time
//this part of code is considered complete