#include <max6675.h>
#include <PID_v1.h>
//#include <Adafruit_MAX31865.h>

/* Key-Value Table
 *  Key     Value
 *  1       is L298N Launched        
 *  2       currentDirection(0 for positive,1 for negetive,2 for stop)
 *  3       currentPMW 
 *  4       set temperature
 *  5       set Kp
 *  6       set Ki
 *  7       set Kd
 *  8       set sampleTime
 */
int baudrate = 9600;

int L298N_ENA_PIN = 5;
int L298N_IN1_PIN = 39;
int L298N_IN2_PIN = 37;

int isLaunched = 0; //0 for stop,   1 for running
int inputMode = 0;  //0 for key,    1 for value
int inputKey = 0;
bool inputState=false;//false for key,true for value

double inputValue = 0;
double setTemp = 0;
double curTemp = 0;
double deltaTemp = 0.3;
double tecPWM = 0;

double Kp = 100;
double Ki = 0;
double Kd = 0;
double sampleTime = 5;
double pwmMax = 250;
double pwmMin = -250;

String strEmpty = "";
String comInputData = "";
String comOutputData="";

int x1 = 512;
int y1 = 512;
int z1 = 512;
int r1 = 512;
int x2 = 512;
int y2 = 512;
int z2 = 512;
int r2 = 512;

//#define RREF      430.0
//#define RNOMINAL  100.0
//Adafruit_MAX31865 max = Adafruit_MAX31865(24,25,26,27);

MAX6675 tc = MAX6675(52, 50, 48);
PID pid(&curTemp, &tecPWM, &setTemp, Kp, Ki, Kd, P_ON_E, DIRECT);

bool serialRead()
{
    while(Serial.available()>0)
    {
        char tmp=char(Serial.read());
        if(tmp==','){
            if(comInputData.length()>0){
                if(!inputState){
                    inputKey=comInputData.toInt();
                    Serial.println("InputKey: " + String(inputKey));
                }
                else{
                    inputValue=comInputData.toDouble();
                    Serial.println("InputValue: " + String(inputValue));
                }
                comInputData=strEmpty;
                inputState=!inputState;
                return !inputState;
            }
            else{
                comInputData=strEmpty;
                return false;
            }
                return false;
        }
        else if(tmp>='0' && tmp<='9'){
            comInputData+=tmp;
            continue;
        }
    }
    return false;
}

void setup()
{
    // put your setup code here, to run once:
    //Serial.begin(9600);
    Serial.begin(baudrate);
    //max.begin(MAX31865_3WIRE);
    //curTemp=max.temperature(RNOMINAL, RREF);
    curTemp = tc.readCelsius();
    setTemp = curTemp;
    setTemp = 40;
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(pwmMin, pwmMax);
    pid.SetSampleTime(sampleTime);
    pinMode(L298N_ENA_PIN, OUTPUT);
    pinMode(L298N_IN1_PIN, OUTPUT);
    pinMode(L298N_IN2_PIN, OUTPUT);
    //Serial.println("On and Off Temperature Controll(via PT100) Start!");
}

void loop()
{
    // put your main code here, to run repeatedly:
    //curTemp=max.temperature(RNOMINAL, RREF);
    curTemp = tc.readCelsius();
    if (Serial.available() > 0 && serialRead())
    {
        switch (inputKey)
        {
        case 1:
            isLaunched = int(inputValue);
            break;
        case 2:
            switch (int(inputValue))
            {
            case 0:
                digitalWrite(L298N_IN1_PIN, 1);
                digitalWrite(L298N_IN2_PIN, 0);
                break;
            case 1:
                digitalWrite(L298N_IN1_PIN, 0);
                digitalWrite(L298N_IN2_PIN, 1);
                break;
            case 2:
                digitalWrite(L298N_IN1_PIN, 0);
                digitalWrite(L298N_IN2_PIN, 0);
                break;
            default:
                break;
            }
            break;
        case 3:
            if (isLaunched == 1)
                analogWrite(L298N_ENA_PIN, int(inputValue));
            break;
        case 4:
            setTemp = inputValue;
            break;
        case 5:
            Kp = inputValue;
            pid.SetTunings(Kp, Ki, Kd);
            break;
        case 6:
            Ki = inputValue;
            pid.SetTunings(Kp, Ki, Kd);
            break;
        case 7:
            Kd = inputValue;
            pid.SetTunings(Kp, Ki, Kd);
            break;
        case 8:
            sampleTime = inputValue;
            pid.SetSampleTime(sampleTime);
        default:
            break;
        }
    }
    if (isLaunched == 1)
    {
        pid.Compute();
        if (tecPWM >= 0)
        {
            digitalWrite(L298N_IN1_PIN, 1);
            digitalWrite(L298N_IN2_PIN, 0);
            analogWrite(L298N_ENA_PIN, int(tecPWM));
        }
        else
        {
            digitalWrite(L298N_IN1_PIN, 0);
            digitalWrite(L298N_IN2_PIN, 1);
            analogWrite(L298N_ENA_PIN, int(-tecPWM));
        }
    }
    else if (isLaunched == 0)
    {
        tecPWM = 0;
        analogWrite(L298N_ENA_PIN, 0);
    }
    comOutputData=
        String(curTemp) + "," + 
        String(tecPWM) + ","; 
    Serial.print(comOutputData);
    delay(100);
}