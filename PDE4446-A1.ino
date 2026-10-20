//Motor pins
const int AIN2 = 5;

const int BIN1 = 6;

//Encoder pins
const int LEFT_ENCODER = 3;
const int RIGHT_ENCODER = 2;

//Encoder counts
volatile long leftCount = 0;
volatile long rightCount = 0; 

//distance calculation values
const int COUNTS_PER_REV = 427;
const float COUNT_PER_MM = 2.091;
float stopDistance = 5000/6*COUNT_PER_MM;
int times = 0;

//targeted speed
int leftTargetSpeed = 255;
int rightTargetSpeed = 255;

//count stop
float targetCount = 5000/6*COUNT_PER_MM;

//Proportional gain
const float KP = 1.2;

//Encoder interrupts
void leftEncoder() {
  leftCount++;
}

void rightEncoder() {
  rightCount++;
}

void setup(){
  delay(2000);
  Serial.begin(9600);
  
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);

  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER),
    leftEncoder, 
    RISING);

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER),
    rightEncoder, 
    RISING);  
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

void loop(){
  driveForward();
  while(true){

  }

}

void driveForward(){
  
  while(true){  
    long left;
    long right;
    noInterrupts();
    left = leftCount;
    right = rightCount;
    interrupts();
    float averageCount = (left + right) / 2.0;
    /*
    if(averageCount >= targetCount){
      if(times<5){
        targetCount+=stopDistance;
        times++;
        analogWrite(AIN2, 0);
        analogWrite(BIN1, 0);
        delay(5000);
        continue;
      }else {
        stop();
        return;
      }
      
    }*/
    

    long error = left - right;
    
    float correction = KP * error;
    int leftCorrection = constrain(leftTargetSpeed - correction, 150, 255);
    int rightCorrection = constrain(rightTargetSpeed + correction, 150, 255);

    Serial.print("Left=");
    Serial.print(left);

    Serial.print(" Right=");
    Serial.print(right);
    Serial.print(" Avg Count=");
    Serial.print(averageCount);
    Serial.print(" Error=");
    Serial.println(error);
    analogWrite(AIN2, leftCorrection);  //204.2 mm revolution count 427
    analogWrite(BIN1, rightCorrection); //204.2 mm revolution count 430
    
    
  }
}

void stop(){
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
}

