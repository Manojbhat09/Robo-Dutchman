#include <SharpIR.h>

#define IR_MODEL 1080
#define IR1_PIN A0
#define IR2_PIN A1
#define IR3_PIN A2
#define IR4_PIN A3
#define IR5_PIN A4

#define JUMP_THRESH 10000

// Create a new instance of the SharpIR class:
SharpIR IR1 = SharpIR(IR1_PIN, IR_MODEL);
SharpIR IR2 = SharpIR(IR2_PIN, IR_MODEL);
SharpIR IR3 = SharpIR(IR3_PIN, IR_MODEL);
SharpIR IR4 = SharpIR(IR4_PIN, IR_MODEL);
SharpIR IR5 = SharpIR(IR5_PIN, IR_MODEL);

float IR1_dist, IR2_dist, IR3_dist, IR4_dist, IR5_dist;
float IR1_new_dist, IR2_new_dist, IR3_new_dist, IR4_new_dist, IR5_new_dist;

void print_vals(){
  Serial.print(IR1_dist);
  Serial.print(' ');
  Serial.print(IR2_dist);
  Serial.print(' ');
  Serial.print(IR3_dist);
  Serial.print(' ');
  Serial.print(IR4_dist);
  Serial.print(' ');
  Serial.println(IR5_dist);
} 

void update_vals(){
  float k = 1;
  IR1_new_dist = IR1.distance();
  IR2_new_dist = IR2.distance();
  IR3_new_dist = IR3.distance();
  IR4_new_dist = IR4.distance();
  IR5_new_dist = IR5.distance();

  if (abs(IR1_new_dist - IR1_dist) < JUMP_THRESH){
    IR1_dist = IR1_dist + k * (IR1_new_dist - IR1_dist);
  }
  if (abs(IR2_new_dist - IR2_dist) < JUMP_THRESH){
    IR2_dist = IR2_dist + k * (IR2_new_dist - IR2_dist);
  }
  if (abs(IR3_new_dist - IR3_dist) < JUMP_THRESH){
    IR3_dist = IR3_dist + k * (IR3_new_dist - IR3_dist);
  }
  if (abs(IR4_new_dist - IR4_dist) < JUMP_THRESH){
    IR4_dist = IR4_dist + k * (IR4_new_dist - IR4_dist);
  }
  if (abs(IR5_new_dist - IR5_dist) < JUMP_THRESH){
    IR5_dist = IR5_dist + k * (IR5_new_dist - IR5_dist);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  update_vals();
  print_vals();
  
  delay(20);
}
