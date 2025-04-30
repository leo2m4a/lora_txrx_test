#include <LiquidCrystal.h>
//使用到的PIN腳
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_D4 = 4; 
const int pin_D5 = 5; 
const int pin_D6 = 6; 
const int pin_D7 = 7; 

//填入正確的PIN腳，建立LCD物件
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_D4,  pin_D5,  pin_D6,  pin_D7);
//
String incomingStr; // for incoming serial2 String

void setup() {
 lcd.begin(16, 2);
 lcd.setCursor(0,0);
 lcd.print("JMaker.com.tw");
 lcd.setCursor(0,1);
 lcd.print("Press Key:");
  Serial1.begin(115200);
  Serial1.println("UROK");
}

void loop() {
  int x;
  x = analogRead (0);  //讀取A0的值
    
  lcd.setCursor(10,1); //把游標移到第2行第11個字
 
 //依A0的值，去判斷是哪個鍵被按下了
 if (x < 60) {
  lcd.print ("Right ");
 }
 else if (x < 200) {
  lcd.print ("Up    ");
 }
 else if (x < 400){
  lcd.print ("Down  ");
 }
 else if (x < 600){
  lcd.print ("Left  ");
 }
 else if (x < 800){
  lcd.print ("Select");
 }
   if (Serial1.available() > 0) {
        // read the incoming byte:
        incomingStr =  Serial1.readString();// read the incoming data as string
        lcd.setCursor(0,0);
        lcd.print(incomingStr);
        // say what you got:
        //Serial.print("ESP32-S received: ");
        //Serial.println(incomingStr);
        //printStr = printStr + incomingStr + "<br>"; 
      }  
} 