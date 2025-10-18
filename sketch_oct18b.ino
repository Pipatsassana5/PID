#include <Servo.h>
#include <math.h> // เพิ่ม library สำหรับ M_PI

#define Umax 66 // degrees
#define Umin -66
#define Umax_rad 1.151 // radians
#define Umin_rad -1.151
#define T 0.09

const int pingPin = D7; // trigpin
int inPin = D6;


Servo servo;

// --- กำหนดค่า Setpoint ที่ต้องการโดยตรง (หน่วยเป็นเมตร) ---
// คุณสามารถเปลี่ยนค่านี้ได้ตามต้องการ เช่น 0.30 คือ 30 เซนติเมตร
const double desired_setpoint_meters = 0.15;

double setpoint, setpoint_prec;
double y, y_prec;
double error;
double P, I, D, U;
double I_prec=0, U_prec=0, D_prec=0;
boolean Saturation = false;

double Kp = 8.6;
double Ki = 1.1;
double Kd = 6.3;
float measure_1 (void);
// --- ลบฟังก์ชัน measure_2 ออกไป ---
void move_servo(int);



void setup() {
Serial.begin(9600);
servo.attach(D8);

    delay(1000);
    move_servo(90);
    delay(2000);

    // --- กำหนดค่า setpoint เริ่มต้นด้วยค่าที่ป้อนไว้ ---
    setpoint_prec = desired_setpoint_meters;
    delay(1000);
    y_prec = measure_1(); // carrello
    delay(1000);
}
void loop()
{
    setpoint = desired_setpoint_meters;

    // --- ไม่จำเป็นต้องใช้ digital filter กับ setpoint ที่เป็นค่าคงที่แล้ว ---
    // setpoint = 0.53*setpoint + 0.47*setpoint_prec;

    delay(3);

    y = measure_1(); // cart // meters (  alfa*y :  if alfa increase, y less filteres  --> so the signal is dirty but fast )
     if (y > 0.0 && y <= 22.0) {
        y =  0.53*y + 0.47*y_prec; // digital filter
    } else {
        y = y_prec;
    }
    delay (3);

    error = round( 100*(y - setpoint) )*0.01;

    // PID control
    P = Kp*error;

    if ( ! Saturation ) I = I_prec + T*Ki*error;

    D = (Kd/T)*(y - y_prec);
    D = 0.56*D + 0.44*D_prec; // filter D     (  alfa*D :  if alfa increase, D less filteres  --> so the signal is dirty but fast )

    U = P + I + round(100*D)*0.01 ; // U in radians

    // saturate control action
    if ( U < Umin_rad) {
        U=Umin_rad;
        Saturation = true;
    }
    else if ( U > Umax_rad) {
        U=Umax_rad;
        Saturation = true;
    }
    else {
        Saturation = false;
    }

    U=round(U*180/M_PI); // Transform U in degrees:   -63 < U° < 63

    U=map(U, Umin, Umax, 156, 24);

    if (U < 83 || U > 95 || abs(error) > 0.02 ) move_servo( round(U) ); // it is used to stop the servo when the setpoint is reached

    delay (24);

    // แสดงผลเพื่อตรวจสอบ
    Serial.print("Setpoint(cm): ");
    Serial.print(setpoint*100);
    Serial.print(" | Cart Position(cm): ");
    Serial.print(y*100);
    Serial.print(" | Error(cm): ");
    Serial.print(error*100);
    Serial.print(" | Servo Angle: ");
    Serial.println(U);

    I_prec = I;
    y_prec = y;
    D_prec = D;
    setpoint_prec = setpoint;
}
long microsecondsToCentimeters(long microseconds)
{
// ความเร็วเสียงในอากาศประมาณ 340 เมตร/วินาที หรือ 29 ไมโครวินาที/เซนติเมตร
// ระยะทางที่ส่งเสียงออกไปจนเสียงสะท้อนกลับมาสามารถใช้หาระยะทางของวัตถุได้
// เวลาที่ใช้คือ ระยะทางไปกลับ ดังนั้นระยะทางคือ ครึ่งหนึ่งของที่วัดได้
return microseconds / 29 / 2;
}
float measure_1(){
long duration, cm;
pinMode(pingPin, OUTPUT);
digitalWrite(pingPin, LOW);
delayMicroseconds(2);
digitalWrite(pingPin, HIGH);
delayMicroseconds(5);
digitalWrite(pingPin, LOW);
pinMode(inPin, INPUT);

    duration = pulseIn(inPin, HIGH, 25000);
cm = microsecondsToCentimeters(duration);

return (float)cm/100.0;
}


void move_servo(int u) {
    servo.write(u-map(u, 30, 150, 14, 3));
}


