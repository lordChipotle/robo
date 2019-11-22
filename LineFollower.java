import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.*;
import java.lang.Math; 
public class LineFollower {
/** The EV3 brick we're controlling */
private  EV3 brick;

private float lastAngle; 

/** The motor on the left side of the robot */
private  UnregulatedMotor motorB;

/** The motor on the right side of the robot */
private  UnregulatedMotor motorC;

/** The raw EV3 Color Sensor object */
private  EV3ColorSensor colorSensor;

private  EV3GyroSensor gyro;
/** The raw EV3 Ultrasonic Sensor object */
private  EV3UltrasonicSensor ultrasonicSensor;

private SampleProvider gyroProvider; // use this to fetch samples
private SampleProvider colorProvider; 
private SampleProvider distanceProvider; 
	        
public  void main(String[] args) {
 motorB = new UnregulatedMotor(MotorPort.B);
 motorC = new UnregulatedMotor(MotorPort.C);
 colorSensor = new EV3ColorSensor(SensorPort.S3);
 ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
 gyro = new EV3GyroSensor(SensorPort.S2);
 gyroProvider = gyro.getAngleMode();
 colorProvider = colorSensor.getMode("color");
 distanceProvider = ultrasonicSensor.getMode("distance");
 gyro.reset();
 move(); // start it running
}
private  void move() {
	while(colorSensor.getColorID() == 5) {
		motorB.stop();
		motorC.stop();
	}		
// so long as we can keep finding the line...
while (findLine()) { followLine(); }
}

public void tankDrive(double left, double right) {
	if(left>100) {
		left = 100;
	}
	else if (left<-100) {
		left = -100;
	}
	if(right>100) {
		right =100;
	}
	else if(right<-100) {
		right = -100;
	}
	if (left>0) {
		motorB.setPower( (int) Math.abs(right));

		motorB.forward();
		if (right>0) {
			motorC.setPower( (int) Math.abs(right));
			motorC.forward();
		}
		else if (right<0) {
			motorC.setPower( (int) Math.abs(right));

			motorC.backward();
		}
		else {
			motorC.stop();
		}
	}
	else if (left<0) {
		motorB.setPower( (int) Math.abs(right));

		motorB.backward();
	}
	else {
		motorB.stop();
	}
		
	}

public void arcadeDrive(double throttleValue, double turnValue) {
    double leftMtr;
    double rightMtr;
    leftMtr = throttleValue + turnValue;
    rightMtr = throttleValue - turnValue;
    tankDrive(leftMtr, rightMtr);
}
public float getM() {
	float[] sample = new float[gyroProvider.sampleSize()];
	while(true) 
		  return sample[0];
}
private void turn(float degrees) {
	int P, I, D = 1;
    int integral, previous_error, setpoint = 0;
    double error = setpoint - getM();
	
	while (getM()!=degrees) {
		
		if (degrees>0) {
			motorB.backward();
			motorC.forward();

		}
		else if(degrees<0) {
			motorB.forward();
			motorC.backward();
		}
		else {
			arcadeDrive(0,0);
		}
	}
}
	
	        
private void findInitialLine() {

	                // Use the "calibrated" variable as your sample source
	                // to call fetchSample() on

	                // Turn using turn(), and take samples while its turning

	                // When you've finished turning, turn back to the location
	                // where you saw the best reading.
	                //
	                // Hint: see the following methods that your motors support:
	                // http://web.suffieldacademy.org/cs/lejos_ev3_api/lejos/robotics/Encoder.html#getTachoCount--
	                // http://web.suffieldacademy.org/cs/lejos_ev3_api/lejos/robotics/Encoder.html#resetTachoCount--
}

private  void followLine(){
 double Kp = 0.005;
 double Ki = 375; 
 double Kd = 25; 
 double integral = 0;
 double derivative = 0;
 double lastError = 0;
 double dt = 0;
 double cTurn;
 double bTurn;
 double offset = 45; 
 double Tp = 50; 
 while(colorSensor.getColorID() == 1){
  lastAngle = getM();
  double lightValue = colorSensor.getFloodlight();
  double error = lightValue - offset; 
  dt = 0;
  integral = integral + error * dt; 
  derivative = (error - lastError)/dt; 
  double turn = Kp * offset + Ki * integral + Kd * derivative;
  bTurn = Tp + turn; 
  cTurn = Tp - turn;
  lastError = error; 
  motorB.setPower(new Double(bTurn).intValue());
  motorB.forward();
  motorC.setPower(new Double(cTurn).intValue());
  motorC.forward();
  }
}

private  boolean findLine() {
	float[] sample = new float[colorProvider.sampleSize()];
	sample[0] = -1;
	while (sample[0] > 0.5) {
		if(lastAngle > getM()) turn(getM()+10); 
		else turn(getM()-10);
		colorProvider.fetchSample(sample, 0);
	}
	return true;
}
}
