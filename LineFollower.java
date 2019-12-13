import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.Button; 
import lejos.robotics.*;
import java.lang.Math;
import lejos.utility.Delay; 
import lejos.hardware.lcd.LCD;


public class LineFollower3{

	/** The motor on the left side of the robot */
	private static EV3LargeRegulatedMotor motorB;

	/** The motor on the right side of the robot */
	private static EV3LargeRegulatedMotor motorC;
	
	/** The motor on the right side of the robot */
	private static EV3LargeRegulatedMotor motorA;

	/** The raw EV3 Color Sensor object */
	private static EV3ColorSensor colorSensor;
	
	private static SampleProvider intensityProvider;
	
	private static float[] values; 
	
	/** The raw EV3 Ultrasonic Sensor object */
	private static EV3UltrasonicSensor ultrasonicSensor;
	
	private static SampleProvider distanceProvider;
	
	private static float[] distance;

	private static double target; 
	
	public static void main(String[] args) {
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		motorA = new EV3LargeRegulatedMotor(MotorPort.A);
		colorSensor = new EV3ColorSensor(SensorPort.S3);
		intensityProvider = colorSensor.getRedMode(); 
		values = new float[intensityProvider.sampleSize()];
		ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
        ultrasonicSensor.enable(); 
		distanceProvider = ultrasonicSensor.getDistanceMode(); 
		distance = new float[distanceProvider.sampleSize()];
		
		Button.waitForAnyPress(); 
		
		intensityProvider.fetchSample(values, 0);
		
		while(colorSensor.getColorID() == Color.RED) { Delay.msDelay(100); }
		
		move(); 
	}
	
	private static void move() {
		double Kp = 400;   
		double Kd = 40; 
		double offset = 0.38; 
		double cTurn;
		double bTurn;
		double Tp = 100; 
		double prevTime = 0; 
		intensityProvider.fetchSample(values,0); 
		while(values[0] < 1){
		  double error = values[0] - offset; 
		  double dt = System.currentTimeMillis() - prevTime; 
		prevTime = System.currentTimeMillis(); 
		  double derror = error * dt;
	      double turn = Kp * error + Kd*derror; 
		  bTurn = Tp - turn;
		  cTurn = Tp + turn;
		  motorB.setSpeed(new Double(bTurn).intValue());
		  motorB.forward();
		  motorC.setSpeed(new Double(cTurn).intValue());
		  motorC.forward();
		  intensityProvider.fetchSample(values,0);
		  distanceProvider.fetchSample(distance,0);
		  if(distance[0] < 0.10) { target = distance[0]; motorB.stop(); motorC.stop(); avoidObstacle(); }
		  while(colorSensor.getColorID() ==  Color.RED) { motorB.stop(); motorC.stop(); Delay.msDelay(100)}
		} 
	}
	
	private static void avoidObstacle(){
	    motorB.setSpeed(50);
	    motorC.setSpeed(50);
	    motorB.rotate(180); 
	    motorC.rotate(-180);
	    motorB.backward();
	    motorC.backward();
	    Delay.msDelay(100);
	    motorA.rotate(-90);
	    motorB.forward();
	    motorC.forward();
	    Delay.msDelay(1000);
	    intensityProvider.fetchSample(values,0);  
	    while(values[0] >= 0.60) {
		  distanceProvider.fetchSample(distance,0);
		  if(distance[0] < target) { motorB.setSpeed(100); motorC.setSpeed(50); motorB.forward(); motorC.forward();}
		  else if(distance[0] > target) { motorB.setSpeed(50); motorC.setSpeed(100); motorB.forward(); motorC.forward(); }
		  else if(distance[0] == target) { motorB.setSpeed(50); motorC.setSpeed(50); motorB.forward(); motorC.forward();}
		  intensityProvider.fetchSample(values,0); 
		}
	    motorB.stop(); 
	    motorC.stop(); 
	    motorA.rotate(90); 
	    motorB.setSpeed(50);
	    motorC.setSpeed(50);
	    motorB.backward(); 
	    motorC.backward():
	    Delay.msDelay(100); 
	    while(colorSensor.getColorID() == Color.WHITE){
		  motorB.rotate(45);
		  motorC.rotate(-45);
		  motorB.forward();
		  motorC.forward(); 
		}
	}
}
