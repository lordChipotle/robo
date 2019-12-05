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
	//private static EV3LargeRegulatedMotor motorD;

	/** The raw EV3 Color Sensor object */
	private static EV3ColorSensor colorSensor;
	
	/** The raw EV3 Ultrasonic Sensor object */
	//private static EV3UltrasonicSensor ultrasonicSensor;
	
	//SampleProvider distanceProvider;
	
	//float[] distance;

	public static void main(String[] args) {
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		//motorD = new EV3LargeRegulatedMotor(MotorPort.D);
		colorSensor = new EV3ColorSensor(SensorPort.S3);
		//ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
        //ultrasonicSensor.enable(); 
		//distanceProvider = ultrasonicSensor.getDistanceMode(); 
		//distance = new float[distanceProvider.sampleSize()];
		
		Button.waitForAnyPress(); 
		
		while(colorSensor.getColorID() == 5) { Delay.msDelay(100);}
		
		move();
		
		motorB.stop(); 
		motorC.stop(); 
		
		//while(colorSensor.getColorID() != 5) {
		//     move(); 
		//     if( distance[0] < 20 ) { motorD.rotate(90); avoidObstacle(); }
		//     while(colorSensor.getColorID() == 5) { Delay.msDelay(100); } 
		//  } 
	}
	
	private static void move() {
		double Kp = 350;  
		double offset = 0.35; //0.38
		double cTurn;
		double bTurn;
		double Tp = 50;
		SampleProvider intensityProvider = colorSensor.getRedMode(); 
		float[] values = new float[intensityProvider.sampleSize()];
		intensityProvider.fetchSample(values,0); 
        //distanceProvider.fetchSample(distance,0); 
		while(colorSensor.getColorID() != 5){
		  LCD.clear(); 
		  LCD.drawChar((char)values[0], 3,3);
		  double error = values[0] - offset; 
	      double turn = Kp * error; 
		  bTurn = Tp - turn;
		  cTurn = Tp + turn;
		  motorB.setSpeed(new Double(bTurn).intValue());
		  motorB.forward();
		  motorC.setSpeed(new Double(cTurn).intValue());
		  motorC.forward();
		  intensityProvider.fetchSample(values,0);
		 // distanceProvider.fetchSample(distance,0);
		 // if(distance[0] < 20) { break; }
		} 
	}
	
	/*private static void avoidObstacle(){
		while( distance[0] < 20 ) {
		  ...
		  distanceProvider.fetchSample(distance,0);
		}
		motorD.rotate(-90); 
	}*/
}
