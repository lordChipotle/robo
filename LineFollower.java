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
	
	private static SampleProvider intensityProvider;
	
	private static float[] values; 
	
	/** The raw EV3 Ultrasonic Sensor object */
	//private static EV3UltrasonicSensor ultrasonicSensor;
	
	//SampleProvider distanceProvider;
	
	//float[] distance;

	public static void main(String[] args) {
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		//motorD = new EV3LargeRegulatedMotor(MotorPort.D);
		colorSensor = new EV3ColorSensor(SensorPort.S3);
		intensityProvider = colorSensor.getRedMode(); 
		values = new float[intensityProvider.sampleSize()];
		//ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
        //ultrasonicSensor.enable(); 
		//distanceProvider = ultrasonicSensor.getDistanceMode(); 
		//distance = new float[distanceProvider.sampleSize()];
		
		Button.waitForAnyPress(); 
		
		intensityProvider.fetchSample(values, 0);
		
		while(colorSensor.getColorID() != Color.BLACK) { Delay.msDelay(100); }
		
		move(); 
	}
	
	private static void move() {
		double Kp = 400; //350;  
		double offset = 0.45; //0.38 // try less than 20 
		double cTurn;
		double bTurn;
		double Tp = 50;
		//SampleProvider intensityProvider = colorSensor.getRedMode(); 
		//float[] values = new float[intensityProvider.sampleSize()];
		intensityProvider.fetchSample(values,0); 
        //distanceProvider.fetchSample(distance,0); 
		while(values[0] < 1){
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
		 // if(distance[0] < 20) { avoidObstacle(); }
		  while(colorSensor.getColorID() !=  Color.BLACK && colorSensor.getColorID() !=  Color.WHITE) { motorB.stop(); motorC.stop(); }
		} 
	}
	
	/*private static void avoidObstacle(){
	    motorB.rotate(90); 
	    motorC.rotate(90); 
	    intensityProvider.fetchSample(values,0); 
	    double target = distance[0]; 
	    while(values[0] >= 0.80/color.getColorID() == Color.WHITE) {
		  distanceProvider.fetchSample(distance,0);
		  if(distance[0] < target) { motorB.rotate(10); motorC.rotate(10); motorB.forward(); motorC.forward();}
		  else if(distance[0] > target) { motorB.rotate(-10); motorC.rotate(-10); motorB.forward(); motorC.forward();}
		  else if(distance[0] == target) { motorB.forward(); motorC.forward(); }
		  intensityProvider.fetchSample(values,0); 
		}
		motorD.rotate(-90); 
	}*/
	
	
}
