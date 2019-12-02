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


public class LineFollower {

	/** The motor on the left side of the robot */
	private static EV3LargeRegulatedMotor motorB;

	/** The motor on the right side of the robot */
	private static EV3LargeRegulatedMotor motorC;

	/** The raw EV3 Color Sensor object */
	private static EV3ColorSensor colorSensor;
	
	//private static float[] samples; 

	public static void main(String[] args) {
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		colorSensor = new EV3ColorSensor(SensorPort.S3);
		
		Button.waitForAnyPress(); 
		
		move(); 
	}
	
	private static void move() {
		double Kp = 300;
		//double Ki = 0;
		//double Kd = 0;
		//double integral = 0;
		//double derivative = 0;
		//double lastError = 0;
		//double dt = System.currentTimeMillis()/1000;
		double offset = 0.45; 
		double cTurn;
		double bTurn;
		double Tp = 50;
		SampleProvider intensityProvider = colorSensor.getRedMode(); 
		float[] values = new float[intensityProvider.sampleSize()];
		intensityProvider.fetchSample(values,0); 
		while(values[0] < 1){
		  double error = values[0] - offset; 
		  //integral = integral + error * dt;
		  //derivative = (error - lastError) / dt;
	      double turn = Kp * error; //Ki * integral + Kd * derivative;
		  bTurn = Tp + turn;
		  cTurn = Tp - turn;
		  lastError = error;
		  motorB.setSpeed(new Double(bTurn).intValue());
		  motorB.forward();
		  motorC.setSpeed(new Double(cTurn).intValue());
		  motorC.forward();
		  intensityProvider.fetchSample(values,0);
		  //colorProvider.fetchSample(samples,0);
		  //dt = System.currentTimeMillis()/1000 - dt; 
		}
		motorB.stop(); 
		motorC.stop(); 
	}
}
