
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.*;
import java.lang.Math;
import lejos.utility.Delay; 


public class LineFollower {
	/** The EV3 brick we're controlling */

	//private static float lastAngle;

	/** The motor on the left side of the robot */
	private static EV3LargeRegulatedMotor motorB;

	/** The motor on the right side of the robot */
	private static EV3LargeRegulatedMotor motorC;

	/** The raw EV3 Color Sensor object */
	private static EV3ColorSensor colorSensor;

	private static EV3GyroSensor gyro;
	/** The raw EV3 Ultrasonic Sensor object */

	private static SampleProvider gyroProvider; // use this to fetch samples
	private static SampleProvider colorProvider;

	public static void main(String[] args) {
		
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		colorSensor = new EV3ColorSensor(SensorPort.S3);
		//ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
		gyro = new EV3GyroSensor(SensorPort.S2);
		gyroProvider = gyro.getAngleMode();
		colorProvider = colorSensor.getColorIDMode();
		gyro.reset();
		move();
		/*if (colorSensor.getColorID() <6 && colorSensor.getColorID()>4){
			move();
		}*/
}
		 // start it running
	

	private static void move() {
        while(colorSensor.getColorID() == 5) { Delay.msDelay(100); }
		if(colorSensor.getColorID() != 5) {
			followLine(); 
		}	
	}

	/*public static void tankDrive(double left, double right) {
		if (left > 100) {
			left = 100;
		} else if (left < -100) {
			left = -100;
		}
		if (right > 100) {
			right = 100;
		} else if (right < -100) {
			right = -100;
		}
		if (left > 0) {
			motorB.setSpeed((int) Math.abs(right));

			motorB.forward();
			if (right > 0) {
				motorC.setSpeed((int) Math.abs(right));
				motorC.forward();
			} else if (right < 0) {
				motorC.setSpeed((int) Math.abs(right));

				motorC.backward();
			} else {
				motorC.stop();
			}
		} else if (left < 0) {
			motorB.setSpeed((int) Math.abs(right));

			motorB.backward();
		} else {
			motorB.stop();
		}

	}

	public static void arcadeDrive(double throttleValue, double turnValue) {
		double leftMtr;
		double rightMtr;
		leftMtr = throttleValue + turnValue;
		rightMtr = throttleValue - turnValue;
		tankDrive(leftMtr, rightMtr);
	}

	public static float getM() {
		float[] sample = new float[gyroProvider.sampleSize()];
		while (true)
			return sample[0];
	}

	private static void turn(float degrees, double kP) {

		double error = degrees - getM();
		if (getM() > degrees) {
			arcadeDrive(0, kP * error);
		} else {
			arcadeDrive(0, -kP * error);
		}
	}*/

	private static void followLine() {
		double Kp = 0.15;
		//double Ki = 0.01;
		//double Kd = 0.0;
		//double Kf = 0.2;
		//double integral = 0;
		//double derivative = 0;
		//double lastError = 0;
		//double dt = 0;
		double cTurn;
		double bTurn;
		double offset = 45;
		double Tp = 50;
		while (getState()) {
			//lastAngle = getM();
			double lightValue = colorSensor.getFloodlight();
			double error = lightValue - offset;
			//dt = 0;
			//integral = integral + error * dt;
			//derivative = (error - lastError) / dt;
			double turn = Kp * error; //+ Ki * integral + Kd * derivative;
			bTurn = Tp + turn;
			cTurn = Tp - turn;
			//lastError = error;
			motorB.setSpeed(new Double(bTurn).intValue());
			motorB.forward();
			motorC.setSpeed(new Double(cTurn).intValue());
			motorC.forward();
		}
	}

	private static boolean getState() {
		/*float[] sample = new float[colorProvider.sampleSize()];
	    sample[0] = -1;
		while (sample[0] > 0.5) {
			turn(45, 0.15);
			colorProvider.fetchSample(sample, 0);
		}*/
		if(colorSensor.getColorID() == 1) return true; 
		else if(colorSensor.getColorID() == 5) return false; 
		else { return findLine(); }
	}
	
	private static boolean findLine() {
		while(colorSensor.getColorID() != 1) { turnDegree(45); }
		return true; 
	}
	
	private static void turnDegree(float degree) {
		motorB.rotate(45);
		motorC.rotate(45);
	}
}
