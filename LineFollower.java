package k18;

import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.*;
import java.lang.Math;


public class LineFollower {
	/** The EV3 brick we're controlling */

	private static float lastAngle;

	/** The motor on the left side of the robot */
	private static UnregulatedMotor motorB;

	/** The motor on the right side of the robot */
	private static UnregulatedMotor motorC;

	/** The raw EV3 Color Sensor object */
	private static EV3ColorSensor colorSensor;

	private static EV3GyroSensor gyro;
	/** The raw EV3 Ultrasonic Sensor object */

	private static SampleProvider gyroProvider; // use this to fetch samples
	private static SampleProvider colorProvider;

	public static void main(String[] args) {
		
		motorB = new UnregulatedMotor(MotorPort.B);
		motorC = new UnregulatedMotor(MotorPort.C);
		colorSensor = new EV3ColorSensor(SensorPort.S3);
		//ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S4);
		gyro = new EV3GyroSensor(SensorPort.S2);
		gyroProvider = gyro.getAngleMode();
		colorProvider = colorSensor.getColorIDMode();
		gyro.reset();
		if (colorSensor.getColorID() <6&&colorSensor.getColorID()>4){
			move();
		}
		}
		 // start it running
	

	private static void move() {
    // so long as we can keep finding the line...
	
		while (findLine()) {
			followLine();
		}
		motorB.stop();
		motorC.stop();
	}

	
	

	public static void tankDrive(double left, double right) {
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
			motorB.setPower((int) Math.abs(right));

			motorB.forward();
			if (right > 0) {
				motorC.setPower((int) Math.abs(right));
				motorC.forward();
			} else if (right < 0) {
				motorC.setPower((int) Math.abs(right));

				motorC.backward();
			} else {
				motorC.stop();
			}
		} else if (left < 0) {
			motorB.setPower((int) Math.abs(right));

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

		/*
		 * double tD = 0.01; int Tprevious_error = 0; double error = degrees - getM();
		 * double Tderivative = (error - Tprevious_error) / timer; double rcw =
		 * tP*error+tD* Tderivative; while ((int)getM()!=(int)degrees) {
		 * 
		 * 
		 * if (degrees>0) { motorB.backward(); motorC.forward();
		 * 
		 * } else if(degrees<0) { motorB.forward(); motorC.backward(); } else {
		 * arcadeDrive(0,0); }
		 * 
		 * arcadeDrive(0,rcw); }
		 */
	}

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
		while (colorSensor.getColorID() == 1) {
			lastAngle = getM();
			double lightValue = colorSensor.getFloodlight();
			double error = lightValue - offset;
			//dt = 0;
			//integral = integral + error * dt;
			//derivative = (error - lastError) / dt;
			double turn = Kp * error; //+ Ki * integral + Kd * derivative;
			bTurn = Tp + turn;
			cTurn = Tp - turn;
			//lastError = error;
			motorB.setPower(new Double(bTurn).intValue());
			motorB.forward();
			motorC.setPower(new Double(cTurn).intValue());
			motorC.forward();
		}
	}

	private static boolean findLine() {
		float[] sample = new float[colorProvider.sampleSize()];
	    sample[0] = -1;
		while (sample[0] > 0.5) {
			//if (lastAngle > getM())
			//	turn(getM() + 10,0.15);
			//else
			//	turn(getM() - 10,0.15);
			turn(45, 0.15);
			colorProvider.fetchSample(sample, 0);
		}
		return true;
	}
}
