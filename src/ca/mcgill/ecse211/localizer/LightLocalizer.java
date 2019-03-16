package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer{
	private static Odometer odo = null;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private SampleProvider leftLightIntensity;
	private SampleProvider rightLightIntensity;

	private final double TRACK; //The width of wheel axis
	private final double WHEEL_RAD; //The radius of wheel
	private final static double CAR_LENGTH = 12.5; //Distance from center of rotation to light sensor
	private final static double TILE_SIZE = 30.48; //Distance from center of rotation to light sensor

	private static final int ROTATE_SPEED = 100; //Speed of robot
	private static double CIRCUMFERENCE; //Circumferencce of wheel axis
	/**
	 * Default constructor initializes all fields of LightLocalizer.
	 * @param lightIntensity
	 * @param odo
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public LightLocalizer(SampleProvider leftLightIntensity, SampleProvider rightLightIntensity, Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		LightLocalizer.odo = odo;
		this.leftLightIntensity = leftLightIntensity;
		this.rightLightIntensity = rightLightIntensity;
		CIRCUMFERENCE = Math.PI * TRACK;
		leftMotor.setAcceleration(3000);
		rightMotor.setAcceleration(3000);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
	}
	/**
	 * This method comes from SquareDriver class. It converts the desired distance
	 * to the degrees of rotation of the motor.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * This method comes from SquareDriver class. It converts the desired turning
	 * angle to the degrees of rotation of the motor.
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**
	 * Extracts data from the light sensor.
	 * @return
	 */
	public double getLightData(SampleProvider lightIntensity ) {
		float [] data = new float[lightIntensity.sampleSize()];
		double intensity;
		lightIntensity.fetchSample(data, 0);
		intensity = data[0];
		return intensity;
	}
	/**
	 * This method turns the robot heading to the specified angle by taking
	 * the difference of its original heading and the desired heading.
	 * @param theta
	 */
	public void turnTo(double theta) {
		double position[] = odo.getXYT();
		/**
		 * use (720 - theta + thetaDesired)%360 ex: current theta = 300, desired theta
		 * 270 --> turn left 30 degrees compute: (720 - 300 + 270)%360 = 330, 330>180,
		 * 330 - 360 = -30 ex: cur 0, desired 45 --> turn right 45 degrees compute:
		 * (720-0+45)%360 = 45 if result > 180 , turn (theta - 360) if result < 180 ,
		 * turn theta
		 */

		double dtheta = (720 - position[2] + theta) % 360;
		dtheta = (dtheta > 180) ? dtheta - 360 : dtheta;		
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dtheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dtheta), false);
	}
	/**
	 * This method relocalizes the robot to the correct heading position.
	 * @param xDesired
	 * @param yDesired
	 */
	public void relocalize(double xDesired, double yDesired) {
		double changeInX = xDesired - odo.getX();
		double changeInY = yDesired - odo.getY();
		double thetaDesired;
		thetaDesired = (360 + Math.toDegrees(Math.atan2(changeInX, changeInY)))%360;
		turnTo(thetaDesired);
		travelTo(xDesired,yDesired);
	}
	public void travelTo(double xDesired, double yDesired) {
		// is travelTo is called, set isNavigating true

		double xCur = odo.getX(), yCur = odo.getY();

		double distance = Math.sqrt(Math.pow(xDesired - xCur, 2) + Math.pow(yDesired - yCur, 2));
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

	}

	/**
	 * This method calculates the robot's x and y position by scanning and detecting for tile lines. Then it moves the robot to head 0 degree.
	 * 
	 */
	public void localize() {
		// preData1 is the sensor on the left side of the robot
		double preData1 = getLightData(leftLightIntensity);//Gets the tile reflection intensity.
		//Robot moves forward.
		do {
			leftMotor.forward();
			rightMotor.forward();	
			//System.out.println("forward");
		}while(getLightData(leftLightIntensity) > 0.26 && getLightData(rightLightIntensity) > 0.26);
		//!lineDetected(preData1,leftLightIntensity)&&!lineDetected(preData1,rightLightIntensity)
		//When it detected a line, it returns by distance between the light sensor and center of rotation.
		System.out.println("forward done");
//		if(lineDetected(preData1,leftLightIntensity)&&!lineDetected(preData1,rightLightIntensity)) {
//			leftMotor.stop();
//			System.out.println("turn left");
//
//			do {
//				rightMotor.forward();	
//			}while(!lineDetected(preData1,leftLightIntensity));
//			//leftMotor.forward();
//
//		}
		while(lineDetected(preData1,leftLightIntensity)&&!lineDetected(preData1,rightLightIntensity)) {
			leftMotor.stop(false);
			rightMotor.forward();	
		}
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//		if(!lineDetected(preData1,leftLightIntensity)&&lineDetected(preData1,rightLightIntensity) ) {
//			rightMotor.stop();
//			System.out.println("turn right");
//
//			do {
//				leftMotor.forward();	
//			}while(!lineDetected(preData1,leftLightIntensity));
//			rightMotor.forward();
//
//		}else {
//			
//		}
//		if(lineDetected(preData1,leftLightIntensity)&&lineDetected(preData1,rightLightIntensity)) {
//			System.out.println("turn 90");
//
//			turnTo(90);
//		}
//		
//		do {
//			leftMotor.forward();
//			rightMotor.forward();	
//		}while(!lineDetected(preData1,leftLightIntensity)&&!lineDetected(preData1,rightLightIntensity));
//		//When it detected a line, it returns by distance between the light sensor and center of rotation.
//		
//		if(lineDetected(preData1,leftLightIntensity)&&!lineDetected(preData1,rightLightIntensity)) {
//			leftMotor.stop();
//			do {
//				rightMotor.forward();	
//			}while(!lineDetected(preData1,leftLightIntensity)&&!lineDetected(preData1,rightLightIntensity));
//		}
//		if(!lineDetected(preData1,leftLightIntensity)&&lineDetected(preData1,rightLightIntensity) ) {
//			rightMotor.stop();
//			do {
//				leftMotor.forward();	
//			}while(!lineDetected(preData1,leftLightIntensity)&&!lineDetected(preData1,rightLightIntensity) );
//		}
//		if(lineDetected(preData1,leftLightIntensity)&&lineDetected(preData1,rightLightIntensity)) {
//			turnTo(-90);
//		}
//		
	}
	public boolean lineDetected(double data,SampleProvider lightIntensity) {
		if(data -  getLightData(lightIntensity) <= 0.3) {
			System.out.print("true");

			return true;
		}else { 
			System.out.print("fasle");

			return false;	
		}
	}
	/**
	 * This method sets the initial position on odometer.
	 * @param SC
	 * @param x
	 * @param y
	 * @param theta
	 */
	public void setInitialPosition(int SC, double x, double y, double theta) {
		switch(SC) {
		case 0:
			odo.setX(TILE_SIZE + x);
			odo.setY(TILE_SIZE + y);
			break;
		case 1:
			odo.setX(7*TILE_SIZE + x);
			odo.setY(TILE_SIZE + y);
			break;
		case 2:

			odo.setX(7*TILE_SIZE + x);
			odo.setY(7*TILE_SIZE + y);
			break;
		case 3:
			odo.setX(TILE_SIZE + x);
			odo.setY(7*TILE_SIZE + y);
			break;
		}
		odo.setTheta(theta);
		turnTo(0);
		odo.setTheta(0);
	}

	/**
	 * Calculate the x position.
	 * @param line2
	 * @param line4
	 * @return
	 */
	public double getFinalX(int line2, int line4) {
		double theta = 2 * Math.PI * (((line4 - line2)/360.0)*2 * Math.PI * WHEEL_RAD )/CIRCUMFERENCE;
		double result = -CAR_LENGTH * Math.cos(theta/2);
		return result;
	}
	/**
	 * Calculates the y position.
	 * @param line1
	 * @param line3
	 * @return
	 */
	public double getFinalY(int line1, int line3) {
		double theta = 2 * Math.PI* (((line3 - line1)/360.0) *2* Math.PI * WHEEL_RAD )/CIRCUMFERENCE;
		double result = -CAR_LENGTH * Math.cos(theta/2);
		return result;
	}
}