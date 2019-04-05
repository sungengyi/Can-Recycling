package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
public class LightLocalizer{
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private SampleProvider leftLightIntensity;
	private SampleProvider rightLightIntensity;

	private final double TRACK;
	private static final double SENSOR_GAP = 3.2; //The width of wheel axis
	private final double WHEEL_RAD; //The radius of wheel
	private static final int ROTATE_SPEED = 120; //Speed of robot
	/**
	 * Default constructor initializes all fields of LightLocalizer.
	 * @param lightIntensity
	 * @param odo
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public LightLocalizer(SampleProvider leftLightIntensity, SampleProvider rightLightIntensity, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.leftLightIntensity = leftLightIntensity;
		this.rightLightIntensity = rightLightIntensity;
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
	 * This method relocalizes the robot to the correct heading position.
	 * @param xDesired
	 * @param yDesired
	 */	
	public void moveForward(double left, double right) {

		//System.out.println("b1");
//		try {
//			Thread.sleep(500);
//		} catch (InterruptedException e) {
//		}
		leftMotor.rotate(convertDistance(WHEEL_RAD,SENSOR_GAP),true);
		rightMotor.rotate(convertDistance(WHEEL_RAD,SENSOR_GAP),false);
	}
	public void turn(double angle) {
		leftMotor.setSpeed(3*ROTATE_SPEED);
		rightMotor.setSpeed(3*ROTATE_SPEED);
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
	}
	public void correction(double left, double right) {
		boolean leftIsOnLine = false,
				rightIsOnLine = false;

		while(!leftIsOnLine||!rightIsOnLine){
			if(lineDetected(left, leftLightIntensity) && lineDetected(right, rightLightIntensity)) {
				leftIsOnLine = true;
				rightIsOnLine = true;
				leftMotor.stop(true);
				rightMotor.stop();
				break;
			}else {
				if(lineDetected(left, leftLightIntensity) && !leftIsOnLine) {
					//System.out.println("case1");
					
					//System.out.println("c1");
					leftIsOnLine = true;
					leftMotor.stop(true);
					rightMotor.stop();
					if(leftIsOnLine && (rightIsOnLine||lineDetected(right, rightLightIntensity))) {
						leftMotor.stop(true);
						rightMotor.stop();
						break;
					}
					rightMotor.forward();
				}
				if(lineDetected(right, rightLightIntensity) && !rightIsOnLine) {
					//System.out.println("case2");
					rightIsOnLine = true;
					leftMotor.stop(true);
					rightMotor.stop();
					//System.out.println("c2");
					
					
					if(rightIsOnLine && (leftIsOnLine||lineDetected(left, leftLightIntensity))) {
						leftMotor.stop(true);
						rightMotor.stop();
						break;
					}
					leftMotor.forward();
				}
				if(!rightIsOnLine) {
					rightMotor.forward();
				}				
				if(!leftIsOnLine) {
					leftMotor.forward();
				}

				
			}
		}
	}

	/**
	 * This method calculates the robot's x and y position by scanning and detecting for tile lines. Then it moves the robot to head 0 degree.
	 * 
	 */
	public void localize() {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.setAcceleration(5000);
		rightMotor.setAcceleration(5000);
		double left = getLightData(leftLightIntensity);
		double right = getLightData(rightLightIntensity);
		correction(left,right);
		moveForward(left,right);
		turn(90);
		correction(left,right);
		moveForward(left,right);
		turn(-90);
	}
	public boolean lineDetected(double fieldReflection,SampleProvider lightIntensity) {
		if((getLightData(lightIntensity) - fieldReflection) <= - 0.15) {
			return true;
		}else { 
			return false;	

		}
	}
	public boolean onNormalField(double fieldReflection,SampleProvider lightIntensity) {
		if(Math.abs(getLightData(lightIntensity)- fieldReflection) < 0.03){
			return true;
		}else {
			return false;
		}
	}



}