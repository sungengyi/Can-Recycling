package ca.mcgill.ecse211.localizer;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer{
	private static Odometer odo = null;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private SampleProvider usDistance;
	private final double TRACK; // The width of the wheel axis
	private final double WHEEL_RAD; //The radius of wheel
	private static final int ROTATE_SPEED = 250; //Speed of the motors
	private static final int WALL_DIST = 30;//Distance from the sensor to the wall
	private static final int THRESHOLD = 5;//Threshold added/subtracted from the wall distance when reading the wall
	private static final int F_CONSTANT1 = 230; //Tweaked constant for falling edge angle conversion
	private static final int F_CONSTANT2 = 50; //Tweaked constant for falling edge angle conversion
	private static final int R_CONSTANT1 = 225; //Constant for rising edge angle conversion
	private static final int R_CONSTANT2 =45; //Constant for rising edge angle conversion
	/**
	 * The constructor initializes fields that being passed to the ultrasonicLocalizer.
	 * @param usDistance
	 * @param odo
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public UltrasonicLocalizer(SampleProvider usDistance, Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) {

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.odo = odo;
		this.usDistance = usDistance;

		leftMotor.setAcceleration(5000);
		rightMotor.setAcceleration(5000);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
	}

/**
 * This method detects each walls and records the angle at which it was detected.
 * Then it calculate the difference between its original heading angle and the real heading 0 angle.
 * 
 */
	public void fallingEdge() {
		double alpha, beta,theta;	
		//If the ultrasonic sensor begins with detecting such a value,
		//the program is going to accidently go through all the loops at once.
		if(getUSData() > WALL_DIST || getUSData() < WALL_DIST + THRESHOLD) {
			accidentHandler();
		
		}
		//Falling edge method usually starts facing away from the wall.
		//If the robot starts facing the wall, it turns until not seeing the wall.
		while(getUSData() < WALL_DIST + THRESHOLD) {
			turnRight();
		}
		//Indication of the process

		//The robot turns left to make a falling angle with the side wall.
		while(getUSData() > WALL_DIST) {
			turnLeft();
		}
		//Record the first falling angle.
		alpha = odo.getT();
		//The robot turns right to make a falling angle with the back wall.
		while(getUSData() < WALL_DIST + THRESHOLD) {
			turnRight();

		}
		//Continue to turn right.
		while(getUSData() >= WALL_DIST ) {
			turnRight();

		}
		//Record the second falling edge angle.
		beta = odo.getT();
		//Stop the motion.
		rightMotor.stop(true);
		leftMotor.stop(false);
		//Calculate the difference in headings.
		theta = getFallingEndAngle(alpha,beta);
		//Add theta to current heading angle.
		odo.update(0, 0, theta);
		//Turn to 0 degree.
		turnTo(0);
		//Prepare for light localization by clearing the odometer readings.
	}	
	/**
	 * This method detects each walls and records the angle at which it was detected.
	 * Then it calculate the difference between its original heading angle and the real heading 0 angle.
	 * 
	 */
	public void risingEdge() {
		double alpha, beta,theta;
		//If the ultrasonic sensor begins with detecting such a value,
		//the program is going to accidently go through all the loops at once.
		if(getUSData() < WALL_DIST || getUSData() > WALL_DIST - THRESHOLD) {
			accidentHandler();
		}
		//Rising edge method usually starts facing the wall.
		//If the robot starts facing away from the wall, it turns until seeing the wall.
		while(getUSData() > WALL_DIST - THRESHOLD) {
			turnLeft();
		}
		//Indication of the process
		//The robot turns right to make a rising angle with the side wall.
		while(getUSData() < WALL_DIST) {
			turnRight();
		}
		//Record the first rising edge angle.
		alpha = odo.getT();
		//The robot turns left to make a rising angle with the side wall.
		while(getUSData() > WALL_DIST - THRESHOLD) 
		{
			turnLeft();
		}
		//Continue to turn left.
		while(getUSData() <= WALL_DIST) {
			turnLeft();
		}
		beta = odo.getT();
		rightMotor.stop(true);
		leftMotor.stop(false);
		theta = getRaisingEndAngle(alpha,beta);
		odo.update(0, 0, theta);
		turnTo(0);
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
	 * This method makes the robot turning right by setting the motor rotatations in different directions.
	 */
	public void turnRight() {
		leftMotor.forward();
		rightMotor.backward();
	}
	
	/**
	 * This method makes the robot turning left by setting the motor rotatations in different directions.
	 */
	public void turnLeft() {
		leftMotor.backward();
		rightMotor.forward();
	}
	/**
	 * 
	 */
	public void accidentHandler() {
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
	}
	/**
	 * This method turns the robot heading to the specified angle by taking
	 * the difference of its original heading and the desired heading.
	 * @param theta
	 */
	public void turnTo(double theta) {
		double position[] = odo.getXYT();
		double dtheta = (360 - position[2] + theta) % 360;
		dtheta = (dtheta > 180) ? dtheta - 360 : dtheta;	
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dtheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dtheta), false);
	}
	/**
	 * This method extracts data from the ultrasonic sensor
	 * @return
	 */
	private int getUSData() {	
		float[] usData = new float [usDistance.sampleSize()];
		usDistance.fetchSample(usData, 0); // acquire data
		int distance =  (int) (usData[0] * 100.0); // extract from buffer, cast to int

		return distance;
	}
	/**
	 * This method calculates the necessary angle difference the robot must turn by using the previously recorded angle
	 * when the robot reads the walls during falling edge.
	 * @param alpha
	 * @param beta
	 * @return
	 */
	private double getFallingEndAngle(double alpha, double beta) {
		return alpha>beta? F_CONSTANT1 - (alpha + beta)/2 : F_CONSTANT2 - (alpha + beta)/2 ;	
	}
	/**
	 * This method calculates the necessary angle difference the robot must turn by using the previously recorded angle
	 * when the robot reads the walls during rising edge.
	 * @param alpha
	 * @param beta
	 * @return
	 */
	private double getRaisingEndAngle(double alpha, double beta) {
		return alpha>beta? R_CONSTANT1 - (alpha + beta)/2 : R_CONSTANT2 - (alpha + beta)/2 ;	
	}
}


