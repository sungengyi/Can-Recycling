package ca.mcgill.ecse211.navigator;
import ca.mcgill.ecse211.ecse211_project.*;
import java.util.Stack;

import ca.mcgill.ecse211.color.ColorData;
import ca.mcgill.ecse211.odometer.Odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ObstacleAvoidance implements Runnable {

	private static Odometer odo = null;
	private boolean obstacle = false;
	private static SampleProvider usDistance = null;
	private static ObstacleAvoidance oa = null;
	private double thetaDesired;//
	private double xDesired, yDesired, preX = 0, preY = 0;
	private int deltax, deltay;
	private double deltaAng = 2;// leaving space for odometer error
	private boolean done; // deciding if the robot reaches waypoint

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static double TRACK;
	private static double WHEEL_RAD = 2.2;

	private static final int block = 20;

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 120;
	private static final double TILE_SIZE = 30.48;
	private static final int ERROR = 5;
	private static int TR;
	private static final long NAVIGATION_PERIOD = 2000;
	private static int[][] arr;
	private static ColorData color = null;

	private int distError = 0; // error (too close/far from wall)

	private static final double Island_LL_x = Project.Island_LL_x; // x coordinate of the lower left corner of the
																	// island
	private static final double Island_LL_y = Project.Island_LL_y; // y coordinate of the lower left corner of the
																	// island
	private static final double Island_UR_x = Project.Island_UR_x; // x coordinate of the upper right corner of the
																	// island
	private static final double Island_UR_y = Project.Island_UR_y; // y coordinate of the upper right corner of the
																	// island

	private static final int corner = Project.corner; // the starting corner
	private static final double LL_x = Project.LL_x; // x coordinate of the lower left corner of the home section
	private static final double LL_y = Project.LL_y; // y coordinate of the lower left corner of the home section
	private static final double UR_x = Project.UR_x; // x coordinate of the upper right corner of the home section
	private static final double UR_y = Project.UR_y; // y coordinate of the upper right corner of the home section
	private static final double TN_LL_x = Project.TN_LL_x; // x coordinate of the lower left of the tunnel
	private static final double TN_LL_y = Project.TN_LL_y; // y coordinate of the lower left of the tunnel
	private static final double TN_UR_x = Project.TN_UR_x; // x coordinate of the upper right of the tunnel
	private static final double TN_UR_y = Project.TN_UR_y; // y coordinate of the upper right of the tunnel



	public ObstacleAvoidance(int[][] arr, Odometer odo, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD) {
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		//		this.WHEEL_RAD = WHEEL_RAD;
		this.arr = arr;
		this.done = false;
	}

	/**
	 * This method is meant to ensure only one instance of the oa is used throughout
	 * the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 * @throws ObstacleAvoidanceException
	 */
	public synchronized static ObstacleAvoidance getOA(int[][] arr, Odometer odo,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws ObstacleAvoidanceException {
		if (oa != null) { // Return existing object
			return oa;
		} else { // create object and return it
			oa = new ObstacleAvoidance(arr, odo, leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return oa;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * @param wheelRad 
	 * @param track2 
	 * @param rightmotor2 
	 * @param leftmotor2 
	 * @param odometer 
	 * @param waypoints 
	 * @return error if no previous odometer exists
	 */
	public synchronized static ObstacleAvoidance getOA(double[][] waypoints, Odometer odometer, EV3LargeRegulatedMotor leftmotor2, EV3LargeRegulatedMotor rightmotor2, double track2, double wheelRad) throws ObstacleAvoidanceException {

		if (oa == null) {
			throw new ObstacleAvoidanceException("No previous Odometer exits.");

		}
		return oa;
	}

	public void updateWaypoints(double x, double y) {

		xDesired = x;
		yDesired = y;
		thetaDesired = getDesAngle();

		this.done = false;

		// is travelTo is called, set isNavigating true

	}

	/**
	 * Compute angle
	 * 
	 * @return
	 */
	public double getDesAngle() {

		double position[] = odo.getXYT();

		// change in x and y in unit of waypoint
		double X = xDesired - position[0];
		double Y = yDesired - position[1];
		// thetaDEsired: angle should achieve
		if (X != 0 && Y != 0) {
			// Converting angles
			// Java math class chooses minimal angle automatically
			thetaDesired = Math.atan2(X, Y);
		} else {
			if (X == 0) {
				thetaDesired = (int) Y < 0 ? Math.PI : 0;

			} else if (Y == 0) {
				thetaDesired = (int) X < 0 ? 3 * Math.PI / 2 : Math.PI / 2;
			}
		}
		thetaDesired = Math.toDegrees(thetaDesired);
		return thetaDesired;
	}
	public double getDesAngle(double x, double y) {

		double position[] = odo.getXYT();

		// change in x and y in unit of waypoint
		double X = x - position[0];
		double Y = y - position[1];
		// thetaDEsired: angle should achieve
		thetaDesired = Math.atan2(X, Y);		
		thetaDesired = (360+Math.toDegrees(thetaDesired))%360;
		return thetaDesired;
	}

	/**
	 * Return true if is at the desired position(waypoint).
	 * @param distance
	 * @return
	 */

	public boolean checkifDone(double[] distance) {

		deltax = Math.abs((int) (xDesired - distance[0]));
		deltay = Math.abs((int) (yDesired - distance[1]));

		// if the current odo position is within 2 cm of the destination point
		return this.done = (deltax < 2 && deltay < 2);
	}

	/**
	 * Return true if is facing the desired angle.
	 * @param angle
	 * @return
	 */
	public boolean isFacingDesAng(double angle) {
		double[] direction = odo.getXYT();
		// leave space for odometer inaccuracy
		deltaAng = 5;
		// If the current angle facing is within some range of the desired angle
		// return true
		return (angle < direction[2] + deltaAng && angle > direction[2] - deltaAng) ? true : false;
	}

	/**
	 * Travel from current position to desired position. Assume already facing the right position.
	 * @param x
	 * @param y
	 */
	public static void travelTo(double x, double y) {
		// is travelTo is called, set isNavigating true
		double position[] = odo.getXYT();
		double xCur = position[0], yCur = position[1];

		// the absolute distance between the initial position and the final position
		double distance = Math.sqrt(Math.pow(x - xCur, 2) + Math.pow(y - yCur, 2));

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);

		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), true);

		while (leftMotor.isMoving() || rightMotor.isMoving()) {

			// take reading
			int size = usDistance.sampleSize();
			float[] usdata = new float[size];
			usDistance.fetchSample(usdata, 0);
			distance = usdata[0] * 100.0;

			// obstacle avoidance
//			if (distance < 10) {
//				selector(odo.getXYT());
//			}
		}

	}

	/**
	 * This method changes the robot's heading. Input theta is positive if it's
	 * turning right, otherwise theta should be nagative.
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {
		// is travelTo is called, set isNavigating true

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

		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		// System.out.println("theta" + (int)theta + "dtheta"+ (int)dtheta);
		// theta should be negative if turnning left
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dtheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dtheta), false);

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

	@Override
	public void run() {
		long updateStart, updateEnd;


		while (true) {
			if (!checkifDone(odo.getXYT())) {
				if (isFacingDesAng(oa.getDesAngle())) {
					oa.travelTo(xDesired, yDesired);
				} else {
					oa.turnTo(oa.getDesAngle());
					oa.travelTo(xDesired, yDesired);
				}

			} else {
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
			}
		}
	}

	



	/**
	 * choose object avoidance mode
	 * @param position
//	 */
//	private void selector(double[] position) {
//		double x, y, t;
//		x = position[0];
//		y = position[1];
//		t = position[2];
//
//		if (t > 315 || t < 45) { // going up
//			if (x < TILE_SIZE) {
//				rightAvoid(); // robot on left side
//			} else {
//				leftAvoid();
//			}
//		} else if(t > 45 && t < 135) { // going right
//			if(y < TILE_SIZE) {
//				leftAvoid(); // robot on lower side
//			} else {
//				rightAvoid(); // robot on upper side
//			}
//		} else if(t > 135 && t < 225) {	// going down
//			if(x < TILE_SIZE) {
//				leftAvoid(); // robot on left side
//			} else {
//				rightAvoid(); // robot on right side
//			}
//		} else if(t > 225 && t < 315) {	// going left
//			if(y < TILE_SIZE) {
//				rightAvoid(); // robot on lower side
//			} else {
//				leftAvoid(); // robot on upper side
//			}
//		} else {
//			// no possible case
//		}
//	}
//
//	/**
//	 * avoid from right
//	 */
//	private void rightAvoid() {
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//
//		leftMotor.setAcceleration(500);
//		rightMotor.setAcceleration(500);
//
//		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
//		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);
//
//		leftMotor.rotate(convertDistance(WHEEL_RAD, block * 1.5), true);
//		rightMotor.rotate(convertDistance(WHEEL_RAD, block * 1.5), false);
//
//		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), true);
//		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), false);
//
//		leftMotor.rotate(convertDistance(WHEEL_RAD, block * 1.5), true);
//		rightMotor.rotate(convertDistance(WHEEL_RAD, block * 1.5), false);
//	}
//
//	/**
//	 * avoid from left
//	 */
//	private void leftAvoid() {
//		leftMotor.stop(true);
//		rightMotor.stop(false);
//
//		leftMotor.setAcceleration(500);
//		rightMotor.setAcceleration(500);
//
//		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), true);
//		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), false);
//
//		leftMotor.rotate(convertDistance(WHEEL_RAD, block * 1.5), true);
//		rightMotor.rotate(convertDistance(WHEEL_RAD, block * 1.5), false);
//
//		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
//		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);
//
//		leftMotor.rotate(convertDistance(WHEEL_RAD, block * 2), true);
//		rightMotor.rotate(convertDistance(WHEEL_RAD, block * 2), false);
//	}
}
