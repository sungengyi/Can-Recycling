package ca.mcgill.ecse211.navigator;

import ca.mcgill.ecse211.odometer.Odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class Navigator implements Runnable {
	private int counter = 0;
	private static Navigator navig = null;
	private static Odometer odo = null;

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private boolean isNavigating = false; // deciding if turnTo or Travel to is called
	// It is initially false
	private double xDesired, yDesired;
	private final double TRACK;
	private static double WHEEL_RAD;
	private double thetaDesired;//
	private int deltax, deltay;

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 120;
	private static final long NAVIGATION_PERIOD = 5000;
	private static double[][] arr;

	/**
	 * The default constructor. It initiates all the parameters.
	 * 
	 * @param x
	 * @param y
	 * @param theta
	 * @param leftMotor
	 * @param rightMotor
	 * @param TRACK
	 * @param WHEEL_RAD
	 */
	public Navigator(double[][] waypoints, Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.arr = waypoints;
		this.odo = odo;

	}

	/**
	 * This method is meant to ensure only one instance of the navigator is used
	 * throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Navigator getNavigator(double[][] waypoints, Odometer odo, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD) throws NavigatorException {
		if (navig != null) { // Return existing object
			return navig;
		} else { // create object and return it
			navig = new Navigator(waypoints, odo, leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return navig;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Navigator getNavigator() throws NavigatorException {

		if (navig == null) {
			throw new NavigatorException("No previous Odometer exits.");

		}
		return navig;
	}

	/**
	 * This method makes robot to travel straight in the direction of its heading.
	 * Inputs should be x or y coordinates.
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {
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
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);

//		while (leftMotor.isMoving() || rightMotor.isMoving()) {
//
//			// take reading
//			int size = usDistance.sampleSize();
//			float[] usdata = new float[size];
//			usDistance.fetchSample(usdata, 0);
//			distance = usdata[0] * 100.0;
//
//			// obstacle avoidance
////			if (distance < 10) {
////				selector(odo.getXYT());
////			}
//		}

	}
	
	

	/**
	 * This method changes the robot's heading. Input theta is positive if it's
	 * turning right, otherwise theta should be nagative.
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {
		// is travelTo is called, set isNavigating true
		isNavigating = true;

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
		//System.out.println("theta" + (int)theta + "dtheta"+ (int)dtheta);
		// theta should be negative if turnning left
		
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dtheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dtheta), false);
		
		isNavigating = false;

	}

	/**
	 * Return true if method in Navigator class is called.
	 * 
	 * @return
	 */
	public boolean isNavigating() {
		return isNavigating;
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
	public void updateWaypoints(double x, double y) {

		xDesired = x;
		yDesired = y;
		thetaDesired = getDesAngle();

		// is travelTo is called, set isNavigating true

	}
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

	public boolean checkifDone(double[] distance) {

		deltax = Math.abs((int) (xDesired - distance[0]));
		deltay = Math.abs((int) (yDesired - distance[1]));

		// if the current odo position is within 2 cm of the destination point
		return (deltax < 2 && deltay < 2);
	}
	/**
	 * Run method
	 */
	@Override
	public void run() {
		long updateStart, updateEnd;
		updateStart = System.currentTimeMillis();
		// 5 waypoints

		while (counter < arr.length - 1) {

			// initial position in waypoint unit(1 unit = 1 TILE_SIZE)
			int xInitial = (int) arr[counter][0]; //initial position is set to waypoint in array at counter position
			int yInitial = (int) arr[counter][1];
			// destination waypoint
			int xDesired = (int) arr[counter + 1][0]; //desired position is set to next waypoint
			int yDesired = (int) arr[counter + 1][1];
			counter++;

			// change in x and y in unit of waypoint
			int X = xDesired - xInitial;
			int Y = yDesired - yInitial;
			// thetaDEsired: angle should achieve
			double thetaDesired = 0;

			/**
			 * Calculation logic: thetaChange in the first or third quadrant is positive, in
			 * the second or forth is negative.
			 * 
			 * thetaDesired is 0 if xChange = 0 and yChange < 0 thetaDesired is pi if
			 * xChange = 0 and yChange > 0 thetaDesired is 3/2 * pi if yChange = 0 and
			 * xChange < 0 thetaDesired is 1/2 * pi if yChange = 0 and xChange > 0
			 */
			if (X != 0 && Y != 0) {
				// Converting angles
				// Java math class chooses minimal angle automatically
				thetaDesired = Math.atan2(X, Y);
			} else {
				if (X == 0) { // move vertically
					thetaDesired = (Y < 0) ? Math.PI : 0;
				} else if (Y == 0) { // move horizontally
					thetaDesired = (X < 0) ? 3 * Math.PI / 2 : Math.PI / 2;
				} else {

				}
			}
			
			navig.turnTo(Math.toDegrees(thetaDesired));
			navig.travelTo(xDesired, yDesired);

			// Wait for codes to be executed
			// Period could be adjusted
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < NAVIGATION_PERIOD) {
				try {
					Thread.sleep(NAVIGATION_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}
}
