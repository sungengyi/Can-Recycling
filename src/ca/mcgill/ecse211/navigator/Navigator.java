package ca.mcgill.ecse211.navigator;

import ca.mcgill.ecse211.ecse211_project.Project;
import ca.mcgill.ecse211.localizer.LightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class Navigator {
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
	public static final double TILE_SIZE = Project.TILE_SIZE;
	private static final double Island_LL_x = Project.Island_LL_x; // x coordinate of the lower left corner of the
	private static final double Island_LL_y = Project.Island_LL_y; // y coordinate of the lower left corner of the
	private static final double Island_UR_x = Project.Island_UR_x; // x coordinate of the upper right corner of the
	private static final double Island_UR_y = Project.Island_UR_y; // y coordinate of the upper right corner of the
	private static final int corner = Project.corner; // the starting corner
	private static final double LL_x = Project.LL_x; // x coordinate of the lower left corner of the home section
	private static final double LL_y = Project.LL_y; // y coordinate of the lower left corner of the home section
	private static final double UR_x = Project.UR_x; // x coordinate of the upper right corner of the home section
	private static final double UR_y = Project.UR_y; // y coordinate of the upper right corner of the home section
	private static final double TN_LL_x = Project.TN_LL_x; // x coordinate of the lower left of the tunnel
	private static final double TN_LL_y = Project.TN_LL_y; // y coordinate of the lower left of the tunnel
	private static final double TN_UR_x = Project.TN_UR_x; // x coordinate of the upper right of the tunnel
	private static final double TN_UR_y = Project.TN_UR_y; // y coordinate of the upper right of the tunnel
	private static final double SZ_LL_x = Project.SZ_LL_x; // x coordinate of the ring tree
	private static final double SZ_LL_y = Project.SZ_LL_y; // y coordinate of the ring tree
	private static final double SZ_UR_x = Project.SZ_UR_x; // x coordinate of the ring tree
	private static final double SZ_UR_y = Project.SZ_UR_y; // y coordinate of the ring tree
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
	public Navigator( Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.odo = odo;

	}

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
	public void TravelToTunnel(LightLocalizer lightLocal, Odometer odometer) {
		double DES_x = 0;
		double DES_y = 0;
		double DES_angle = 0;
		double LOC_x = 0;
		double LOC_y = 0;
		double CEN_x = 0;
		double CEN_y = 0;
		boolean isTunnelVertical;
		if ((corner == 0 || corner == 3) && TN_UR_x > UR_x)
			isTunnelVertical = false;
		else if ((corner == 1 || corner == 2) && TN_LL_x < LL_x)
			isTunnelVertical = false;
		else
			isTunnelVertical = true;
//case (0,0)
		if(corner == 0) {
			if(isTunnelVertical) {
				DES_x = TN_LL_x-1.5;
				DES_y = TN_LL_y+0.5;
				DES_angle = 90;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.5);
				CEN_y = (LOC_y+0.5);
			}
			else {
				DES_x = TN_LL_x +0.5;
				DES_y = TN_LL_y -1.5;
				DES_angle = 0;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.5);
				CEN_y = (LOC_y+0.5);
			}
		}
		else if (corner == 1) {
			Sound.beep();
			Sound.beep();
			odometer.setXYT(14*TILE_SIZE, TILE_SIZE, 270);
			if(isTunnelVertical) {
				DES_x = TN_UR_x + 1.5;
				DES_y = TN_UR_y - 0.5;
				DES_angle = 270;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.5);
				CEN_y = (LOC_y+0.5);
			}
			else {

				DES_x = TN_LL_x + 0.5;
				DES_y = TN_LL_y - 1.5;
				DES_angle = 0;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x-0.5);
				CEN_y = (LOC_y-0.5);
			}
		}
		else if(corner == 2) {
			odometer.setXYT(14*TILE_SIZE, 8*TILE_SIZE, 180);
			if(isTunnelVertical) {
				DES_x = TN_UR_x + 1.5;
				DES_y = TN_UR_y - 0.5;
				DES_angle = 270;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.5);
				CEN_y = (LOC_y-0.5);
			}
			else {
				DES_x = TN_UR_x - 0.5;
				DES_y = TN_UR_y + 1.5;
				DES_angle = 180;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.5);
				CEN_y = (LOC_y-0.5);
			}
			
		}
		else if(corner == 3) {
			odometer.setXYT(TILE_SIZE, 8*TILE_SIZE, 90);
			if(isTunnelVertical) {
				DES_x = TN_LL_x + 0.5;
				DES_y = TN_LL_y - 1.5;
				DES_angle = 180;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.5);
				CEN_y = (LOC_y-0.5);
			}
			else {
				DES_x = TN_LL_x - 1.5;
				DES_y = TN_LL_y + 0.5;
				DES_angle = 90;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.5);
				CEN_y = (LOC_y+0.5);
			}
		}
		else {
			Sound.buzz();
		}

		//Sound.beep();
		System.out.print("vertical: " + isTunnelVertical);

		System.out.print(DES_x + " destination x");
		System.out.print(DES_y + " destination y");
		System.out.println(DES_angle + " angle");

		double angleA = getDesAngle(DES_x*TILE_SIZE,DES_y*TILE_SIZE);
		turnTo(angleA);
		travelTo(DES_x*TILE_SIZE, DES_y*TILE_SIZE);
		
		turnTo(DES_angle);
		lightLocal.localize();
		odometer.setXYT((LOC_x)*TILE_SIZE, LOC_y*TILE_SIZE, DES_angle);
		double angle_turn = getDesAngle(CEN_x*TILE_SIZE,CEN_y*TILE_SIZE);
		turnTo(angle_turn);
		Sound.beep();
		//Sound.beep();
		//travelTo(0.5*TILE_SIZE, 0.5*TILE_SIZE);
		Sound.beep();
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

}
