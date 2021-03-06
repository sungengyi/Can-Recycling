package ca.mcgill.ecse211.navigator;

import ca.mcgill.ecse211.color.ColorData;
import ca.mcgill.ecse211.ecse211_project.Project;
import ca.mcgill.ecse211.localizer.LightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


public class Navigator {
	private int counter = 0;
	private static Navigator navig = null;
	private static Odometer odo = null;

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private boolean isNavigating = false; // deciding if turnTo or Travel to is called

	public static final double CAN_RADIUS = 5.5; // this is the offset from the 2 line-detecting light sensors to the wheel
	public static double HIT_GAP =0; //Radius of wheel
	public static boolean HEAVY = false;

	// It is initially false
	private double xDesired, yDesired;
	private final double TRACK;
	private static double WHEEL_RAD;
	private double thetaDesired;//
	private int deltax, deltay;
	private static final int FORWARD_SPEED = 320;

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
	public Navigator(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.odo = odo;

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.setAcceleration(5000);
		rightMotor.setAcceleration(5000);


	}

	public void travelTo(double x, double y) {
		// is travelTo is called, set isNavigating true
		double position[] = odo.getXYT();
		double xCur = position[0], yCur = position[1];

		// the absolute distance between the initial position and the final position
		double distance = Math.sqrt(Math.pow(x - xCur, 2) + Math.pow(y - yCur, 2));

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

		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dtheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dtheta), false);

		isNavigating = false;

	}
	public void TravelToTunnel(LightLocalizer lightLocal) {
		double DES_x = 0;
		double DES_y = 0;
		double DES_angle = 0;
		double LOC_x = 0;
		double LOC_y = 0;
		double CEN_x = 0;
		double CEN_y = 0;
		double DEN_x = 0;
		double DEN_y = 0;		
		boolean isTunnelVertical;

		if ((corner == 0 || corner == 3) && TN_UR_x > UR_x)
			isTunnelVertical = false;
		else if ((corner == 1 || corner == 2) && TN_LL_x < LL_x)
			isTunnelVertical = false;
		else
			isTunnelVertical = true;
		//case (0,0)
		if(corner == 0) {

			odo.setXYT(TILE_SIZE, TILE_SIZE, 0);

			if(!isTunnelVertical) {

				DES_x = TN_LL_x-1.5;
				DES_y = TN_LL_y+0.5;
				DES_angle = 90;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.45);
				CEN_y = (LOC_y+0.61);
				DEN_x = TN_UR_x+0.5;
				DEN_y = TN_UR_y-0.5;
			}
			else {
				DES_x = TN_LL_x +0.5;
				DES_y = TN_LL_y -1.5;
				DES_angle = 0;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.61);
				CEN_y = (LOC_y+0.45);
				DEN_x = TN_UR_x-0.5;
				DEN_y = TN_UR_y+0.5;
			}
		}
		else if (corner == 1) {
			odo.setXYT(14*TILE_SIZE, TILE_SIZE, 270);
			if(!isTunnelVertical) {
				DES_x = TN_UR_x + 1.5;
				DES_y = TN_UR_y - 0.5;
				DES_angle = 270;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.5);
				CEN_y = (LOC_y-0.5);
				DEN_x = TN_LL_x-0.5;
				DEN_y = TN_LL_y+0.5;
			}
			else {

				DES_x = TN_LL_x +0.5;
				DES_y = TN_LL_y -1.5;
				DES_angle = 0;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.5);
				CEN_y = (LOC_y+0.5);
				DEN_x = TN_UR_x-0.5;
				DEN_y = TN_UR_y+0.5;

			}
		}
		else if(corner == 2) {

			odo.setXYT(14*TILE_SIZE, 8*TILE_SIZE, 180);
			if(isTunnelVertical) {
				DES_x = TN_UR_x + 1.5;
				DES_y = TN_UR_y - 0.5;
				DES_angle = 270;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.5);
				CEN_y = (LOC_y-0.5);
				DEN_x = TN_LL_x-0.5;
				DEN_y = TN_LL_y+0.5;
			}
			else {
				DES_x = TN_UR_x - 0.5;
				DES_y = TN_UR_y + 1.5;
				DES_angle = 180;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.5);
				CEN_y = (LOC_y-0.5);
				DEN_x = TN_LL_x+0.5;
				DEN_y = TN_LL_y-0.5;
			}

		}
		else if(corner == 3) {
			odo.setXYT(TILE_SIZE, 8*TILE_SIZE, 90);
			if(isTunnelVertical) {
				DES_x = TN_UR_x - 0.5;
				DES_y = TN_UR_y + 1.5;
				DES_angle = 180;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.5);
				CEN_y = (LOC_y-0.5);
				DEN_x = TN_LL_x + 0.5;
				DEN_y = TN_LL_y - 0.5;
			}
			else {
				DES_x = TN_LL_x - 1.5;
				DES_y = TN_LL_y + 0.5;
				DES_angle = 90;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.5);
				CEN_y = (LOC_y+0.5);
				DEN_x = TN_UR_x+0.5;
				DEN_y = TN_UR_y-0.5;
			}
		}
		else {
		}


		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		turnTo(getDesAngle(DES_x*TILE_SIZE,DES_y*TILE_SIZE));
		travelTo(DES_x*TILE_SIZE, DES_y*TILE_SIZE);

		turnTo(DES_angle);
		lightLocal.localize();
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.setAcceleration(7000);
		rightMotor.setAcceleration(7000);
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		
		odo.setXYT((LOC_x)*TILE_SIZE, LOC_y*TILE_SIZE, DES_angle);



		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		turnTo(getDesAngle(CEN_x*TILE_SIZE,CEN_y*TILE_SIZE));
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		travelTo(CEN_x*TILE_SIZE, CEN_y*TILE_SIZE);
		odo.setX(CEN_x*TILE_SIZE);
		odo.setY(CEN_y*TILE_SIZE);

		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		turnTo(getDesAngle(DEN_x*TILE_SIZE,DEN_y*TILE_SIZE));
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	//	Button.waitForAnyPress();
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		travelTo(DEN_x*TILE_SIZE, DEN_y*TILE_SIZE);
		odo.setX(DEN_x*TILE_SIZE);
		odo.setY(DEN_y*TILE_SIZE);
	}
	public void TravelToSearchArea() {
		double des_x  = 0;
		double des_y = 0;
		des_x = SZ_LL_x + (SZ_UR_x-SZ_LL_x)/2;
		des_y = SZ_LL_y + (SZ_UR_y - SZ_LL_y)/2;
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		turnTo(getDesAngle(des_x*TILE_SIZE,des_y*TILE_SIZE));
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		travelTo(des_x*TILE_SIZE,des_y*TILE_SIZE);
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		odo.setX(des_x*TILE_SIZE);
		odo.setY(des_y*TILE_SIZE);
	}
	public void TravelToBase() {
		double des_x = 0;
		double des_y = 0;
		if(corner == 0) {
			des_x = LL_x+1;
			des_y = LL_y+1;
		}
		else if(corner == 1) {
			des_x = UR_x-1;
			des_y = LL_y+1;
		}
		else if(corner == 2) {
			des_x = UR_x-1;
			des_y = UR_y-1;
		}
		else if(corner == 3) {
			des_x = LL_x+1;
			des_y = UR_y-1;
		}
		else {
			System.out.print(" might as well die ");
		}
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		turnTo(getDesAngle(SZ_LL_x*des_x,des_y*TILE_SIZE));
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		travelTo(des_x*TILE_SIZE,des_y*TILE_SIZE);
	}
	public void TravelBackToTunnel(LightLocalizer lightLocal) {
		double DES_x = 0;
		double DES_y = 0;
		double DES_angle = 0;
		double LOC_x = 0;
		double LOC_y = 0;
		double CEN_x = 0;
		double CEN_y = 0;
		double DEN_x = 0;
		double DEN_y = 0;		
		boolean isTunnelVertical;

		if ((corner == 0 || corner == 3) && TN_UR_x > UR_x)
			isTunnelVertical = false;
		else if ((corner == 1 || corner == 2) && TN_LL_x < LL_x)
			isTunnelVertical = false;
		else
			isTunnelVertical = true;
		//case (0,0)
if(corner == 0) {
			
		
			if(!isTunnelVertical) {
				DES_x = TN_UR_x+1.5;
				DES_y = TN_UR_y-0.5;
				DES_angle = 270;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.45);
				CEN_y = (LOC_y-0.61);
				DEN_x = TN_LL_x-0.5;
				DEN_y = TN_LL_y+0.5;
			}
			else {
				DES_x = TN_UR_x -0.5;
				DES_y = TN_UR_y +1.5;
				DES_angle = 180;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.61);
				CEN_y = (LOC_y-0.45);
				DEN_x = TN_UR_x+0.5;
				DEN_y = TN_UR_y-0.5;
			}
		}
		else if (corner == 1) {
			
			
			if(!isTunnelVertical) {
				DES_x = TN_LL_x - 1.5;
				DES_y = TN_LL_y + 0.5;
				DES_angle = 90;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.45);
				CEN_y = (LOC_y+0.61);
				DEN_x = TN_UR_x+0.5;
				DEN_y = TN_UR_y-0.5;
			}
			else {

				DES_x = TN_UR_x -0.5;
				DES_y = TN_UR_y +1.5;
				DES_angle = 180;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.61);
				CEN_y = (LOC_y-0.45);
				DEN_x = TN_LL_x+0.5;
				DEN_y = TN_LL_y-0.5;
				
			}
		}
		else if(corner == 2) {
		
			
			if(!isTunnelVertical) {
				DES_x = TN_LL_x - 1.5;
				DES_y = TN_LL_y + 0.5;
				DES_angle = 90;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y - 0.5;
				CEN_x = (LOC_x+0.45);
				CEN_y = (LOC_y+0.61);
				DEN_x = TN_UR_x+0.5;
				DEN_y = TN_UR_y-0.5;
			}
			else {
				DES_x = TN_LL_x + 0.5;
				DES_y = TN_LL_y - 1.5;
				DES_angle = 0;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.61);
				CEN_y = (LOC_y+0.45);
				DEN_x = TN_UR_x-0.5;
				DEN_y = TN_UR_y+0.5;
			}
			
		}
		else if(corner == 3) {
			
			if(isTunnelVertical) {
				DES_x = TN_LL_x + 0.5;
				DES_y = TN_LL_y - 1.5;
				DES_angle = 0;
				LOC_x = DES_x + 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.61);
				CEN_y = (LOC_y+0.45);
				DEN_x = TN_UR_x - 0.5;
				DEN_y = TN_UR_y + 0.5;
			}
			else {
				DES_x = TN_UR_x + 1.5;
				DES_y = TN_UR_y - 0.5;
				DES_angle = 270;
				LOC_x = DES_x - 0.5;
				LOC_y = DES_y + 0.5;
				CEN_x = (LOC_x-0.45);
				CEN_y = (LOC_y-0.61);
				DEN_x = TN_LL_x-0.5;
				DEN_y = TN_LL_y+0.5;
			}
		}
		else {
		}
	try {
		Thread.sleep(200);
	} catch (InterruptedException e1) {
		// TODO Auto-generated catch block
		e1.printStackTrace();
	}
		turnTo(getDesAngle(DES_x*TILE_SIZE,DES_y*TILE_SIZE));
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		travelTo(DES_x*TILE_SIZE, DES_y*TILE_SIZE);

		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		turnTo(DES_angle);
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		lightLocal.localize();
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}
		odo.setXYT((LOC_x)*TILE_SIZE, LOC_y*TILE_SIZE, DES_angle);
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		turnTo(getDesAngle(CEN_x*TILE_SIZE,CEN_y*TILE_SIZE));
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		travelTo(CEN_x*TILE_SIZE, CEN_y*TILE_SIZE);
		odo.setX(CEN_x*TILE_SIZE);
		odo.setY(CEN_y*TILE_SIZE);
	
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		turnTo(getDesAngle(DEN_x*TILE_SIZE,DEN_y*TILE_SIZE));
		leftMotor.setSpeed(FORWARD_SPEED+60);
		rightMotor.setSpeed(FORWARD_SPEED+60);
		//Button.waitForAnyPress();
		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		travelTo(DEN_x*TILE_SIZE, DEN_y*TILE_SIZE);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		odo.setX(DEN_x*TILE_SIZE);
		odo.setY(DEN_y*TILE_SIZE);
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

	public void SearchAndGrabTest(EV3LargeRegulatedMotor ultraMotor,EV3LargeRegulatedMotor upMotor,ColorData color, SampleProvider sensor) throws InterruptedException {

		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.setAcceleration(10000);
		rightMotor.setAcceleration(10000);
		upMotor.setSpeed(200);

		//find the can and stop in front of it
		detectForCans((SZ_UR_x-SZ_LL_x)*TILE_SIZE, sensor);
		do {
			leftMotor.forward();
			rightMotor.forward();
		}while(getUSData(sensor)<10000);

		leftMotor.stop(true);
		rightMotor.stop();

		weightingTest(ultraMotor, sensor);
		ultraMotor.setSpeed(100);
		detectForCans(15, sensor);


		//go back for claw to grab
		leftMotor.rotate((convertDistance(WHEEL_RAD,-CAN_RADIUS)),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,-CAN_RADIUS)),false);

		//open
		ultraMotor.rotate(100);

		//go forward
		leftMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + CAN_RADIUS))),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + CAN_RADIUS))),false);

		//identify
		color.identify(HEAVY,upMotor);

		leftMotor.rotate((convertDistance(WHEEL_RAD,CAN_RADIUS)),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,CAN_RADIUS)),false);
		//grab
		ultraMotor.rotate(-100);

		leftMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + CAN_RADIUS))),true);
		rightMotor.rotate((convertDistance(WHEEL_RAD,(HIT_GAP + CAN_RADIUS))),false);
	}
	public void weightingTest(EV3LargeRegulatedMotor ultraMotor, SampleProvider sensor) {
		ultraMotor.setSpeed(2000);
		ultraMotor.setAcceleration(20000);
		ultraMotor.rotate(20);
		HIT_GAP = getUSData(sensor);
		ultraMotor.rotate(-20);

		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
		}


		if(HIT_GAP < 5)  HEAVY=true;
		else if(HIT_GAP > 15) HEAVY = false; HIT_GAP = 10;
	}
	public void detectForCans(double range,SampleProvider sensor) throws InterruptedException {

		leftMotor.forward();
		rightMotor.backward();
		long start, end;
		double period = 50;
		double distance;
		boolean detected = false;
		boolean switchs = true;
		int counter = 0;
		while(range > 5 && !detected) {
			start = System.currentTimeMillis();
			distance = getUSData(sensor);
			if(distance>range) {
				leftMotor.forward();
				rightMotor.backward();
			}
			if(distance < 5) {
				leftMotor.stop(true);
				rightMotor.stop();
				leftMotor.rotate(convertDistance(WHEEL_RAD, distance/2), true);
				rightMotor.rotate(convertDistance(WHEEL_RAD, distance/2), false);
				detected = true;
			}
			if(distance <= range && !detected) {
				leftMotor.stop(true);
				rightMotor.stop();
				leftMotor.rotate(convertDistance(WHEEL_RAD, distance/2), true);
				rightMotor.rotate(convertDistance(WHEEL_RAD, distance/2), false);
				range /= 2;
			}
			end = System.currentTimeMillis();
			if(end - start < period) {
				Thread.sleep((long) (period-end+start));

			}


		}
	}
	public void releaseCan(EV3LargeRegulatedMotor ultraMotor) {
		ultraMotor.rotate(100);

		leftMotor.rotate(-(convertDistance(WHEEL_RAD,(HIT_GAP + 2*CAN_RADIUS))),true);
		rightMotor.rotate(-(convertDistance(WHEEL_RAD,(HIT_GAP + 2*CAN_RADIUS))),false);
	}
	private static int getUSData(SampleProvider usDistance) {	
		float[] usData = new float [usDistance.sampleSize()];
		usDistance.fetchSample(usData, 0); // acquire data
		int distance =  (int) (usData[0] * 100.0); // extract from buffer, cast to int
		return distance;
	}



}
