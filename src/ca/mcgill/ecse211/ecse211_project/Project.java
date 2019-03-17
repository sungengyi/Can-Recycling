package ca.mcgill.ecse211.ecse211_project;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.wifiTest.wifiTest;
import ca.mcgill.ecse211.color.Display;
import ca.mcgill.ecse211.localizer.LightLocalizer;
import ca.mcgill.ecse211.localizer.UltrasonicLocalizer;
import ca.mcgill.ecse211.navigator.ObstacleAvoidanceException;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Project {
	// The parameters for driving the robot
		// The island parameters
		public static double Island_LL_x = 0; // x coordinate of the lower left corner of the island
		public static double Island_LL_y = 5; // y coordinate of the lower left corner of the island
		public static double Island_UR_x = 6; // x coordinate of the upper right corner of the island
		public static double Island_UR_y = 8; // y coordinate of the upper right corner of the island

		// The team-specific parameters
		public static int corner = 1; // the starting corner
		public static double LL_x = 2; // x coordinate of the lower left corner of the home section
		public static double LL_y = 0; // y coordinate of the lower left corner of the home section
		public static double UR_x = 8; // x coordinate of the upper right corner of the home section
		public static double UR_y = 3; // y coordinate of the upper right corner of the home section
		public static double TN_LL_x = 2; // x coordinate of the lower left of the tunnel
		public static double TN_LL_y = 3; // y coordinate of the lower left of the tunnel
		public static double TN_UR_x = 3; // x coordinate of the upper right of the tunnel
		public static double TN_UR_y = 5; // y coordinate of the upper right of the tunnel
		public static double T_x = 5; // x coordinate of the can tree
		public static double T_y = 7; // y coordinate of the can tree
	
	//TESTED CONSTANTS, DO NOT CHANGE
	public static final double WHEEL_RAD = 2.1; //Radius of wheel
	public static final double TRACK = 13.7;//Width of wheel axis
	public static final double TILE_SIZE = 30.48;

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port leftLightPort = LocalEV3.get().getPort("S1");
	private static final Port rightLightPort = LocalEV3.get().getPort("S4");
	private static final Port frontUSPort = LocalEV3.get().getPort("S3");
//	private static final Port backUSPort = LocalEV3.get().getPort("S2");


	//package-private
	static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	//static final EV3LargeRegulatedMotor clawMotor =
		//	new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	public static final EV3LargeRegulatedMotor upMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static void main (String [] args) throws OdometerExceptions, InterruptedException, ObstacleAvoidanceException {
		
		Map data = wifiTest.WIFI();
	

		int redTeam = ((Long) data.get("RedTeam")).intValue();
		int greenTeam = ((Long) data.get("GreenTeam")).intValue();

		if (redTeam == 9) {

			corner = ((Long) data.get("RedCorner")).intValue(); // the starting corner
			LL_x = ((Long) data.get("Red_LL_x")).intValue(); // x coordinate of the lower left corner of the home
																// section
			LL_y = ((Long) data.get("Red_LL_y")).intValue(); // y coordinate of the lower left corner of the home
																// section
			UR_x = ((Long) data.get("Red_UR_x")).intValue(); // x coordinate of the upper right corner of the home
																// section
			UR_y = ((Long) data.get("Red_UR_y")).intValue(); // y coordinate of the upper right corner of the home
																// section
			TN_LL_x = ((Long) data.get("TNR_LL_x")).intValue(); // x coordinate of the lower left of the tunnel
			TN_LL_y = ((Long) data.get("TNR_LL_y")).intValue(); // y coordinate of the lower left of the tunnel
			TN_UR_x = ((Long) data.get("TNR_UR_x")).intValue(); // x coordinate of the upper right of the tunnel
			TN_UR_y = ((Long) data.get("TNR_UR_y")).intValue(); // y coordinate of the upper right of the tunnel
			T_x = ((Long) data.get("TR_x")).intValue(); // x coordinate of the can tree
			T_y = ((Long) data.get("TR_y")).intValue(); // y coordinate of the can tree

		} else {

			corner = ((Long) data.get("GreenCorner")).intValue(); // the starting corner
			LL_x = ((Long) data.get("Green_LL_x")).intValue(); // x coordinate of the lower left corner of the home
																// section
			LL_y = ((Long) data.get("Green_LL_y")).intValue(); // y coordinate of the lower left corner of the home
																// section
			UR_x = ((Long) data.get("Green_UR_x")).intValue(); // x coordinate of the upper right corner of the home
																// section
			UR_y = ((Long) data.get("Green_UR_y")).intValue(); // y coordinate of the upper right corner of the home
																// section
			TN_LL_x = ((Long) data.get("TNG_LL_x")).intValue(); // x coordinate of the lower left of the tunnel
			TN_LL_y = ((Long) data.get("TNG_LL_y")).intValue(); // y coordinate of the lower left of the tunnel
			TN_UR_x = ((Long) data.get("TNG_UR_x")).intValue(); // x coordinate of the upper right of the tunnel
			TN_UR_y = ((Long) data.get("TNG_UR_y")).intValue(); // y coordinate of the upper right of the tunnel
			T_x = ((Long) data.get("TG_x")).intValue(); // x coordinate of the can tree
			T_y = ((Long) data.get("TG_y")).intValue(); // y coordinate of the can tree

		}

		Island_LL_x = ((Long) data.get("Island_LL_x")).intValue(); // x coordinate of the lower left corner of the
																	// island
		Island_LL_y = ((Long) data.get("Island_LL_y")).intValue(); // y coordinate of the lower left corner of the
																	// island
		Island_UR_x = ((Long) data.get("Island_UR_x")).intValue(); // x coordinate of the upper right corner of the
																	// island
		Island_UR_y = ((Long) data.get("Island_UR_y")).intValue(); // y coordinate of the upper right corner of the
																	// island

		
		
		int buttonChoice;
		@SuppressWarnings("resource") // Because we don't bother to close this resource
		SensorModes frontUSSensor = new EV3UltrasonicSensor(frontUSPort); // usSensor is the instance
		SampleProvider frontUSDistance = frontUSSensor.getMode("Distance");// usDistance provides samples from

//		@SuppressWarnings("resource") 
//		SensorModes backUSSensor = new EV3UltrasonicSensor(backUSPort); // usSensor is the instance
//		SampleProvider backUSDistance = backUSSensor.getMode("Distance");// usDistance provides samples from

		@SuppressWarnings("resource")
		SensorModes leftLightSensor = new EV3ColorSensor(leftLightPort);//lightSensor is the instance 
		SampleProvider leftLightIntensity = leftLightSensor.getMode("Red");

		@SuppressWarnings("resource")
		SensorModes rightLightSensor = new EV3ColorSensor(rightLightPort);//lightSensor is the instance 
		SampleProvider  rightLightIntensity = rightLightSensor.getMode("Red");

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 

		//Instances of localizers
		LightLocalizer lightLocal = new LightLocalizer(leftLightIntensity, rightLightIntensity, odometer, leftMotor, rightMotor, TRACK, WHEEL_RAD );
		UltrasonicLocalizer usLocal = new UltrasonicLocalizer (frontUSDistance, odometer, leftMotor, rightMotor, TRACK, WHEEL_RAD );
		//ColorData color = new ColorData(rightLightIntensity);
		//Display onto the screen
		Display odometryDisplay = new Display(lcd); // No need to change
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Color  | Field  ", 0, 2);
			lcd.drawString("Identi-|  Test  ", 0, 3);
			lcd.drawString(" fier  |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		lcd.clear();
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		//Odometer display
		Thread odoDisplay = new Thread(odometryDisplay);
		odoDisplay.start();
		usLocal.fallingEdge();
		lightLocal.localize();
		
	}
}