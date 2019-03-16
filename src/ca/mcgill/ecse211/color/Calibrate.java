package ca.mcgill.ecse211.color;

import java.text.DecimalFormat;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Calibrate {
	private static final Port lightPort3 = LocalEV3.get().getPort("S1");
	public static float [] COLOR; //Array contains the mean values for RGB
	public static void main(String args []) {
		//-------------------------------------------------------------
		
		//This class was only used before demonstration.
		
		//For getting the Guassian distribution of the 4 colors.
		
		//We record the values from console.
		
		//It was only submitted to show our work.
		
		//-------------------------------------------------------------
		
		@SuppressWarnings("resource")
		SensorModes RGBColor = new EV3ColorSensor(lightPort3);//Color Sensor is the instance 
		SampleProvider  RGB = RGBColor.getMode("RGB"); //RGB mode
		ColorData calibration = new ColorData(RGB);
	
		String str = "GREEN"; //String can be modified when doing calibrating
		LCD.drawString(str, 0, 0);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		COLOR = calibration.calibrate();
		LCD.clear();
		DecimalFormat dec = new DecimalFormat("#####0.00000");	
		for(int j =0;j<3;j++) {
			System.out.println(str+dec.format(COLOR[j]));
		}

	}

}
