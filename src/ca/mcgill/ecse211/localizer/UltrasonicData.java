package ca.mcgill.ecse211.localizer;
import java.io.*;
import lejos.hardware.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class UltrasonicData {
	public  static void main(String [] args) throws InterruptedException, FileNotFoundException, UnsupportedEncodingException {
		PrintWriter writer = new PrintWriter("data.csv", "UTF-8");
		EV3UltrasonicSensor sensor = new EV3UltrasonicSensor(SensorPort.S3);
		SampleProvider usSensor = sensor.getMode("Distance");
		float [] usData = new float[usSensor.sampleSize()];
		int distance;
		try {
			while(true) {
				usSensor.fetchSample(usData, 0);
				distance =(int) (usData[0] * 100.0);
				System.out.print(String.format("%d:%d%n",System.currentTimeMillis(),distance));
			}

		}finally {
			writer.close();
			sensor.close();
		}

		}

	}
