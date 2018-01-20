package cleartable.stupkajr.schaubce;

import java.awt.Point;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class TestClass {

	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;
	static ArrayList<Vector> points = new ArrayList<Vector>();

	static int clock = 0; // Time
	static double angle = 0; // Direction of Travel
	static int speed = 180; // 1 RPM
	static int plowSize = 2017238;
	static long width = 0;
	static long height = 0; 

	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S1);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
	// static EV3UltrasonicSensor sonicSensor = new
	// EV3UltrasonicSensor(SensorPort.S3);
	static SampleProvider colorSample;
	static SampleProvider angleSample;

	// static SampleProvider sonicSample;

	/******
	 * Caitlin:" HEY JOHN HERE ARE SOME CODING STANDARDS FOR YOU TO REMEMBER:
	 * 
	 * 1. CamelCase 2. No Underscores 3. Variables/Methods should start
	 * lowercase"
	 * 
	 * John: "Thanks! <3"
	 * 
	 */

	public static void main(String[] args) {

		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		// finds the area
		searchTable();

		Sound.beep();
		Sound.beep();
		Sound.beep();
		Sound.beep();
		findArea(points);
		// final traverse
		traverseFinalPath();

		leftMotor.stop();
		rightMotor.stop();
		for (int x = 0; x < points.size(); x++) {
			System.out.println(points.get(x));
		}
		System.out.println("Area is " + findArea(points));
		Delay.msDelay(20000);
	}

	private static void searchTable() {
		angleSample = gyroSensor.getAngleMode();
		colorSample = colorSensor.getRGBMode();
		// sonicSample = sonicSensor.getDistanceMode();

		// Zero the Gyro sensor
		gyroSensor.reset();

		while (circling()) {

			float[] color = new float[colorSample.sampleSize()];
			colorSample.fetchSample(color, 0);
			float[] gyroAngle = new float[angleSample.sampleSize()];
			angleSample.fetchSample(gyroAngle, 0);

			// if off the table
			if (color[0] <= 0) {
				angle = gyroAngle[0];
				if (points.isEmpty()) {
					points.add(new Vector(new Point((speed * clock), 0), angle));
				} else {
					Point point = calculateVector(speed, clock, angle);
					points.add(new Vector(point, angle));
				}
				clock = 0;
				// Back from edge after we've saved the points
				float angleChange = backFromEdge();

			}
			clock++;
			// move forward
			leftMotor.forward();
			rightMotor.forward();

			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();
				// End and Give Points, so we can view them
				for (int x = 0; x < points.size(); x++) {
					System.out.println(points.get(x));

				}
				Delay.msDelay(20000);
				System.exit(0);
			}
		}
		leftMotor.stop();
		rightMotor.stop();

	}

	private static boolean circling() {
		// If the angle is less than 360, return true
		return angle < 360 || angle < -360;
	}

	private static long findArea(ArrayList<Vector> points) {
		long maxX = Integer.MIN_VALUE;
		long minX = Integer.MAX_VALUE;

		long maxY = Integer.MIN_VALUE;
		long minY = Integer.MAX_VALUE;

		for (int x = 0; x < points.size(); x++) {
			points.get(x);
			// If our current max, is less than checked x. Current max, becomes
			// that X.
			if (maxX < points.get(x).position.x) {
				maxX = points.get(x).position.x;
			}
			// If our current min, is greater than checked x. Current min,
			// becomes that X.
			if (minX > points.get(x).position.x) {
				minX = points.get(x).position.x;
			}
		}
		for (int y = 0; y < points.size(); y++) {
			points.get(y);
			// If our current max, is less than checked x. Current max, becomes
			// that X.
			if (maxY < points.get(y).position.y) {
				maxY = points.get(y).position.y;
			}
			// If our current min, is greater than checked x. Current min,
			// becomes that X.
			if (minY > points.get(y).position.y) {
				minY = points.get(y).position.y;
			}
		}
		// Estimated area = Width * Height
		 width = maxX + (-1*minX);
		 height = maxY + (-1*minY);

		return (maxX + (-1 * minX)) * (maxY + (-1 * minY));
	}

	private static void traverseFinalPath() {
		boolean dir = true;
		// make a copy to sort the list
		ArrayList<Vector> tempX = new ArrayList<Vector>();
		tempX.addAll(points);
		
		// sort the points by x
		Collections.sort(tempX, new Comparator<Vector>() {
			public int compare(Vector o1, Vector o2) {
				return o1.getPosition().x - o2.getPosition().x;
			}
		});

		//get to the start position
		Point current = points.get(points.size() - 1).getPosition();
		Point start = tempX.get(0).position;
		// start at the left most position 
		driveToPoint(start, current, dir);
		current = start;
		
		int sweeps = (int)Math.ceil(width / (plowSize/2)); 
		System.out.println("width: " + width );
		System.out.println("plow: " + (plowSize) );
		System.out.println("sweeps: " + sweeps);
		int i = 1; 
		do {
			// drive forward
			leftMotor.forward();
			rightMotor.forward();
			//gets sample of the table
			float[] color = new float[colorSample.sampleSize()];
			colorSample.fetchSample(color, 0);
			
			// if off the table
			if (color[0] <= 0) {
				
				leftMotor.stop();
				rightMotor.stop();
				
				Sound.beep();
				Sound.beep();	
				
				leftMotor.backward();
				rightMotor.backward();
			
				
				leftMotor.stop();
				rightMotor.stop();

				//and after it stops, rotate
				int tempWidth = (int)width; 
				if (i % 2 == 0){
					tempWidth= tempWidth * -1; 	
				}
				dir = !dir;
				System.out.println("tempWidth: "+ tempWidth);
				double tempHeight = height; 
				System.out.println("Height is:" + height);
				System.out.println("tempHeight is:" + tempHeight);
				System.out.println("sweeps and i's are: "+ i +" | " + sweeps);
				double sweepRatio = ((double) i) / ((double) sweeps);
				System.out.println("tempHeight: "+ (int) ( sweepRatio * tempHeight) );
				Point nextPos = new Point(tempWidth,  (int) ( sweepRatio* tempHeight) ); 
				
				driveToPoint(nextPos, current, dir);
				current = nextPos; 
				i++; 
			}
			
			//end if button is pushed
			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();
				System.exit(0);
			}
		} while (sweeps >= i);

		// DIM is done.
		leftMotor.stop();
		rightMotor.stop();

	}

	private static float backFromEdge() {
		// stop
		leftMotor.stop();
		rightMotor.stop();

		// turn
		rightMotor.rotate(180);

		// beep to tell it is moving in another direction
		Sound.beep();
		Sound.beep();

		return 90;
	}

	private static void driveToPoint(Point start, Point current, boolean dir) {
		// set up the gyro sensor
		gyroSensor.reset();
		double finalAngle = 0;
		float[] gyroAngle = new float[angleSample.sampleSize()];

		// get the angle between two points
		double deltaY = Math.abs(current.getY()) - Math.abs(start.getY());
		double deltaX = Math.abs(current.getX()) - Math.abs(start.getX());
		double rotate = Math.atan(deltaY / deltaX);

		//System.out.println("angle" + rotate);
		// turn right if dir = true
		if(dir){
			rightMotor.rotate((int) (rotate + angle));
		}else if(!dir){
			leftMotor.rotate((int) (rotate + angle));
		}

		System.out.println(rotate);
		while (finalAngle <= rotate) {
			// confirm angle
			angleSample.fetchSample(gyroAngle, 0);
			finalAngle = gyroAngle[0];
			if(dir){
				rightMotor.rotate((int)finalAngle+10);
			}else if(!dir){
				leftMotor.rotate((int)finalAngle+10);
			}

			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();
				System.exit(0);
			}
		}

	}

	private static Point calculateVector(int speed, int clock, double angle) {
		// speed(Rotation/Minute) * clock (System Time) = Rotations
		int distance = speed * clock;
		float x = getXDistance(distance, angle);
		float y = getYDistance(distance, angle);
		Point p = new Point();
		p.x = (int) x;
		p.y = (int) y;
		return p;
	}

	private static float getYDistance(int distance, double angle) {
		// Distance * cosine(angle) = y-component
		return (float) (distance * Math.cos(Math.toRadians(angle)));
	}

	private static float getXDistance(int distance, double angle) {
		// Distance * sine(angle) = x-component
		return (float) (distance * Math.sin(Math.toRadians(angle)));
	}

}
