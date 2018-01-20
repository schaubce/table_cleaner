package cleartable.stupkajr.schaubce;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class ClearTable {

	static RegulatedMotor leftMotor = Motor.B;
	static RegulatedMotor rightMotor = Motor.C;
	static USSensor sensor;

	public static void introMessage() {

		GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
		g.drawString("ClearTable Program", 5, 0, 0);
		g.setFont(Font.getSmallFont());
		g.drawString("Demonstration of the clearing", 2, 20, 0);
		g.drawString("a table of cups. Requires", 2, 30, 0);
		g.drawString("a wheeled vehicle with two", 2, 40, 0);
		g.drawString("independently controlled", 2, 50, 0);
		g.drawString("motors connected to motor", 2, 60, 0);
		g.drawString("ports B and C, and an", 2, 70, 0);
		g.drawString("Ultra Sonic Sensor connected", 2, 80, 0);
		g.drawString("to port 1.", 2, 90, 0);

		// Quit GUI button:
		g.setFont(Font.getSmallFont()); // can also get specific size using
										// Font.getFont()
		int y_quit = 100;
		int width_quit = 45;
		int height_quit = width_quit / 2;
		int arc_diam = 6;
		g.drawString("QUIT", 9, y_quit + 7, 0);
		g.drawLine(0, y_quit, 45, y_quit); // top line
		g.drawLine(0, y_quit, 0, y_quit + height_quit - arc_diam / 2); // left
																		// line
		g.drawLine(width_quit, y_quit, width_quit, y_quit + height_quit / 2); // right
																				// line
		g.drawLine(0 + arc_diam / 2, y_quit + height_quit, width_quit - 10,
				y_quit + height_quit); // bottom line
		g.drawLine(width_quit - 10, y_quit + height_quit, width_quit, y_quit
				+ height_quit / 2); // diagonal
		g.drawArc(0, y_quit + height_quit - arc_diam, arc_diam, arc_diam, 180,
				90);

		// Enter GUI button:
		g.fillRect(width_quit + 10, y_quit, height_quit, height_quit);
		g.drawString("GO", width_quit + 15, y_quit + 7, 0, true);

		Button.waitForAnyPress();
		if (Button.ESCAPE.isDown())
			System.exit(0);
		g.clear();
	}

	public static void main(String[] args) {

		Sound.beep();
		introMessage();

		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		leftMotor.rotateTo(0);
		rightMotor.rotateTo(0);
		leftMotor.setSpeed(400);
		rightMotor.setSpeed(400);
		leftMotor.setAcceleration(800);
		rightMotor.setAcceleration(800);
		sensor = new USSensor();
		sensor.setDaemon(true);
		sensor.start();
		Behavior b1 = new DriveForward();
		Behavior b2 = new DetectWall();
		Behavior[] behaviorList = { b1, b2 };
		Arbitrator arbitrator = new Arbitrator(behaviorList);
		LCD.drawString("ClearTable Program", 0, 1);
		Button.LEDPattern(6);
		Button.waitForAnyPress();
		arbitrator.start();

	}

}

class USSensor extends Thread {
	EV3UltrasonicSensor us = new EV3UltrasonicSensor(SensorPort.S1);
	SampleProvider sp = us.getDistanceMode();
	public int control = 0;
	public int distance = 255;

	USSensor() {

	}

	public void run() {
		while (true) {
			float[] sample = new float[sp.sampleSize()];
			control = 0;
			sp.fetchSample(sample, 0);
			distance = (int) sample[0];
			System.out
					.println("Control: " + control + " Distance: " + distance);

		}

	}
}

class DriveForward implements Behavior {

	private boolean _suppressed = false;

	public boolean takeControl() {
		if (Button.readButtons() != 0) {
			_suppressed = true;
			ClearTable.leftMotor.stop();
			ClearTable.rightMotor.stop();
			Button.LEDPattern(6);
			Button.discardEvents();
			System.out.println("Button pressed");
			if ((Button.waitForAnyPress() & Button.ID_ESCAPE) != 0) {
				Button.LEDPattern(0);
				System.exit(1);
			}
			System.out.println("Button pressed 2");
			Button.waitForAnyEvent();
			System.out.println("Button released");
		}
		return true; // this behavior always wants control.
	}

	public void suppress() {
		_suppressed = true;// standard practice for suppress methods
	}

	public void action() {
		_suppressed = false;
		// EV3BumperCar.leftMotor.forward();
		// EV3BumperCar.rightMotor.forward();
		while (!_suppressed) {
			// EV3BumperCar.leftMotor.forward();
			// EV3BumperCar.rightMotor.forward();
			switch (ClearTable.sensor.control) {
			case 0:
				ClearTable.leftMotor.setSpeed(400);
				ClearTable.rightMotor.setSpeed(400);
				ClearTable.leftMotor.stop(true);
				ClearTable.rightMotor.stop(true);
				break;
			case 1:
				ClearTable.leftMotor.setSpeed(400);
				ClearTable.rightMotor.setSpeed(400);
				ClearTable.leftMotor.forward();
				ClearTable.rightMotor.forward();
				break;
			case 2:
				ClearTable.leftMotor.backward();
				ClearTable.rightMotor.backward();
				break;
			case 3:
				ClearTable.leftMotor.setSpeed(200);
				ClearTable.rightMotor.setSpeed(200);
				ClearTable.leftMotor.forward();
				ClearTable.rightMotor.backward();
				break;
			case 4:
				ClearTable.leftMotor.setSpeed(200);
				ClearTable.rightMotor.setSpeed(200);
				ClearTable.leftMotor.backward();
				ClearTable.rightMotor.forward();
				break;

			}
			Thread.yield(); // don't exit till suppressed
		}
		// EV3BumperCar.leftMotor.stop(true);
		// EV3BumperCar.rightMotor.stop(true);
		// EV3BumperCar.leftMotor.
	}
}

class DetectWall implements Behavior {

	public DetectWall() {
		// touch = new TouchSensor(SensorPort.S1);
		// sonar = new UltrasonicSensor(SensorPort.S3);
	}

	private boolean checkDistance() {

		int dist = ClearTable.sensor.distance;
		if (dist < 30) {
			Button.LEDPattern(2);
			return true;
		} else {
			Button.LEDPattern(1);
			return false;
		}
	}

	public boolean takeControl() {
		return checkDistance();
	}

	public void suppress() {
		// Since this is highest priority behavior, suppress will never be
		// called.
	}

	public void action() {
		ClearTable.leftMotor.rotate(-180, true);// start Motor.A rotating
												// backward
		ClearTable.rightMotor.rotate(-180); // rotate C farther to make the
											// turn
		if ((System.currentTimeMillis() & 0x1) != 0) {
			ClearTable.leftMotor.rotate(-180, true);// start Motor.A
													// rotating backward
			ClearTable.rightMotor.rotate(180); // rotate C farther to make
												// the turn
		} else {
			ClearTable.rightMotor.rotate(-180, true);// start Motor.A
														// rotating backward
			ClearTable.leftMotor.rotate(180); // rotate C farther to make
												// the turn
		}
	}
}
