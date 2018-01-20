package cockroach.stupkajr.schaubce;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/*
 * Things to consider:
 * Apparently it might have to go through the light to find the darkest place?
 * Will probably use steepest ascent because Annealing may take too long 
 * Two minute time limit
 * 
 * 
 */

public class Cockroach {
	// Motors
	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;

	// Sensors
	static EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S1);
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

	//initialize sample providers
	static SampleProvider touchSample;
	static SampleProvider gyroSample;
	static SampleProvider lightSample;
	
	//State Nodes
	private static Node bestState; 
	private static Node current; 
	
	//initialize speed/ambient
	static int speed = 540; // 1 RPM
	static float currentAmbient =  0;
	static ArrayList<Node> neighbors;
	
	/*
 	*	Main method will control access to all submethods of Cockroach program
 	*	1. Determine change in light intensity
 	*	2. initialize program
 	*	3. allow exit from press of ENTER button
 	*/
	public static void main(String[] args) {
		//Set Sample Providers
		gyroSample = gyroSensor.getAngleAndRateMode();
		lightSample = colorSensor.getAmbientMode();
		touchSample = touchSensor.getTouchMode();
			
		//Set Motor Speeds
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
			 		
		//get the distance of the wall
		float[] ambient = new float[2];
		// Give light sensor a second to work
		Delay.msDelay(1000);
		do{
			lightSample.fetchSample(ambient, 0);
			lightSample.fetchSample(ambient, 1);
		}while(ambient[0] != ambient[1]);
		//get current light intensity
		currentAmbient = ambient[0];

		// while there is no light
		boolean light = false;
		while (true) {
			// sense for light
			light = senseLight();

			// When the spider sees light, find the dark
			if (light) {
				findTheDark();
			}

			// Safety Exit, because DIM is a little DIM witted
			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();

				System.exit(0);
			}
		}
	}

	/*
 	*	senseLight method is used in retrieving light samples with the lightSensor, and alerting agent to start of program 
 	*	1. ensure data retrieved is accurate
 	*	2. if currentAmbient is changed by > 0.05 initialize program (This is at a high level due to testing in a room with changing light)
 	*
 	*/
	private static boolean senseLight() {
		
		// get the color sample to save
		float[] ambient = new float[2];
		// retrieve samples, until the samples retrieved at a steady state are the same
		do{
			lightSample.fetchSample(ambient, 0);
			lightSample.fetchSample(ambient, 1);
		}while(ambient[0] != ambient[1]);

		// If the light intensity has changed we should begin our action
		if( ambient[0] - currentAmbient > 0.015){
			System.out.println(ambient[0] + " is equal to " + currentAmbient  + "| RUN PROGRAM");
		}
		return ambient[0] - currentAmbient > 0.05;
	}

	/*
 	*	findTheDark method is the main control code in our agent,
 	*	allowing us to access all methods that are used by the program
 	*	1. continue until the 'darkestPlace' has been found 
 	*	2. retrieve area sample of current position
 	*	3. Average position to allow for ending in a dark, but not perfect position (This allows us to finish the algrithm in a timely manner)
 	*	4. move to next position after samples have been found, or end the program at our best position 
 	*/
	public static void findTheDark() {
		float[] ambient = new float[lightSample.sampleSize()];
		lightSample.fetchSample(ambient, 0);

		//initialize the Current state and Best state nodes
		current = new Node(ambient[0], 0);
		bestState = current; 

		boolean darkestPlace = false;
		// while the robot hasn't found the darkest place or the current state
		// hasn't changed
		while (!darkestPlace) {
			lightSample.fetchSample(ambient, 0);
			// Climb to the next best spot, if there are no better spots we've darkest local spot
			Node nextSpot = hillClimb(current);
			Sound.beep();
			System.out.println("Current Spot: "+ current.getAmbient() + "--Actual Ambient:" + ambient[0]);
			float ave = Integer.MAX_VALUE;
			float total = 0;
			// Error handling
			if(neighbors != null){
				for(int x=0; x< neighbors.size(); x++){
					total = total + neighbors.get(x).getAmbient();
				}	
				ave = total/neighbors.size();
				
			}
			System.out.println("Average: "+ave);
			if ((nextSpot.getAmbient() == current.getAmbient() && ambient[0] < 0.03) || (ave <= 0.0125)) {
				darkestPlace = true;
			} else {
				
				// move next spot
				moveTo(nextSpot.getAngle());

				// currentNode = nextNode;
				current = nextSpot;
			}
			
			// Safety Exit, because DIM is a little DIM witted
			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();

				System.exit(0);
			}
		}

		// signal it is in the darkest place
		Sound.beep();
		Sound.beep();
		Sound.beep();
		System.out.println("Current: "+ current);
		System.out.println("Darkest Spot: " + darkestPlace);
		leftMotor.stop();
		rightMotor.stop();
		Delay.msDelay(10000);
		System.exit(0);
	}

	/*
 	*	moveTo method allow the agent to orient itself correctly and move to a given position that was scanned
 	*	1. halt motion, and reset gyro for move
 	*	2. rotate to new angle
 	*	3. move to that direction 
 	*	4. if a wall was found stop and move back
 	*/
	private static void moveTo(float finalAngle) {
		leftMotor.stop();
		rightMotor.stop();
		Delay.msDelay(500);
		gyroSensor.reset();
		double angle = 0;
 
		float[] gyroAngle = new float[gyroSample.sampleSize()];
		gyroSample.fetchSample(gyroAngle, 0);
		angle = gyroAngle[0];
		
		//rotate until it hits the angle
		leftMotor.backward();
		rightMotor.forward();
		while (Math.abs(angle) < finalAngle) {
			Delay.msDelay(10);
			gyroSample.fetchSample(gyroAngle, 0);
			angle = gyroAngle[0];
			
			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();

				System.exit(0);
			}
		}
		
		//stop
		leftMotor.stop();
		rightMotor.stop();
		
		//drive to the spot
		leftMotor.forward();
		rightMotor.forward();
		// While we are far enough away from the wall continue to move
		boolean move = true;
		int shift = 0;
		while(atWall() && move){
			// once we've moved the full 2000 ms, end move
			if(shift >= 2500){
				move = false;
			}
			Delay.msDelay(250);
			// Add 500 to shift every time we loop
			shift += 250;
		}

		//stop
		leftMotor.stop();
		rightMotor.stop();
	}

	/*
 	*	hillClimb method is our actual hill Climbing algrithm, allowing us to choose the best neighbor position
 	*	1. Generate sample of neighbors states
 	*	2. sort and choose the best neighbor
 	*	3. replace our current position with the new position and return 
 	*
 	*/
	public static Node hillClimb(Node currentNode) {
		// get the children (neighbors in this case)
		ArrayList<Node> neighbors = getNeighbors();

		//pick the best neighbor
		Node nextNode = getBestNeighbor(neighbors); 

		//if the next best node is darker, save it
		if (nextNode.getAmbient() < bestState.getAmbient()){
			// TODO: Check if this is the best way to return and save best state
			currentNode = nextNode; 
			bestState = currentNode; 
		}

		return currentNode;
	}

	/*
 	*	getBestNeighbor method takes a list of sample positions and find the best position based off of sorting and ambient light measured
 	*	1. sort the neighbors
 	*	2. pick the best ambient neighbor
	*
 	*/
	private static Node getBestNeighbor(ArrayList<Node> neighbors) {
		for(int i = 0; i<neighbors.size(); i++){
			System.out.println("State "+ i + " - " + neighbors.get(i).getAmbient() + " Angle of: " + neighbors.get(i).getAngle());
		}
		// sort the points by ambient light 
		Node min = Collections.min(neighbors, new Comparator<Node>() {

			@Override
			public int compare(Node arg0, Node arg1) {
				if (arg0.getAmbient() < arg1.getAmbient()) {
					return -1;
				} else if (arg0.getAmbient() == arg1.getAmbient()) {
					return 0;
				} else {
					return 1;
				}
			}
		});
		System.out.println("Picked this state "+ min.getAngle() + " - " + min.getAmbient());
		// Return neighbor with the darkest light
		return min;
	}

	/*
 	*	getNeighbors method retrieves our position samples and allow us to save the angle at which the position was found
 	*	1. halt motion and prep gyro for move
 	*	2. move in 4 different directions at an ~90 degree angle due to mechanical inaccuracy 
 	*	3. scan and save positions
 	*	4. return positions
 	*/
	private static ArrayList<Node> getNeighbors() {
		ArrayList<Node> neighbors = new ArrayList<Node>();
		int delayTime = 1000;
		// get 4 neighbors
		leftMotor.stop();
		rightMotor.stop();
		gyroSensor.reset();
		
		for (int i = 0; i < 4; i++) {
			//reset the angle
			//gyroSample = gyroSensor.getAngleMode();
			

			
			// spin 85  degrees (180 on carpet = 170 * 2 / 4 = 85)  
			leftMotor.backward();
			rightMotor.forward();
			
			// get the sample
			float[] gyroAngle = new float[gyroSample.sampleSize()];
			gyroSample.fetchSample(gyroAngle, 0);
			float angle = gyroAngle[0];
			

			// Rotate until we've reached desired angle
			while (Math.abs(angle) < 85*i ) {
				Delay.msDelay(10);
				gyroSample.fetchSample(gyroAngle, 0);
				angle = gyroAngle[0];
				if (Button.ENTER.isDown()) {
					leftMotor.stop();
					rightMotor.stop();

					System.exit(0);
				}
			}
			
			//stop rotation
			leftMotor.stop();
			rightMotor.stop();
			
			//drive forward a foot
			leftMotor.forward();
			rightMotor.forward();
			
			// Move for 2 Seconds < 1 foot of motion
			boolean move = true;
			int shift = 0;
			while(atWall() && move){
				// once we've moved the full 2000 ms, end move
				if(shift >= delayTime){
					move = false;
				}
				Delay.msDelay(250);
				// Add 500 to shift every time we loop
				shift += 250;
			}

			leftMotor.stop();
			rightMotor.stop();
			
			// get the color sample to save
			float[] color = new float[lightSample.sampleSize()];
			lightSample.fetchSample(color, 0);

			// add the node to the neighbors
			neighbors.add(new Node(color[0], (float) (i + 1) * 85));
			
			//go back to the start
			leftMotor.backward();
			rightMotor.backward();
			
			
			Delay.msDelay(shift);

			leftMotor.stop();
			rightMotor.stop();
		}

		return neighbors;
	}

	/*
 	*	atWall method determines if our agent has reached a wall and is being pressed against a wall
 	*	1. if touching a wall, tell agent
  	*
 	*/
	private static boolean atWall() {
		float[] touched = new float[touchSample.sampleSize()];
		touchSample.fetchSample(touched, 0);
		if(touched[0] == 1){
			System.out.println("Touched a wall");
			return false;
		}else{
			return true;	
		}		
	}
	
}