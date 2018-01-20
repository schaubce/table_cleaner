package cockroach.stupkajr.schaubce;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;


/*
 * Things to consider:
 * Apparently it might have to go through the light to find the darkest place?
 * Will probably use steepest ascent because Annealing may take too long 
 * Two minute time limit
 * 
 * 
 */

public class Spider {
	//Motors
	static RegulatedMotor leftMotor = Motor.D;
	static RegulatedMotor rightMotor = Motor.A;
	
	//Sensors
	static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S1);
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

	static SampleProvider colorSample;
	static SampleProvider angleSample;
	
	private static float originalColor; 
	
	public static void main(String[] args) {
		boolean light = false; 
		float[] color = new float[colorSample.sampleSize()];
		colorSample.fetchSample(color, 0);
		
		originalColor = color[0]; 
		
		
		//while there is no light 
		while(true){
			//sense for light
			light = senseLight(); 
			
			//When the spider sees light, find the dark
			if (light){
				findTheDark();
			}
			
			if (Button.ENTER.isDown()) {
				leftMotor.stop();
				rightMotor.stop();

				System.exit(0);
			}
		}
	}

	
	private static boolean senseLight() {
		//get the color sample to save 
		float[] color = new float[colorSample.sampleSize()];
		colorSample.fetchSample(color, 0);
		
		return color[0] != originalColor;
	}

	public static void findTheDark() {
		//while the robot hasn't found the darkest place or the current state hasnt changed
			//get currentState
			//Node nextSpot = hillClimb(currentState); 
			//move next spot
		// currentNode = nextNode;
		
		
	}
	
	

//2. Loop until a solution is found or until a complete iteration produces no change to current state:
//a. Let SUCC be a state such that any possible successor of the current state will better than SUCC.
//b. For each child of the current state do:
//Evaluate the child:	If it is the goal then return it and quit. 
//			If it is not the goal, compare it to SUCC.
//			   	If it is better than SUCC, set SUCC to 					this child.
//			  	If it is not better, leave SUCC alone
//c. If SUCC is better than the current state, then set current state to SUCC.
//
//	
	public Node hillClimb(Node currentNode){
		//get the children (neighbors in this case)
		ArrayList<Node> neighbors = getNeighbors();
//		      nextEval = -INF;
//		      nextNode = NULL;
		for(int i = 0; i < neighbors.size(); i++){
//	         //
			//if (EVAL(x) > nextEval)
//            nextNode = x;
//            nextEval = EVAL(x);
		}

//		      if nextEval <= EVAL(currentNode)
//		         //Return current node since no better neighbors exist
//		         return currentNode;
//		     
		      
		return currentNode; 
	}

	private Node getBestNeighbor(ArrayList<Node> neighbors){
		// sort the points by x
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
		
		return min;
	}

	private ArrayList<Node> getNeighbors() {
		ArrayList<Node> neighbors = new ArrayList<Node> ();  
		
		// get 36 neighbors
		for (int i = 0; i < 36; i++){
			angleSample = gyroSensor.getAngleMode();
			gyroSensor.reset();
			
			//get the sample
			float[] gyroAngle = new float[angleSample.sampleSize()];
			angleSample.fetchSample(gyroAngle, 0);
			float angle = gyroAngle[0];
			
			//spin 10 degrees
			int rotated = 10; 
			while(Math.abs(angle) != 10){
				rightMotor.rotate(rotated);
				angleSample.fetchSample(gyroAngle, 0); 
				angle = gyroAngle[0];
				rotated = 1; 
			}
			
			//get the color sample to save 
			float[] color = new float[colorSample.sampleSize()];
			colorSample.fetchSample(color, 0);
			
			//add the node to the neighbors
			neighbors.add(new Node(color[0], (float)(i+1) * 10));
		}
		
		return neighbors;
	}
}
