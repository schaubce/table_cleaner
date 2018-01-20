package cleartable.stupkajr.schaubce;

import java.awt.Point;

public class Vector {
	Point position = new Point();
	double angle;
	
	public Vector(){
		this.position = new Point(0,0); 
		this.angle = 0; 
		
	}
	
	public Vector(Point point, double angle){
		this.position = point;
		this.angle = angle;
	}
	
	public Point getPosition(){
		return position;
	}

	public double getAngle(){
		return angle;
	}
	
	public String toString(){
		return position.toString()+"|"+angle+" ";
	}

}
