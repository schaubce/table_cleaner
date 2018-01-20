package cockroach.stupkajr.schaubce;

public class Node  {
	
	private float ambient;
	private float angle; 
	
	Node(float ambient, float angle){
		this.setAmbient(ambient); 
		this.setAngle(angle);
	}

	public float getAngle() {
		return angle;
	}

	public void setAngle(float angle) {
		this.angle = angle;
	}

	public float getAmbient() {
		return ambient;
	}

	public void setAmbient(float ambient) {
		this.ambient = ambient;
	}

	public String toString(){
		return "Ambient: " + ambient + "Angle: " + angle; 
	}
}
