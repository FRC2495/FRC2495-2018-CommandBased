package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.Vector;

public class PositionTracker {
	
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	
	double heading = 0.0;
	
	public class Position {
		String filter;
		String label;
		
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		
		double heading = 0.0;
		
		public Position(String filter_in, String label_in, double x_in, double y_in, double z_in, double heading_in) {
			filter = filter_in;
			label = label_in;
			
			x = x_in;
			y = y_in;
			z = z_in;
			
			heading = heading_in;
		}
	}
	
	Vector<Position> history = new Vector<>();
	
	public PositionTracker() {
		history.add(new Position("initial position","(initial position)",x,y,z,heading)); // initial position
	}
	
	// distance in inches, positive distance means move forward
	private void moveDistanceIncognito(double distance) {
		x = x + distance * Math.sin(Math.toRadians(heading)); 		
		y = y + distance * Math.cos(Math.toRadians(heading));
	}
	
	// distance in inches, positive distance means move forward
	public void moveDistance(double distance) {
		moveDistanceIncognito(distance);
		
		String label;
		DecimalFormat df = new DecimalFormat("#.#");
		
		if (distance > 0) {
			label = "(move forward " + df.format(Math.abs(distance)) + " inches)";
		} else if (distance < 0) {
			label = "(move back " + df.format(Math.abs(distance)) + " inches)";
		} else {
			label =  "(no move)";
		}
		
		history.add(new Position("move distance",label,x,y,z,heading));
	}
	
	// angle in degrees, positive angle means turns clockwise
	private void turnAngleIncognito(double angle) {
		heading = heading + angle; 
	}
	
	// angle in degrees, positive angle means turns clockwise
	public void turnAngle(double angle) {
		turnAngleIncognito(angle); 
		
		String label;
		DecimalFormat df = new DecimalFormat("#.#");
		
		if (angle > 0) {
			label = "(turn right " + df.format(Math.abs(angle)) + " degrees)";
		} else if (angle < 0) {
			label = "(turn left " + df.format(Math.abs(angle)) + " degrees)";
		} else {
			label = "(no move)";
		}
		
		history.add(new Position("turn angle",label,x,y,z,heading));
	}
	
	// distance in inches, positive distance means move right
	public void moveDistanceAthwart(double distance) {
		turnAngleIncognito(90); // turn right
		moveDistanceIncognito(distance);
		turnAngleIncognito(-90); // turn back left
		
		String label;
		DecimalFormat df = new DecimalFormat("#.#");
		
		if (distance > 0) {
			label = "(move arthwart right " + df.format(Math.abs(distance)) + " inches)";
		} else if (distance < 0) {
			label = "(move arthwart left " + df.format(Math.abs(distance)) + " inches)";
		} else {
			label = "(no move)";
		}
		
		history.add(new Position("move athwart",label,x,y,z,heading));
	}
	
	public void updateAltitude(double altitude) {
		
		double old_z = z;
		
		z = altitude;
		
		String label;		
		DecimalFormat df = new DecimalFormat("#.#");
		
		if (z > old_z) {
			label = "(move up to " + df.format(Math.abs(altitude)) + " inches)";
		} else if (z < old_z) {
			label = "(move down to " + df.format(Math.abs(altitude)) + " inches)";
		} else {
			label = "(no move)";
		}
		
		history.add(new Position("update altitude",label,x,y,z,heading));
	}
	
	public void printState() {
		DecimalFormat df = new DecimalFormat("#.#");
		
		System.out.println("PositionTracker: STATE x = " + df.format(x) + " inches, y = " + df.format(y) + " inches, " +
				"z = " + df.format(z) + " inches, heading = " + df.format(heading) + " degrees\n");
	}
	
	public Vector<Position> getHistory() {
		return history;
	}
	
	public void saveHistoryAsCsvFile(String path) {
		PrintWriter pw;
		
		try {
			pw = new PrintWriter(new File(path));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return; // not critical, we just return
		}
		
        StringBuilder sb = new StringBuilder();
        
        sb.append("index");
        sb.append(',');
        sb.append("filter");
        sb.append(',');
        sb.append("label");
        sb.append(',');
        sb.append("x");
        sb.append(',');
        sb.append("y");
        sb.append(',');
        sb.append("z");
        sb.append(',');
        sb.append("heading");
        sb.append('\n');

        for (int i = 0; i < history.size(); i++) {
        	
        	Position position = history.get(i);
        	
        	sb.append(i);
	        sb.append(',');
	        sb.append(position.filter);
	        sb.append(',');
        	sb.append(position.label);
	        sb.append(',');
	        sb.append(position.x);
	        sb.append(',');
	        sb.append(position.y);
	        sb.append(',');
	        sb.append(position.z);
	        sb.append(',');
	        sb.append(position.heading);
	        sb.append('\n');
        }
        
        pw.write(sb.toString());
        
        pw.close();
	}
	
}
