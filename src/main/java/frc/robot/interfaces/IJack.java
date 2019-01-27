package frc.robot.interfaces;


public interface IJack {

	public enum Position {
		LARGE_DRIVETRAIN, // outer drivetrain is down   //Previously UP
		MINI_DRIVETRAIN, // outer drivetrain is up     //Previously DOWN
		UNKNOWN;
	}	

	public void setPosition(Position pos);	
	
	public Position getPosition();
	
	public void waitSetPosition();
}
