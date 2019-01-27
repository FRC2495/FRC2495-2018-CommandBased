package frc.robot.interfaces;

public interface IGameData {

	public enum Plate {
		UNKNOWN,
		LEFT,
		RIGHT,
	}
	
	// this method needs to be called to retrieve the data once on the transition to Autonomous Enabled
	public void update();
	
	public Plate getAssignedPlateAtFirstSwitch();

	public Plate getAssignedPlateAtScale();

	public Plate getAssignedPlateAtSecondSwitch();

}
