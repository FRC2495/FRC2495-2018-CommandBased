package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.interfaces.*;


public class GameData implements IGameData{
	
	/*public enum Plate {
		UNKNOWN,
		LEFT,
		RIGHT,
	}*/

	private String gameData;
	
	// this method needs to be called to retrieve the data once on the transition to Autonomous Enabled
	public void update() {
		DriverStation driverStation = DriverStation.getInstance();
		
		if (driverStation != null) {
			gameData = driverStation.getGameSpecificMessage();
		} else {
			gameData = null;
		}
	}
	
	public Plate getAssignedPlateAtFirstSwitch() {
		
		if (gameData != null && gameData.length() >= 1) {
			if (gameData.charAt(0) == 'L') {
				return Plate.LEFT;
			} else if (gameData.charAt(0) == 'R') {
				return Plate.RIGHT;
			}
		}
		
		return Plate.UNKNOWN; // if not left or right
	}

	public Plate getAssignedPlateAtScale() {
		
		if (gameData != null && gameData.length() >= 2) {
			if (gameData.charAt(1) == 'L') {
				return Plate.LEFT;
			} else if (gameData.charAt(1) == 'R') {
				return Plate.RIGHT;
			}
		}
		
		return Plate.UNKNOWN; // if not left or right
	}

	public Plate getAssignedPlateAtSecondSwitch() {
		
		if (gameData != null && gameData.length() >= 3) {
			if (gameData.charAt(2) == 'L') {
				return Plate.LEFT;
			} else if (gameData.charAt(2) == 'R') {
				return Plate.RIGHT;
			}
		}
		
		return Plate.UNKNOWN; // if not left or right
	}

}
