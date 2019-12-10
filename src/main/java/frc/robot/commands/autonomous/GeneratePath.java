package frc.robot.commands.autonomous;

import frc.robot.OI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class GeneratePath extends Command {
	public GeneratePath() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_drivetrain.resetPath();
        Robot.m_drivetrain.addPoint(0, 0);
        Robot.m_drivetrain.addPoint(3, 0);
        Robot.m_drivetrain.generatePath();
        
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
