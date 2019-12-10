package frc.robot.commands.shooter;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DeliverGoalHigh extends Command {
	public DeliverGoalHigh() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_shooter);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	  Robot.m_shooter.setShooterSpeed(RobotMap.highGoalSpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
        Robot.m_shooter.shooterMotor.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.m_shooter.shooterMotor.set(0);
	}
}
