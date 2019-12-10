package frc.robot.commands.misc;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Timer;

public class BlinkForATime extends Command {
	Timer timer;
	double time;
	public BlinkForATime(double time) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
        requires(Robot.m_shooter);
        timer = new Timer();
        timer.reset();
        this.time = time;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        timer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
      Robot.m_limelight.setLED(2);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
        return timer.get() > time;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_limelight.setLED(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.m_limelight.setLED(0);
	}
}
