package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.chassis.Drivetrain;

public class DriveToTarget extends Command {
	private int pipeline;
	private int camMode;

	public DriveToTarget(int pipeline, int camMode) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);
		requires(Robot.m_limelight);
		// requires(Robot.arm);

		this.pipeline = pipeline;
		this.camMode = camMode;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.m_limelight.setPipeline(pipeline);
		Robot.m_limelight.setCamMode(camMode);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double c = Robot.m_limelight.getX() < 0 ? -RobotMap.vFFValue : RobotMap.vFFValue;
		double forward = Math.min(RobotMap.vMaxOutput, (Math.abs(Robot.m_limelight.getDistance()) - RobotMap.highGoalDistance) * RobotMap.vPValue);
		Robot.m_drivetrain.arcadeDrive(forward, -((Robot.m_limelight.getX() * RobotMap.vPAltValue) + c));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_limelight.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_limelight.stop();
	}
}