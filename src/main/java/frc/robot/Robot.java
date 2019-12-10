/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.chassis.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.shooter.Shooter;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.subsystems.feeder.Feeder;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static AHRS m_navX;
  public static Drivetrain m_drivetrain;
  public static Shooter m_shooter;
  public static Feeder m_feeder;
  public static Vision m_limelight;
  public static OI m_oi;

  
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    try {
			m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);
		} catch (RuntimeException e) {
			DriverStation.reportError("NAVX ERROR: " + e.getMessage(), true);
    }

    m_drivetrain = new Drivetrain();
    m_shooter = new Shooter();
    m_feeder = new Feeder();
    m_limelight = new Vision();
    m_oi = new OI();
    
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putNumber("Heading ", 0);
    m_drivetrain.resetDrivetrain();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    
    SmartDashboard.putNumber("Output (Left Wheels)", 0);
    SmartDashboard.putNumber("Output (Right Wheels)", 0);
    SmartDashboard.putNumber("Heading ", 0);
    SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    Robot.m_drivetrain.leftMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.m_drivetrain.leftMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
		Robot.m_drivetrain.rightMotorA.setIdleMode(CANSparkMax.IdleMode.kCoast);
    Robot.m_drivetrain.rightMotorB.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
    SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.leftMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.leftMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    Robot.m_drivetrain.rightMotorA.getEncoder().setPosition(0);
    Robot.m_drivetrain.rightMotorB.getEncoder().setPosition(0);
    SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    
    SmartDashboard.putNumber("Output (Left Wheels)", 0);
    SmartDashboard.putNumber("Output (Right Wheels)", 0);
    SmartDashboard.putNumber("Heading ", 0);
    SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Left Encoder Values", Robot.m_drivetrain.leftMotorA.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Encoder Values", Robot.m_drivetrain.rightMotorA.getEncoder().getPosition());
    SmartDashboard.putNumber("Distance Covered (Right Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.rightMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Distance Covered (Left Wheels) (In Feet)", Robot.m_drivetrain.distanceInFeet(Robot.m_drivetrain.leftMotorA.getEncoder().getPosition()));
    SmartDashboard.putNumber("Angle ", Robot.m_navX.getAngle());
    m_limelight.printTelemetry();
    Scheduler.getInstance().run();
  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void printAutonomousTelemetry() {
    
  }
}
