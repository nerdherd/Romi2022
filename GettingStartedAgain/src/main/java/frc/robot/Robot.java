// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

      // Constants such as camera and target height stored. Change per robot and goal!
      final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
      final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
      // Angle between horizontal and the camera.
      final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  
      // How far from the target we want to be
      final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  
      // Change this to match the name of your camera
      PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  
      // PID constants should be tuned per robot;
      ""
      final double LINEAR_P = 0.1;
      final double LINEAR_D = 0.0;
      PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  
      final double ANGULAR_P = 0.1;
      final double ANGULAR_D = 0.0;
      PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);`
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // Get selected routine from the SmartDashboard
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running which will
    // use the default command which is ArcadeDrive. If you want the autonomous
    // to continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forwardSpeed;
    double rotationSpeed;

    //forwardSpeed = 10;//-m_robotContainer.m_controller.getRawAxis();
    forwardSpeed = -m_robotContainer.m_controller.getRawAxis(2);

    SmartDashboard.putBoolean("Button A:", m_robotContainer.m_buttonA.get());

    if (m_robotContainer.m_buttonA.get() == true) {
      
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      SmartDashboard.putBoolean("Has targets:", result.hasTargets());

      if (result.hasTargets()) {
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        // If we have no targets, stay still.
        rotationSpeed = 0;
      }
    } else {
      // Manual Driver Mode
      //rotationSpeed = 10;//xboxController.getLeftX();
      rotationSpeed = m_robotContainer.m_controller.getRawAxis(4);
    }

    // Use our forward/turn speeds to control the drivetrain
    m_robotContainer.m_drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
