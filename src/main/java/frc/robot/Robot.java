/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick m_stick;

  Supplier<Double> m_leftEncoderPosition;
  Supplier<Double> m_rightEncoderPosition;
  Supplier<Double> m_leftEncoderRate;
  Supplier<Double> m_rightEncoderRate;
  Supplier<Double> m_gyroAngleRadians;

  NetworkTableEntry m_autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry m_telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry m_rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  
  double m_priorAutoSpeed = 0;
  Number[] m_numberArray = new Number[10];
  
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final RomiGyro m_gyro = new RomiGyro();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // Note that the angle from the gyro must be negated because 
    // getAngle returns a clockwise positive
    m_gyroAngleRadians = () -> (-m_gyro.getAngleZ() * Math.PI / 180.0);

    m_leftEncoderPosition = m_drivetrain::getLeftDistanceInch;
    m_leftEncoderRate = m_drivetrain::getLeftEncoderRate;

    m_rightEncoderPosition = m_drivetrain::getRightDistanceInch;
    m_rightEncoderRate = m_drivetrain::getRightEncoderRate;

    NetworkTableInstance.getDefault().setUpdateRate(0.010);
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
    SmartDashboard.putNumber("l_encoder_pos", m_leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", m_leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", m_rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", m_rightEncoderRate.get());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
    m_drivetrain.resetEncoders();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double now = Timer.getFPGATimestamp();

    double leftPosition = m_leftEncoderPosition.get();
    double leftRate = m_leftEncoderRate.get();

    double rightPosition = m_rightEncoderPosition.get();
    double rightRate = m_rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();

    double motorVolts = battery * Math.abs(m_priorAutoSpeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = m_autoSpeedEntry.getDouble(0);
    m_priorAutoSpeed = autospeed;

    // Command motors to do things
    m_drivetrain.tankDrive(
      (m_rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );

    m_numberArray[0] = now;
    m_numberArray[1] = battery;
    m_numberArray[2] = autospeed;
    m_numberArray[3] = leftMotorVolts;
    m_numberArray[4] = rightMotorVolts;
    m_numberArray[5] = leftPosition;
    m_numberArray[6] = rightPosition;
    m_numberArray[7] = leftRate;
    m_numberArray[8] = rightRate;
    m_numberArray[9] = m_gyroAngleRadians.get();

    m_telemetryEntry.setNumberArray(m_numberArray);
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_drivetrain.arcadeDrive(m_stick.getY(Hand.kLeft), m_stick.getX(Hand.kRight));
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
    System.out.println("Robot disabled");
    m_drivetrain.tankDrive(0, 0);
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
