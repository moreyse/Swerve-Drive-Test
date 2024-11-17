// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {


  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(11, 10,  
                                                    0, .634);
  private final SwerveModule m_frontRight = new SwerveModule(15, 14, 
                                                    2, .834); 
  private final SwerveModule m_backLeft = new SwerveModule(13, 12,  
                                                    1, .412);
  private final SwerveModule m_backRight = new SwerveModule(17, 16, 
                                                    3, .336); 


  public final AnalogGyro m_gyro = new AnalogGyro(0);

  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Pose2d robotPose2d = new Pose2d();

  public Drivetrain() {
    m_gyro.reset();

    
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
      var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
                
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxRobotSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    double[] moduleDesiredStates = {
      swerveModuleStates[0].angle.getDegrees(),
      swerveModuleStates[0].speedMetersPerSecond,
      swerveModuleStates[1].angle.getDegrees(),
      swerveModuleStates[1].speedMetersPerSecond,
      swerveModuleStates[2].angle.getDegrees(),
      swerveModuleStates[2].speedMetersPerSecond,
      swerveModuleStates[3].angle.getDegrees(),
      swerveModuleStates[3].speedMetersPerSecond
    };

    double[] moduleOptDesStates = {
      m_frontLeft.getOptState().angle.getRadians(),
      m_frontLeft.getOptState().speedMetersPerSecond,
      m_frontRight.getOptState().angle.getRadians(),
      m_frontRight.getOptState().speedMetersPerSecond,
      m_backLeft.getOptState().angle.getRadians(),
      m_backLeft.getOptState().speedMetersPerSecond,
      m_backRight.getOptState().angle.getRadians(),
      m_backRight.getOptState().speedMetersPerSecond
    };

    double[] moduleActualStates = {
      m_frontLeft.getState().angle.getDegrees(),
      m_frontLeft.getState().speedMetersPerSecond,
      m_frontRight.getState().angle.getDegrees(),
      m_frontRight.getState().speedMetersPerSecond,
      m_backLeft.getState().angle.getDegrees(),
      m_backLeft.getState().speedMetersPerSecond,
      m_backRight.getState().angle.getDegrees(),
      m_backRight.getState().speedMetersPerSecond
    };

    SmartDashboard.putNumberArray("DesiredStates", moduleDesiredStates);
    SmartDashboard.putNumberArray("ActualStates", moduleActualStates);
    SmartDashboard.putNumberArray("OptDesiredStates", moduleOptDesStates);
    SmartDashboard.putNumber("FL_adj_Position", m_frontLeft.getAdjustedAngle());
    SmartDashboard.putNumber("FR_adj_Position", m_frontRight.getAdjustedAngle());
    SmartDashboard.putNumber("BL_adj_Position", m_backLeft.getAdjustedAngle());
    SmartDashboard.putNumber("BR_adj_Position", m_backRight.getAdjustedAngle());
    SmartDashboard.putNumber("FL_abs_Position", m_frontLeft.getAbsAngle());
    SmartDashboard.putNumber("FR_abs_Position", m_frontRight.getAbsAngle());
    SmartDashboard.putNumber("BL_abs_Position", m_backLeft.getAbsAngle());
    SmartDashboard.putNumber("BR_abs_Position", m_backRight.getAbsAngle());
    SmartDashboard.putNumber("FL_Desired_Angle", swerveModuleStates[0].angle.getRadians());
    SmartDashboard.putNumber("FR_Desired_Angle", swerveModuleStates[1].angle.getRadians());
    SmartDashboard.putNumber("BL_Desired_Angle", swerveModuleStates[2].angle.getRadians());
    SmartDashboard.putNumber("BR_Desired_Angle", swerveModuleStates[3].angle.getRadians());

    SmartDashboard.putNumberArray("FL Actual V", m_frontLeft.getActualMotorVoltageData());
    SmartDashboard.putNumberArray("FR Actual V", m_frontRight.getActualMotorVoltageData());
    SmartDashboard.putNumberArray("BL Actual V", m_backLeft.getActualMotorVoltageData());
    SmartDashboard.putNumberArray("BR Actual V", m_backLeft.getActualMotorVoltageData());

    SmartDashboard.putNumberArray("FL Command", m_frontLeft.getMotorVoltageData());
    SmartDashboard.putNumberArray("FR Command", m_frontRight.getMotorVoltageData());
    SmartDashboard.putNumberArray("BL Command", m_backLeft.getMotorVoltageData());
    SmartDashboard.putNumberArray("BR Command", m_backRight.getMotorVoltageData());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Pose2d getPose() {
    return robotPose2d;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxRobotSpeed);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
  }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

}
