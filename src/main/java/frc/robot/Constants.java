package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    //Drivetrain (whole robot) constants
    public static final double kMaxRobotSpeed = 3; // meters per second
    public static final double kMaxRobotAngularSpeed = Math.PI; // 1/2 rotation per second = Pi
    public static final double kWheelRadius = 0.0508;   //4" mk4 wheels
    public static final double kGearRatio = 8.1;  //actual 8.1
    public static final int kEncoderResolution = 42;



    //Swerve Module Configs
    public static final double kRobotWidth  = 0.381;      //Distance in x-coor from center of robot to swerve module
    public static final double kRobotLength = 0.381;      //Distance in y-coor from center of robot to swerve module
    public static final double kTurnMotorMaxAngSpeed = 2*Math.PI*2;     //2*pi*2 = 2 rotations per second

    //Front Left Swerve Module
    public static final double kFL_DriveChannel = 11;
    public static final double kFL_TurnChannel  = 10;
    public static final double kFL_TurnEncoderChannel = 0;
    public static final double kFL_TurnEncoderOffset = 0.634;

    public static final double kFR_DriveChannel = 15;
    public static final double kFR_TurnChannel  = 14;
    public static final double kFR_TurnEncoderChannel = 2;
    public static final double kFR_TurnEncoderOffset = 0.834;

    public static final double kBL_DriveChannel = 13;
    public static final double kBL_TurnChannel  = 12;
    public static final double kBL_TurnEncoderChannel = 1;
    public static final double kBL_TurnEncoderOffset = 0.412;

    public static final double kBR_DriveChannel = 17;
    public static final double kBR_TurnChannel  = 16;
    public static final double kBR_TurnEncoderChannel = 3;
    public static final double kBR_TurnEncoderOffset = 0.336;

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = .5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
