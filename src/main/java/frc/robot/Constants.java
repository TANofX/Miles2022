// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.HatSwitchButton;
import frc.robot.util.HatSwitchButton.HatDirection;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
                        ///////////////////////  D R I V I N G ///////////////////////
    //BUTTON INPUTS FOR FLIGHT STICK CONTROLLER

    // Flight Stick Constants
    public static final int FLIGHTSTICK_PORT = 1;

    // Shooting
    public static final int FLIGHTSTICK_SHOOT_ALL_BUTTON = 1;
    public static final int FLIGHTSTICK_SHOOT_ONE_BUTTON = 2;
    public static final int FLIGHTSTICK_SHOOTER_STOP_BUTTON = 7;
    public static final int FLIGHTSTICK_LOW_SHOT_BUTTON = 3;
    public static final int FLIGHTSTICK_HIGH_GOAL_BUTTON = 5;
    public static final int FLIGHTSTICK_HIGH_GOAL_HARD_BUTTON = 4;
    public static final int FLIGHTSTICK_FULL_POWER_BUTTON = 6;

    // Climbing
    public static final int FLIGHTSTICK_HIGH_TO_TRAVERSAL = 8;
    public static final int XBOX_MID_TO_HIGH = 6;
    public static final int FLIGHTSTICK_GABE_PNEUMATICS_BUTTON = 9;
    public static final int FLIGHTSTICK_RACHEL_REACH_PNEUMATICS_BUTTON = 10;
    public static final int FLIGHTSTICK_LOCK_RACHEL_MOVE_JOYSTICK_BUTTON = 11;
    public static final int FLIGHTSTICK_CALIBRATE_CLIMBER_BUTTON = 12;


    public static final int MOVE_RACHEL_AXIS = 2;

    public static final HatSwitchButton.HatDirection FLIGHTSTICK_HAT_PREPARE_TO_CLIMB_DIRECTION = HatDirection.UP;
    public static final HatSwitchButton.HatDirection FLIGHTSTICK_HAT_HIGH_BAR_CLIMB_DIRECTION = HatDirection.LEFT;
    public static final HatSwitchButton.HatDirection FLIGHTSTICK_HAT_TRAVERSAL_BAR_CLIMB_DIRECTION = HatDirection.RIGHT;
    public static final HatSwitchButton.HatDirection FLIGHTSTICK_HAT_SIMPLE_CLIMB_DIRECTION = HatDirection.DOWN;
  

 //   public static  int CLIMB_TRAVERSAL_BAR = 10;
 //   public static  int CLIMB_HIGH_BAR = 8;
  //  public static  int CANCEL_CLIMBER = 10;

                        ///////////////////////  C L I M B E R  ///////////////////////
    //  CLIMBER SUBSYSTEM CONSTANTS
    public static int RACHEL_UP_LEFT_LIMIT = 0;
    public static int RACHEL_UP_RIGHT_LIMIT = 1;
    public static int RACHEL_DOWN_LEFT_LIMIT = 2;
    public static int RACHEL_DOWN_RIGHT_LIMIT = 3;

    public static int GABE_CLAW = 1;
    public static int RACHEL_FORWARD_REACH = 3;
    public static int RACHEL_BACKWARD_REACH = 2;

    public static int GABE_LEFT_IDENTIFICATION = 3;
    public static int GABE_RIGHT_IDENTIFICATION = 2;
    public static int RACHEL_LEFT_BAR_SENSOR = 0;
    public static int RACHEL_RIGHT_BAR_SENSOR = 1;

    public static int RIGHT_RACHEL_FALCON = 12;
    public static int LEFT_RACHEL_FALCON = 13;

    public static double RACHEL_F = 0;
    public static double RACHEL_I = 0;
    public static double RACHEL_P = 0;

    public static double CLIMBER_THRESHOLD_CURRENT = 60;
    public static double CLIMBER_CURRENT_LIMIT = 39;
    public static double CLIMBER_THRESHOLD_TIMEOUT = 0.1;

    public static double CLIMB_HEIGHT_THRESHOLD = 400.0;
    public static double CLIMBER_MAX_VELOCITY = 13000.0;
    public static final double CLIMBER_MAX_ACCELERATION = 18000;





    // Shooter Constants
    public static final int SHOOTER_CURRENT_LIMIT = 39;
    public static final int SHOOTER_THRESHOLD_CURRENT = 60;
    public static final double SHOOTER_THRESHOLD_TIMEOUT = 0.01;
    public static final double SHOOTER_F = 0.06669998; //0.04;
    public static final double SHOOTER_P = 0.003;
    public static final double SHOOTER_I = 0.0001;//0.00125;
    public static final double SHOOTER_SPEED = 0.9;
    public static final double SHOOTER_SPIN_ERROR = 80;
    public static final int PRIMARY_SHOOTER_ID = 10;
    public static final int SECONDARY_SHOOTER_ID = 11;

    // Cargo (Ball) Handler Constants
    public static final int HANDLERSENSOR_1_PORT = 0;
    public static final int HANDLERSENSOR_2_PORT = 1;
    public static final int HANDLERSENSOR_3_PORT = 2;
    public static final int HANDLERSENSOR_4_PORT = 3;
    public static final int TRANSIT_MOTOR_ID = 8;
    public static final double TRANSIT_MOTOR_SPEED = -0.5;
    public static final double CLEAR_TIMEOUT = 2.0;

    // Intake Constants
    public static final int INTAKE_SOLENOID_PORT = 0;
    public static final int DETECTABLE_DISTANCE = 250;
    public static final double REVERSE_INTAKE_TIMEOUT = 1.0;
    public static final int INTAKE_MOTOR_ID = 9;
    public static final double INTAKE_SPEED = 0.45;

    // Xbox Constants
    public static final int XBOX_PORT = 0;
    public static final int XBOX_RUNINTAKE_BUTTON = 3;
    public static final double XBOX_DEADBAND = 0.15;


    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 4;
        public static final int kLeftMotor2Port = 5;
        public static final int kRightMotor1Port = 6;
        public static final int kRightMotor2Port = 7;
    
        public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
        public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final double kTrackwidthMeters = 0.5842;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kGearBoxRatio = 1.0 / 12.727;
        // Need gear ratio for test bed
        // 12.727
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            ((kWheelDiameterMeters * Math.PI) * kGearBoxRatio) / (double) kEncoderCPR;
    
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
        public static final double ksVolts = 0.58128;
        public static final double kvVoltSecondsPerMeter = 0.10789;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0069034;
    
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.29187;
        public static final double kMinimumTurnRate = 0.15;
        public static final double kThresholdCurrent = 40.0;
        public static final double kThresholdTimeout = 0.1;
        public static final double kCurrentLimit = 40.0;
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }
    
      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.8;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.9;
    
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }    
}
