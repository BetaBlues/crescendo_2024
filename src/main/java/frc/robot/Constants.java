// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.PIDGains;
import frc.robot.Subsystems.ClimbingSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public final class k_chassis {
    //Chassis Motor ports
    public final static int leftFrontMotorPort = 13;
    public final static int rightFrontMotorPort = 12;
    public final static int rightRearMotorPort = 45;
    public final static int leftRearMotorPort = 46;
    // 5 2 1 0 is technically correct but i changed it so that it would drive correctrly 
    // 2 5 0 1
    //chassis speeds
    public final static double normalDriveSpeed = 0.6;
    public final static double slowDriveSpeed = 0.5;
    public final static double normalRotationSpeed = 0.5;
    public final static double slowRotationSpeed = 0.2;
    
    public final static double inPerSecSpeed = 1; //edit

    public final static double gyro = 0;
  }

  //wrong
  public final class k_xbox {
    //buttons --> correct
        public static final int buttonA = 1; 
        public static final int buttonB = 2;
        public static final int buttonX = 3;
        public static final int buttonY = 4;
        public static final int buttonLeftBumper = 5;
        public static final int buttonRightBumper = 6;
        public static final int buttonLeftLowerBumper = 9;
        public static final int buttonRightLowerBumper = 10;
        public static final int buttonBack = 7;
        public static final int buttonStart = 8;
        //joysticks --> incorrect
        public static final int leftXAxis = 0;
        public static final int leftYAxis = 1;
        public static final int rightXAxis = 4; 
        public static final int rightYAxis = 3;
    }

    public static class ArmConstants 
    {
      // public static final int armID = 15;
      // public static final double p_loading = 25; //edit
      // public static final double p_shooting = 50; //edit
      // public static final double p_resting = 75; //edit
      // public static final double p_speedSlow = 0.1; //edit
      // public static final double p_speedFast = 0.25; //edit 

      public static final double incShooting = 4500; //edit
      public static final double incLoading = 19100; //edit
      public static final double velocityUp = -0.5; //edit
      public static final double velocityDown = 0.4; //edit


      public static final int kArmCanId = 8;
      public static final boolean kArmInverted = false;
      public static final int kCurrentLimit = 57;

      public static final double kSoftLimitReverse = -25000;
      public static final double kSoftLimitForward = 35000;

      public static final double kArmGearRatio = 100; //or 0.1
      public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; // multiply SM value by this number and get arm position in radians
      public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
      public static final double kArmFreeSpeed = 50 * kVelocityFactor;
      public static final double kArmZeroCosineOffset = 1.342; // radians to add to converted arm position to get real-world arm position (starts at
      // ~76.9deg angle)
      public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);
      public static final PIDGains kArmPositionGains = new PIDGains(0.4, 0.0, 0.0);
      public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(1.0, 2.0);

      public static final double kHomePosition = 0.0;
      public static final double kScoringPosition = 0.0;
      public static final double kIntakePosition = -1.17;
    }

    public static class IntakeConstants {

      public static final int intakeID = 9; //edit
      public static final double intakeP = 0.0;
      public static final double intakeD = 0.0;
      public static final double intakeI = 0.0;
  
      
  
      public static final double intakeSpeed = 0.2; //edit
      public static final double outputSpeed = -0.75; //edit
        // negative = counter-clockwise (?)
        // positive = clockwise (?)

      public static final int leftXAxis = 0; //edit
      public static final int leftYAxis = 1; //edit
  
      //public static final int start = 360; //previously start
      //public static final int open = 0; //previously open

      //public static final double intakeCloseSpeed = 0.1;
      //public static final double intakeOpenSpeed = -0.13;

    }

    public static class ClimbingSubsystemConstants {
      public static final int canID = 21;
    }
/* 
//Estimate Distance
public static class EstimateDistanceConstants
{
    final public static double limelightMountAngleDegrees = -45.0; // how many degrees back is your limelight rotated from perfectly vertical?
    final public static double limelightLensHeightInches = 27.5; // distance from the center of the Limelight lens to the floor
    final public static double goalHeightInchesAmp = 48.5; // bottom of AprilTag height --> Amp
    final public static double goalHeightInchesLoading = 48.125; //bottom of AprilTag height --> Loading Station
    final public static double goalHeightInchesStage = 47.5; //bottom of AprilTag height --> Stage
}

//Getting in Range
public static class GettingInRangeConstants
{
    final public static double KpDistance = -0.1; // Proportional control constant for distance
}

//Amp 
public static class AmpConstants
{
    //public static double AprilTagSpacing = 66.00; //in inches
    public static double desiredDistanceAmp = 132.25; //in inches --> edit 
    public static double limelightOffsetFromRobotCenter = 4.625; //in inches --> edit 
}

//Stage
public static class StageConstants
{
    public static double desiredDistanceStage = 16.625; //inches
    public static double limelightOffsetFromRobotCenter = 4.625; //in inches --> edit 
    
}

//Source
public static class LoadingConstants
{
    public static double desiredDistanceLoading = 43.797; //in inches --> edit
    public static double limelightOffsetFromRobotCenter = 4.625; //in inches
}
*/
}