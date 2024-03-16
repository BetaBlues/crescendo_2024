// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel;

//import com.ctre.phoenix.motorcontrol.can.CANSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class ChassisSubsystem extends SubsystemBase{
  //Creates new Chassis
  /*public CANSparkMax leftFrontMotor;
  public CANSparkMax leftRearMotor;
  public CANSparkMax rightFrontMotor;
  public CANSparkMax rightRearMotor;*/

  public WPI_VictorSPX leftFrontMotor; 
  public WPI_VictorSPX leftRearMotor;
  public WPI_VictorSPX rightFrontMotor;
  public WPI_VictorSPX rightRearMotor; 
  private MecanumDrive driveTrain;
  private AHRS gyro; 

  public ChassisSubsystem() {
    /*leftFrontMotor = new CANSparkMax(Constants.k_chassis.leftFrontMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    leftRearMotor = new CANSparkMax(Constants.k_chassis.leftRearMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    rightFrontMotor = new CANSparkMax(Constants.k_chassis.rightFrontMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    rightRearMotor = new CANSparkMax(Constants.k_chassis.rightRearMotorPort, CANSparkLowLevel.MotorType.kBrushed);
    */
    
    
    leftFrontMotor = new WPI_VictorSPX(1);
    leftRearMotor = new WPI_VictorSPX(5);
    rightFrontMotor = new WPI_VictorSPX(2);
    rightRearMotor = new WPI_VictorSPX(0);
    
    
    rightFrontMotor.setInverted(true);
    rightRearMotor.setInverted(true);
    driveTrain = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

    gyro = new AHRS(SPI.Port.kMXP);
    

    AutoBuilder.configureHolonomic( //TODO: get pose methods
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public Pose2d getPose(){
    
  }


  

  public void driveFieldOriented(XboxController controller){
    double forward = -controller.getRawAxis(1);
    double strafe = controller.getRawAxis(0);
    double rotation = controller.getRawAxis(4);

    double gyroAngle = gyro.getAngle(); 
    double radianAngle = Math.toRadians(gyroAngle);
    double temp = forward * Math.cos(radianAngle) + strafe * Math.sin(radianAngle);
    strafe = -forward * Math.sin(radianAngle) + strafe * Math.cos(radianAngle);
    forward = temp; 

    driveTrain.driveCartesian(strafe, forward, rotation);
  }

  private double getGyroAngle(){
   
    return gyro.getAngle();
  }

  private void resetGyroAngle(){
    gyro.reset();
  }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, Rotation2d gyroAngle) {
    driveTrain.driveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
  }


  public void driveCartesian(double ySpeed, double xSpeed, double zRotation)
  {
    driveTrain.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  /*public static void setMaxOutput(double maxSpeed) {
    Chassis.setMaxOutput(0.5);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command followPathCommand(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return AutoBuilder.followPath(path);

  }


  



/*
 * -------------------------------------------------------------
 * -------------------------------------------------------------
 *                    LIMELIGHT STUFF !!!
 * -------------------------------------------------------------
 * -------------------------------------------------------------
 */
  
 /* 
  //Calculating Classes
  public boolean isTargetFound() 
  {
      //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

      return tv != 0;
      //ouble tv = table.getEntry("tv").getDouble(0);
  }
  
  public long AprilTagFoundID() // returns AprilTag ID after determining target has been found
  {
    long AprilTagID = 0;
    
    if(isTargetFound())
      {
        AprilTagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0); 
      }
    
    return AprilTagID;
  }
  
  public double Estimate_Distance() 
  {
      NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

      double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0); //vertical offset

      double angleToGoalDegrees = Constants.EstimateDistanceConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
      
      //calculate distance
      double distanceFromLimelightToGoalInches = (Constants.EstimateDistanceConstants.goalHeightInchesAmp - Constants.EstimateDistanceConstants.limelightLensHeightInches)/Math.tan(angleToGoalRadians); //return
      return distanceFromLimelightToGoalInches;
  }
  
  public static double DistanceToTimeCalculation(double distance)
  {
    double time = distance/Constants.k_chassis.inPerSecSpeed; //edit constant
    return time;
  }

  public void findCorrectSpotX(double tx){
    if(tx > 0)
      {
        //new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed, this.withTimeout(DistanceToTimeCalculation(-tx + Constants.AmpConstants.limelightOffsetFromRobotCenter))));
      }
      else if(tx < 0)
      {
        //new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed, this.withTimeout(DistanceToTimeCalculation(-tx + Constants.AmpConstants.limelightOffsetFromRobotCenter))));
      }
  }
  public void findCorrectSpotY(double time, double distanceError){
    if(distanceError > 0) //goes forwards
      {
        //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time))); //(zRotation, ySpeed, xSpeed)
      }
      else if(distanceError < 0)//goes backwards
      {
        //new RunCommand(() -> driveCartesian(0, Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time))); //(zRotation, ySpeed, xSpeed)
      }
  }
 
  //Driving/Positional Classes

  //Amp on Blue Alliance --> ID 6
  //add code for angle of moving arm --> loading mech
  public void TargetAmpBlue()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    long AprilTagID = AprilTagFoundID();

    double distanceError = 0;
    double drivingAdjust = 0;
    double desiredDistance = Constants.AmpConstants.desiredDistanceAmp;

    if(AprilTagID == 6)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
          //move -tx
      findCorrectSpotX(tx);
      
      //left AprilTagSpacing (negative value)
      //new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed, this.withTimeout(DistanceToTimeCalculation(Constants.AmpConstants.AprilTagSpacing))));
      
      double currentDistance = Estimate_Distance();
      
      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
      
      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time)));//(zRotation, ySpeed, xSpeed)
      
    }
  }

  //Amp on Red Alliance --> ID 5
  //add code for angle of moving arm --> loading mech
  public void TargetAmpRed()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    long AprilTagID = AprilTagFoundID();

    double distanceError = 0;
    double drivingAdjust = 0;
    double desiredDistance = Constants.AmpConstants.desiredDistanceAmp;

    if(AprilTagID == 5)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
          //move -tx
      findCorrectSpotX(tx);
      
      //right AprilTagSpacing (positive value)
      //new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed, this.withTimeout(DistanceToTimeCalculation(Constants.AmpConstants.AprilTagSpacing))));
      
      double currentDistance = Estimate_Distance();
      
      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
      
      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time)));//(zRotation, ySpeed, xSpeed)
      
    }
  }

    public void TargetStageChain()
  {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    long AprilTagID = AprilTagFoundID();

    double distanceError = 0;
    double drivingAdjust = 0;
    double desiredDistance = Constants.StageConstants.desiredDistanceStage;

    if(AprilTagID == 11 || AprilTagID == 12 || AprilTagID == 13 || AprilTagID == 14 || AprilTagID == 15 || AprilTagID == 16)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
          //move -tx
      findCorrectSpotX(tx);
      
      //right AprilTagSpacing (positive value)
      //new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed, this.withTimeout(DistanceToTimeCalculation(Constants.StageConstants.AprilTagSpacing))));
      
      double currentDistance = Estimate_Distance();
      
      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
      
      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time)));//(zRotation, ySpeed, xSpeed)
      
    }
  }

  public void loadingBlue ()
{
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  long AprilTagID = AprilTagFoundID();
  double distanceError = 0;
  double drivingAdjust = 0;
  double desiredDistance = Constants.LoadingConstants.desiredDistanceLoading;

  if(AprilTagID == 1 || AprilTagID == 2)
  {
      double tx = table.getEntry("tx").getDouble(0);
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
      
        //move -tx
      findCorrectSpotX(tx);

      //left AprilTagSpacing (negative value)
      //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time)));//(zRotation, yspeed, xSpeed)

      double currentDistance = Estimate_Distance();

      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;

      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time)));//(zRotation, ySpeed, xSpeed)
  }
}

public void loadingRed ()
{
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  long AprilTagID = AprilTagFoundID();
  double distanceError = 0;
  double drivingAdjust = 0;
  double desiredDistance = Constants.LoadingConstants.desiredDistanceLoading;

  if(AprilTagID == 9 || AprilTagID == 10)
  {
      double tx = table.getEntry("tx").getDouble(0);
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot

        //move -tx
      findCorrectSpotX(tx);

      //left AprilTagSpacing (negative value)
      //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time));//(zRotation, yspeed, xSpeed)

      double currentDistance = Estimate_Distance();

      distanceError = desiredDistance - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;

      //move backwards
      double time = DistanceToTimeCalculation(distanceError);
      //new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0, this.withTimeout(time)));//(zRotation, ySpeed, xSpeed)
  }
  
}
  //input ApirlTag ID into desired distance variable
/*
  public void getOnChargeStation() //automatically drive to desired distance from target
  {
      AprilTagID = AprilTagFoundID();

      double distanceError = 0;
      double drivingAdjust = 0;
      double desiredDistance = Constants.ChargeStationConstants.desiredDistanceCS;
  
      if(AprilTagID == 6 || AprilTagID == 1)
      {
          double tx = table.getEntry("tx").getDouble(0);
          
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          findCorrectSpotX(tx);
          
          //left AprilTagSpacing (negative value)
          new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ChargeStationConstants.AprilTagSpacing));
          
          double currentDistance = Estimate_Distance();
          
          distanceError = desiredDistance - currentDistance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError);
          new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
          
      }
      else if(AprilTagID == 7 || AprilTagID == 2)
      {
          double tx = table.getEntry("tx").getDouble(0);
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          findCorrectSpotX(tx);
          
          double current_distance = Estimate_Distance();
          
          distanceError = desiredDistance - current_distance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError);
          new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
          
      }
      else if(AprilTagID == 8 || AprilTagID == 3)
      {
          double tx = table.getEntry("tx").getDouble(0);
          
          //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
              //move -tx
          findCorrectSpotX(tx);
          
          //right AprilTagSpacing (positive value)
          new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ChargeStationConstants.AprilTagSpacing));
          
          double currentDistance = Estimate_Distance();
          
          distanceError = desiredDistance - currentDistance;
          drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
          //move backwards
          double time = DistanceToTimeCalculation(distanceError); 
          new RunCommand(() -> driveCartesian(0, -Constants.k_chassis.normalDriveSpeed, 0), this).withTimeout(time);//(zRotation, ySpeed, xSpeed)
      }
  }
  
  

  public void ShoulderLineUpLeft()
  {
    double distanceError = 0;
    double drivingAdjust = 0;

    if(AprilTagID == 1 || AprilTagID == 2 || AprilTagID == 3 || AprilTagID == 6 || AprilTagID == 7 || AprilTagID == 8)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
        //move -tx
      findCorrectSpotX(tx);

      //go left
      new RunCommand(() -> driveCartesian(0, 0, -Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ShoulderDriveConstants.nodeSpacingFromAprilTag));
      
      double currentDistance = Estimate_Distance();

      distanceError = Constants.ShoulderDriveConstants.desiredDistanceSD - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
      //move backwards
      double time = DistanceToTimeCalculation(distanceError); 
      findCorrectSpotY(time, distanceError);
      
    }
  }

  public void ShoulderLineUpCenter()
  {
    double distanceError = 0;
    double drivingAdjust = 0;

    if(AprilTagID == 1 || AprilTagID == 2 || AprilTagID == 3 || AprilTagID == 6 || AprilTagID == 7 || AprilTagID == 8)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
        //move -tx
      findCorrectSpotX(tx);

      double currentDistance = Estimate_Distance();

      distanceError = Constants.ShoulderDriveConstants.desiredDistanceSD - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
      //move backwards
      double time = DistanceToTimeCalculation(distanceError); 
      findCorrectSpotY(time, distanceError);
      
    }
  }

  public void ShoulderLineUpRight()
  {
    double distanceError = 0;
    double drivingAdjust = 0;

    if(AprilTagID == 1 || AprilTagID == 2 || AprilTagID == 3 || AprilTagID == 6 || AprilTagID == 7 || AprilTagID == 8)
    {
      double tx = table.getEntry("tx").getDouble(0);
          
      //center robot --> parallel w/ AprilTag and AprilTag is centered w/ robot
        //move -tx
      findCorrectSpotX(tx);

      //go right
      new RunCommand(() -> driveCartesian(0, 0, Constants.k_chassis.normalDriveSpeed), this).withTimeout(DistanceToTimeCalculation(Constants.ShoulderDriveConstants.nodeSpacingFromAprilTag));
      
      double currentDistance = Estimate_Distance();

      distanceError = Constants.ShoulderDriveConstants.desiredDistanceSD - currentDistance;
      drivingAdjust = Constants.GettingInRangeConstants.KpDistance * distanceError;
          
      //move backwards
      double time = DistanceToTimeCalculation(distanceError); 
      findCorrectSpotY(time, distanceError);
      
    }
  }
*/

}

