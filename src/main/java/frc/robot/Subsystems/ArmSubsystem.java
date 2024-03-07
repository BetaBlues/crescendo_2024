// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkPIDController armController;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
  private double m_manualValue;

  private double m_positionResting;
  private double m_positionShooting;
  private double m_positionLoading;

  private double direction;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // create a new SPARK MAX and configure it
    armMotor = new CANSparkMax(Constants.ArmConstants.kArmCanId, MotorType.kBrushless);
    armMotor.setInverted(false);
    armMotor.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ArmConstants.kSoftLimitForward);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.ArmConstants.kSoftLimitReverse);

    // set up the motor encoder including conversion factors to convert to radians and radians per
    // second for position and velocity
    armEncoder = armMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42); //edit
    armEncoder.setPositionConversionFactor(Constants.ArmConstants.kPositionFactor);
    armEncoder.setVelocityConversionFactor(Constants.ArmConstants.kVelocityFactor);
    armEncoder.setPosition(0.0);

    armController = armMotor.getPIDController();
    PIDGains.setSparkMaxGains(armController, Constants.ArmConstants.kArmPositionGains);

    armMotor.burnFlash();

    m_setpoint = armEncoder.getPosition();

    direction = 0.0;

    m_timer = new Timer();
    m_timer.start();

    // updateMotionProfile();
  }

  public void inc_setpoint()
  {
    setTargetPosition(armEncoder.getPosition() + 1500);

    System.out.println("setpoint = " + m_setpoint);
    System.out.println("current position = " + armEncoder.getPosition());
  }

  public void dec_setpoint()
  {
    setTargetPosition(armEncoder.getPosition() - 1500);
    
    System.out.println("setpoint = " + m_setpoint);
    System.out.println("current position = " + armEncoder.getPosition());
  }

  /**
   * Sets the target position and updates the motion profile if the target position changed.
   *
   * @param _setpoint The new target position in radians.
   */
  public void setTargetPosition(double pGoal) {
    if (armEncoder.getPosition() != pGoal) {
      m_setpoint = pGoal;
      // updateMotionProfile();

      if(pGoal > armEncoder.getPosition())
      {
        direction = 1.0;
      }
      else if (pGoal < armEncoder.getPosition())
      {
        direction = -1.0;
      }
      else
      {
        direction = 0;
      }
      System.out.println("setting setpoint = " + m_setpoint);
    }
  }


  /**
   * Update the motion profile variables based on the current setpoint and the pre-configured motion
   * constraints.
   */
  // private void updateMotionProfile() {
  //   m_startState = new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
  //   m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
  //   m_profile = new TrapezoidProfile(Constants.ArmConstants.kArmMotionConstraint);
  //   m_timer.reset();
  //     }

  /**
   * Drives the arm to a position using a trapezoidal motion profile. This function is usually
   * wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
   *
   * <p>This function updates the motor position control loop using a setpoint from the trapezoidal
   * motion profile. The target position is the last set position with {@code setTargetPosition}.
   */
  public void runAutomatic() {
    // double elapsedTime = m_timer.get();
    double distToTarget = 0;

    distToTarget = m_setpoint - armEncoder.getPosition();
    if(Math.abs(distToTarget) < 25)
    {
      armMotor.stopMotor();

      return;
    }

    if(distToTarget < 0 && direction < 0)
    {
      System.out.println("set velocity = " + ArmConstants.velocityUp);

      armMotor.set(ArmConstants.velocityUp);
    }
    else if(distToTarget > 0 && direction > 0)
    {
      System.out.println("set velocity = " + ArmConstants.velocityDown);

      armMotor.set(ArmConstants.velocityDown);
    }
    else
    {
      System.out.println("Stop Motor");
      armMotor.stopMotor();
    }

    // if (m_profile.isFinished(elapsedTime)) {
    //   m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);

    //   System.out.println("setpoint = " + m_setpoint + " finished setpoint");
    //   System.out.println("current position = " + armEncoder.getPosition());

    //   return;
    // } 
    // else {
    //   m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);

    //   

    //   System.out.println("setpoint = " + m_setpoint);
    //   System.out.println("current position = " + armEncoder.getPosition());
    // }

    
    System.out.println("distance to target = " + distToTarget);
    System.out.println("current position = " + armEncoder.getPosition());
    System.out.println("setpoint = " + m_setpoint);

    // m_feedforward =
    //     Constants.ArmConstants.kArmFeedforward.calculate(
    //         armEncoder.getPosition() + Constants.ArmConstants.kArmZeroCosineOffset, m_targetState.velocity);
    // armController.setReference(
    //     m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
  }

  /**
   * Drives the arm using the provided power value (usually from a joystick). This also adds in the
   * feedforward value which can help counteract gravity.
   *
   * @param _power The motor power to apply.
   */
  // public void runManual(double _power) {
  //   // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
  //   // passively
  //   m_setpoint = armEncoder.getPosition();
  //   updateMotionProfile();
  //   // update the feedforward variable with the newly zero target velocity
  //   m_feedforward =
  //       Constants.ArmConstants.kArmFeedforward.calculate(
  //           armEncoder.getPosition() + Constants.ArmConstants.kArmZeroCosineOffset, m_targetState.velocity);
  //   // set the power of the motor
  //   armMotor.set(_power + (m_feedforward / 12.0));
  //   m_manualValue = _power; // this variable is only used for logging or debugging if needed
  // }

  // @Override
  // public void periodic() { // This method will be called once per scheduler run
  // }

  public void offsetPosition() 
  {
    m_positionResting = armEncoder.getPosition();

    m_positionShooting = m_positionResting + ArmConstants.incShooting;
    m_positionLoading = m_positionResting + ArmConstants.incLoading;
  }

  public void position_Shooting()
  {
    setTargetPosition(m_positionShooting);
  }

  public void position_Loading()
  {
    setTargetPosition(m_positionLoading);
  }

  public void position_Resting()
  {
    setTargetPosition(m_positionResting);
  }

}