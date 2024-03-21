package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import frc.PIDGains;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SeeSawConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SeeSawSubsystem extends SubsystemBase {

    private CANSparkMax seeSawController;
    private RelativeEncoder seeSawEncoder;
    private SparkPIDController seeSawPIDController;
    private double m_setpoint;
    private double m_direction;

    public SeeSawSubsystem() {

        // Create a new object to control a SPARK MAX motor Controller
        seeSawController = new CANSparkMax(Constants.SeeSawConstants.kCanId, MotorType.kBrushless);
        seeSawController.setInverted(false);
        seeSawController.setSmartCurrentLimit(Constants.SeeSawConstants.kCurrentLimit); //edit depending on arm vs seesaw
        seeSawController.setIdleMode(IdleMode.kBrake); 
        seeSawController.enableSoftLimit(SoftLimitDirection.kForward, true);
        seeSawController.enableSoftLimit(SoftLimitDirection.kReverse, true);
        seeSawController.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.SeeSawConstants.kSoftLimitForward);
        seeSawController.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.SeeSawConstants.kSoftLimitReverse);

        // set up the motor encoder including conversion factors to convert to radians and radians per
        // second for position and velocity
        seeSawEncoder = seeSawController.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42); 
        //seeSawEncoder.setPositionConversionFactor(Constants.SeeSawConstants.kPositionFactor);
        //seeSawEncoder.setVelocityConversionFactor(Constants.SeeSawConstants.kVelocityFactor);
        //seeSawEncoder.setPosition(0.0);

        seeSawPIDController = seeSawController.getPIDController();
        PIDGains.setSparkMaxGains(seeSawPIDController, Constants.SeeSawConstants.kPIDGains);

        seeSawController.burnFlash();

        m_setpoint = seeSawEncoder.getPosition(); // position = 0 at this point

        m_direction = 0.0;

    }

    public void moveToInputPosition()
    {
        System.out.println("seesaw setpoint = " + m_setpoint);
        System.out.println("current position = " + seeSawEncoder.getPosition());
        setTargetPosition(Constants.SeeSawConstants.kPositionInput);
    }
  
    public void moveToOutputPosition()
    {
        System.out.println("seesaw setpoint = " + m_setpoint);
        System.out.println("current position = " + seeSawEncoder.getPosition());
        setTargetPosition(Constants.SeeSawConstants.kPositionOutput);
    }

    public void stopMotor()
    {
        System.out.println("Stop Motor");
        System.out.println("current position = " + seeSawEncoder.getPosition());
        seeSawController.stopMotor();
    }


  /**
   * Sets the target position and updates the motion profile if the target position changed.
   *
   * @param _setpoint The new target position.
   */
  public void setTargetPosition(double pGoal) 
  {
    if (seeSawEncoder.getPosition() != pGoal) 
    {
      m_setpoint = pGoal;

      if(pGoal > seeSawEncoder.getPosition())
      {
        m_direction = 1.0;
      }
      else if (pGoal < seeSawEncoder.getPosition())
      {
        m_direction = -1.0;
      }
      else
      {
        m_direction = 0;
      }

      System.out.println("proceeding to setpoint = " + m_setpoint);
    }
  }

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

    distToTarget = m_setpoint - seeSawEncoder.getPosition();
    if(Math.abs(distToTarget) < 25)
    {
        System.out.println("Target Found - Stop Motor");
        seeSawController.stopMotor();
        return;
    }
    if(distToTarget < 0 && m_direction < 0)
    {
        System.out.println("set velocity = " + SeeSawConstants.kVelocityInput);
        seeSawController.set(SeeSawConstants.kVelocityInput);
    }
    else if(distToTarget > 0 && m_direction > 0)
    {
        System.out.println("set velocity = " + SeeSawConstants.kVelocityOutput);
        seeSawController.set(SeeSawConstants.kVelocityOutput);
    }
    else
    {
        System.out.println("Stop Motor");
        seeSawController.stopMotor();
    }
  }    
}
