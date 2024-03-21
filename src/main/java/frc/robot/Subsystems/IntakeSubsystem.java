//Controls Glossary
//https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controls-glossary.html#term-setpoint

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
    //align angle with apriltags/fixed angle and distance 
    //compensate for movement/pushing?

    //manual control for it 
    //triggers for shoot/take it
    //left joystick for up down movement

    private CANSparkMax intakeNeo;
    private RelativeEncoder intakeEncoder; 

    //Encoder encoder = new Encoder();
    final double setpoint = 0.0; //find setpoint and replace
    

    //replace with PID constants idfk them we have to test
    final double kP = IntakeConstants.intakeP; 
    final double kI = IntakeConstants.intakeI; 
    final double kD = IntakeConstants.intakeD; 

    final double kToleranceDegrees = 2.0; //again idk the tolerance value

   
    // IntakeSubsystem (PIDController pid) 
    // {
        
    //     pid.setTolerance(kToleranceDegrees); 

    //     //i'm not done with this srrryyy add a pid here
    // }

    
    public IntakeSubsystem () 
    {
        intakeNeo = new CANSparkMax(Constants.IntakeConstants.intakeID, CANSparkLowLevel.MotorType.kBrushless);
        
        //find soft limits
        intakeNeo.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        intakeNeo.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        
        intakeEncoder = intakeNeo.getEncoder(); //initializes encoder on DIO pins 0 and 1

        
        //encoder.setPositionConversionFactor(360);
        
        
        intakeNeo.setIdleMode(CANSparkBase.IdleMode.kBrake);
        
        // encoder.setPosition(Constants.IntakeConstants.open);
        intakeNeo.set(0.25);
        
        intakeNeo.burnFlash();

        //random shit
        
        //intakeNeo.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1533);
        //intakeNeo.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -85);

        //PIDController pid = new PIDController(kP, kI, kD);

        //limitSwitch = intakeNeo.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        
        /*
        intakeNeoController.setP(IntakeConstants.intakeP);
        intakeNeoController.setI(IntakeConstants.intakeP);
        intakeNeoController.setD(IntakeConstants.intakeP);
         */
    }

    //setpoint describes the reference of a PID controller
    // public void setSetpoint() {
    //     pid.setSetpoint(setpoint);
    // }
    

    //shooting/loading based off of joystick position --> can simplify later
    //define normalize scale 
    public void IntakeSpeed (double joystickPosition)
    {
        if(joystickPosition > 0.25)
        {
            intakeNeo.set(IntakeConstants.outputSpeed); 
        }
        else if(joystickPosition < -0.25)
        {
            intakeNeo.set(IntakeConstants.intakeSpeed); 
        }
        else
        {
            intakeNeo.stopMotor();
        }
    }

    // public void stopIntake()
    // {
    //     intakeNeo.stopMotor();
    // }


    // public void LoadingSpeed() //have speed value inputed or set in command?
    // {
    //     //put how much we want to move here for loading

    //     intakeNeo.set(IntakeConstants.intakeSpeed); // the % output of the motor, between -1 and 1
    //         // negative = counter-clockwise (?)
    //         // positive = clockwise (?)

    //     //set speed of movement --> no/low/high gradiant speed according to joystick position

    // }

    // public void ShootingSpeed() //have speed value inputed or set in command?
    // {
    //     //put how much we want to move here for shooting
    //     //set direction of movement
    //     intakeNeo.set(IntakeConstants.outputSpeed); // the % output of the motor, between -1 and 1
    //         // negative = counter-clockwise (?)
    //         // positive = clockwise (?)
        
    // }

    //stops motor
   

    /* 
    public void useOutputLoading(double output, double setpoint){
        //use the output from the pid to adjust the intake movement
        loading(output);
    }

    public void useOutputShooting(double output, double setpoint){
        //use the output from the pid to adjust the intake movement
        shooting(output);
    }
    */
}