package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ChassisSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ArmActions {
    
     public static Command load(ArmSubsystem arm){        
        Command load = new InstantCommand(() -> arm.position_Loading())
            .andThen(new RunCommand(() -> IntakeSubsystem.intakeNeo.set(Constants.IntakeConstants.intakeSpeed)))
            .withTimeout(1.5); //TODO: time necessary intake
        
        return load;
    }

      public static Command shootAmp(ArmSubsystem arm){
        Command shootAmp = new InstantCommand(() -> arm.position_Shooting())
            .andThen(new RunCommand(() -> IntakeSubsystem.intakeNeo.set(Constants.IntakeConstants.outputSpeed)))
            .withTimeout(2); //TODO: time necessary shoot
        
        return shootAmp;
    }

    public static Command retract(ArmSubsystem arm){
                Command retract = new InstantCommand(() -> arm.position_Resting());

            return retract; 
    }
  

    

}
