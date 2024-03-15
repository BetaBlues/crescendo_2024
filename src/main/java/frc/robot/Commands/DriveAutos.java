package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.ChassisSubsystem;


public class DriveAutos {
    

    public static Command manualLeave(ChassisSubsystem chassis){
        
        Command manualLeave = new RunCommand(() -> chassis.driveCartesian(0.7, 0, 0), chassis).withTimeout(3);
  
        return manualLeave;
    }

    
}
