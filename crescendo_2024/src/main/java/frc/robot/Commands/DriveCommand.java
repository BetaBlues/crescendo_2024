package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ChassisSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;

public class DriveCommand extends CommandBase{
    public final ChassisSubsystem chassis;
    public final XboxController controller; 

    public DriveCommand(ChassisSubsystem chassis, XboxController controller){
        this.chassis = chassis; 
        this.controller = controller; 
        addRequirements(chassis);
    }

    @Override
    public void execute(){

        chassis.driveFieldOriented(controller);
    }
}
