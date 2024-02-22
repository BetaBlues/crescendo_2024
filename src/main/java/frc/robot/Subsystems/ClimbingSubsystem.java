package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClimbingSubsystem {

    final DoubleSolenoid armSolenoid; 


    //move a certain distance to the central april tag

    //hang up using double solenoids (double can maintain position that is different than the start)
    //a button hang up, b button go down
    //double prong single length

    public ClimbingSubsystem(PneumaticsModuleType module, int forwardChannel, int reverseChannel){
        armSolenoid = new DoubleSolenoid(module, forwardChannel, reverseChannel);
        armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }


    public void toggleCommand(){
        armSolenoid.toggle(); 
    }
}