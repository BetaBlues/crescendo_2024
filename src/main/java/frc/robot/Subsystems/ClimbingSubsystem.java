package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.ClimbingSubsystemConstants;

public class ClimbingSubsystem extends SubsystemBase {

    final DoubleSolenoid armSolenoid; 
    final Compressor m_compressor;

    //move a certain distance to the central april tag

    //hang up using double    solenoids (double can maintain position that is different than the start)
    //a button hang up, b button go down
    //double prong single length

    public ClimbingSubsystem(PneumaticsModuleType module, int forwardChannel, int reverseChannel){
        armSolenoid = new DoubleSolenoid(ClimbingSubsystemConstants.canID, module, forwardChannel, reverseChannel);
        armSolenoid.set(DoubleSolenoid.Value.kReverse);

        m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    public void disableCompressor() 
    {
        double pressure = m_compressor.getPressure();

        if(pressure > 118)
        {
            m_compressor.disable();
        }
    }


    public void toggleCommand(){
        armSolenoid.toggle(); 
    }
}