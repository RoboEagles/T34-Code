// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package org.usfirst.frc4579.4yetrihs-T.subsystems;
import org.usfirst.frc4579.4yetrihs-T.RobotMap;
import org.usfirst.frc4579.4yetrihs-T.commands.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
/**
 *
 */
public class shooting extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    Compressor compressor = RobotMap.shootingcompressor;
    Relay shooter = RobotMap.shootingshooter;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void startCompress() {
        compressor.setRelayValue(Relay.Value.kOn);
    }
    
    public void stopCompress() {
         compressor.setRelayValue(Relay.Value.kOff);
    }
    
    public void openSolenoid() {
        shooter.set(Relay.Value.kOn);
    }
    public void closeSolenoid() {
        shooter.set(Relay.Value.kOff);
    }
}
