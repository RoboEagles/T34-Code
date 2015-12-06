// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package org.usfirst.frc4579.T34.commands;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc4579.T34.Robot;
/**
 *
 */
public class  aimCmd extends Command {
    
    private double percent = 0.0;
    
    public aimCmd() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
	
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.aiming);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }
    // Called just before this Command runs the first time
    protected void initialize() {
        
        update();
        
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
        update();
        Robot.aiming.setAnglePercent(percent);
        
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.aiming.limitReached();
    }
    // Called once after isFinished returns true
    protected void end() {
        
        Robot.aiming.setAngle(0); // Sets angle to zero
        
    }
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
    
    private void update() {
        percent = Robot.oi.getdriveStick().getThrottle();
    }
}
