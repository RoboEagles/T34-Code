// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package org.usfirst.frc4579.T34;
    
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType; import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import java.util.Vector;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static SpeedController driveTrainfrontLeft_M;
    public static SpeedController driveTrainfrontRight_M;
    public static SpeedController driveTrainrearLeft_M;
    public static SpeedController driveTrainrearRight_M;
    public static RobotDrive driveTraindriveBase;
    public static Gyro driveTrainrobotGyro;
    public static Accelerometer driveTrainrobotAccelerometer;
    public static Encoder aimingangleEncoder;
    public static SpeedController aimingaimMotor;
    public static Solenoid shootshootSolenoid;
    public static Solenoid reloadreloadSolenoid;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    public static final int AIM_ENCR_PPR = 250; // Pulses per revolution for aim encoder
        
    public static final int AIM_DRIVE_SPROCKET_DIAMETER = 1; // Diameter (inches) of drive sprocket
    public static final int AIM_ACTIVE_SPROCKET_DIAMETER = 2; // Diameter (inches) of active sprocket
    
    /*

        Settings for Aiming Mechanism

        Finding angle: 
            - Angle = x
            - Encoder pulses per revolution = ppr
            - Drive Sprocket Diameter = dMin
            - Active Sprocket Diameter = dMax
            - Number of pulses = p

        x = ( (360 / ppr) * (dMin / dMax) ) * p

        Distance per pulse should be change in angle per pulse for this application
        There are 360 degrees per revolution, therefore, angle per pulse is 360 divided
        by the number of pulses per revolution. To get angle of drive sprocket. To get angle of 
        active sprocket, mutliply the angle of the drive sprocket by the diameter ratio between
        the sprockets (dMin / dMax). To get the angle, multiply change in angle per pulse by number of pulses.

    */

    
    public static final double SHOOT_TIME = 0.4; // Time solenoid is open for shooting
    public static final double RELOAD_TIME = 0.5; // Time reload solenoid stays extended
    
    public static void init() {
        

        
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainfrontLeft_M = new Victor(1, 1);
	LiveWindow.addActuator("driveTrain", "frontLeft_M", (Victor) driveTrainfrontLeft_M);
        
        driveTrainfrontRight_M = new Victor(1, 2);
	LiveWindow.addActuator("driveTrain", "frontRight_M", (Victor) driveTrainfrontRight_M);
        
        driveTrainrearLeft_M = new Victor(1, 3);
	LiveWindow.addActuator("driveTrain", "rearLeft_M", (Victor) driveTrainrearLeft_M);
        
        driveTrainrearRight_M = new Victor(1, 4);
	LiveWindow.addActuator("driveTrain", "rearRight_M", (Victor) driveTrainrearRight_M);
        
        driveTraindriveBase = new RobotDrive(driveTrainfrontLeft_M, driveTrainrearLeft_M,
              driveTrainfrontRight_M, driveTrainrearRight_M);
	
        driveTraindriveBase.setSafetyEnabled(true);
        driveTraindriveBase.setExpiration(0.1);
        driveTraindriveBase.setSensitivity(0.5);
        driveTraindriveBase.setMaxOutput(1.0);
        driveTrainrobotGyro = new Gyro(1, 1);
	LiveWindow.addSensor("driveTrain", "robotGyro", driveTrainrobotGyro);
        driveTrainrobotGyro.setSensitivity(0.007);
        driveTrainrobotAccelerometer = new Accelerometer(1, 2);
	LiveWindow.addSensor("driveTrain", "robotAccelerometer", driveTrainrobotAccelerometer);
        driveTrainrobotAccelerometer.setSensitivity(1.0);
        driveTrainrobotAccelerometer.setZero(2.5);
        aimingangleEncoder = new Encoder(1, 1, 1, 2, false, EncodingType.k4X);
	LiveWindow.addSensor("aiming", "angleEncoder", aimingangleEncoder);
        aimingangleEncoder.setDistancePerPulse(1.0);
        aimingangleEncoder.setPIDSourceParameter(PIDSourceParameter.kDistance);
        aimingangleEncoder.start();
        aimingaimMotor = new Jaguar(1, 5);
	LiveWindow.addActuator("aiming", "aimMotor", (Jaguar) aimingaimMotor);
        
        shootshootSolenoid = new Solenoid(1, 1);
	LiveWindow.addActuator("shoot", "shootSolenoid", shootshootSolenoid);
        
        reloadreloadSolenoid = new Solenoid(1, 2);
	LiveWindow.addActuator("reload", "reloadSolenoid", reloadreloadSolenoid);
        
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    
        //RESET ROBOT BUILDER ENCODER SETTINGS TO ALIGN WITH OUR PURPOSES
        aimingangleEncoder.stop();
        //SET DISTANCE PER PULSE AS (delta angle / delta pulse) * (sprocket diamter ratio): SEE EARLIER COMMENT ~ LINE 50
        aimingangleEncoder.setDistancePerPulse((double)((360 / AIM_ENCR_PPR)*(AIM_DRIVE_SPROCKET_DIAMETER/AIM_ACTIVE_SPROCKET_DIAMETER)));
        aimingangleEncoder.start();
    
    }
}
