// RobotBuilder Version: 1.5
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
    public static SpeedController drivetrainFL_Motor;
    public static SpeedController drivetrainFR_Motor;
    public static SpeedController drivetrainBL_Motor;
    public static SpeedController drivetrainBR_Motor;
    public static RobotDrive drivetrainDrivebase;
    public static Solenoid shooterShootSolenoid;
    public static Gyro measurementGyro;
    public static DigitalInput feederIsFed;
    public static SpeedController feederFeed_Motor;
    public static Solenoid ejecterEjectSolenoid;
    public static DigitalInput aimerMaxAngleReached;
    public static SpeedController aimerAimMotor;
    public static Encoder aimerShaftEncoder;
    public static Compressor compressionRobotCompressor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drivetrainFL_Motor = new Talon(0);
        LiveWindow.addActuator("Drivetrain", "FL_Motor", (Talon) drivetrainFL_Motor);
        
        drivetrainFR_Motor = new Talon(1);
        LiveWindow.addActuator("Drivetrain", "FR_Motor", (Talon) drivetrainFR_Motor);
        
        drivetrainBL_Motor = new Talon(2);
        LiveWindow.addActuator("Drivetrain", "BL_Motor", (Talon) drivetrainBL_Motor);
        
        drivetrainBR_Motor = new Talon(3);
        LiveWindow.addActuator("Drivetrain", "BR_Motor", (Talon) drivetrainBR_Motor);
        
        drivetrainDrivebase = new RobotDrive(drivetrainFL_Motor, drivetrainBL_Motor,
              drivetrainFR_Motor, drivetrainBR_Motor);
        
        drivetrainDrivebase.setSafetyEnabled(true);
        drivetrainDrivebase.setExpiration(0.1);
        drivetrainDrivebase.setSensitivity(0.5);
        drivetrainDrivebase.setMaxOutput(1.0);

        shooterShootSolenoid = new Solenoid(0, 0);
        LiveWindow.addActuator("Shooter", "ShootSolenoid", shooterShootSolenoid);
        
        measurementGyro = new Gyro(0);
        LiveWindow.addSensor("Measurement", "Gyro", measurementGyro);
        measurementGyro.setSensitivity(0.007);
        feederIsFed = new DigitalInput(3);
        LiveWindow.addSensor("Feeder", "IsFed", feederIsFed);
        
        feederFeed_Motor = new Talon(4);
        LiveWindow.addActuator("Feeder", "Feed_Motor", (Talon) feederFeed_Motor);
        
        ejecterEjectSolenoid = new Solenoid(0, 1);
        LiveWindow.addActuator("Ejecter", "EjectSolenoid", ejecterEjectSolenoid);
        
        aimerMaxAngleReached = new DigitalInput(2);
        LiveWindow.addSensor("Aimer", "MaxAngleReached", aimerMaxAngleReached);
        
        aimerAimMotor = new Talon(5);
        LiveWindow.addActuator("Aimer", "AimMotor", (Talon) aimerAimMotor);
        
        aimerShaftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        LiveWindow.addSensor("Aimer", "ShaftEncoder", aimerShaftEncoder);
        aimerShaftEncoder.setDistancePerPulse(0.421);
        aimerShaftEncoder.setPIDSourceParameter(PIDSourceParameter.kDistance);
        compressionRobotCompressor = new Compressor(0);
        
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
