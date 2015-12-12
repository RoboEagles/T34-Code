// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package org.usfirst.frc4579.T34.subsystems;
import com.RoboEagles4579.math.Vector2d;
import com.RoboEagles4579.math.Vector3d;
import org.usfirst.frc4579.T34.RobotMap;
import org.usfirst.frc4579.T34.commands.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
/**
 *
 */
public class measurement extends Subsystem {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    Gyro robotGyro = RobotMap.measurementrobotGyro;
    Accelerometer robotAccelerometer = RobotMap.measurementrobotAccelerometer;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    
    //Keeps track of whether or not the subsystem has been calibrated
    private static boolean CALIBRATED = false;
    
    //Override robotAcceleromter from RobotBuilder with ADXL345_SPI accelerometer (3-axis)
    ADXL345_SPI robotPrimaryAccelerometer = new ADXL345_SPI(1, 2, ADXL345_SPI.DataFormat_Range.k2G);
    
    //Acceleration in Gs
    Vector3d accelVector = new Vector3d(0,0,0),
             lastAccelVector = new Vector3d(0,0,0), //Previous cycle's acceleration measurement
             accelGainPerSecond = new Vector3d(0,0,0); //Loss of accel (Gs) per second
    
    //Stores the calculated relative field position of the robot- disregarding rotation (in feet)
    Vector2d relativeFieldPosition = new Vector2d(0,0);
    
    //Gyro angle in degrees
    double gyroAngle = 0,
           lastGyroAngle = 0, // Previous cycle's gyro measurement
           gyroGainPerSecond = 0; //Loss of angle (degrees) per second
    
    
    private double startTime = 0;
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new measureCmd());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
	
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public double getGyroAngle() {
        return gyroAngle;
    }
    
    public double getGyroRadians() {
        return getGyroAngle() * (Math.PI / 180);
    }
    
    public Vector3d getAccelVector() {
        return accelVector;
    }
    
    public void initialize() {
        robotGyro.reset();
    }
    
    public void calibrate() {
        
        measureRaw();
        
        lastAccelVector.X = accelVector.X;
        lastAccelVector.Y = accelVector.Y;
        lastAccelVector.Z = accelVector.Z;
        lastGyroAngle = gyroAngle;
        
        Timer.delay(RobotMap.CALIBRATION_TIME); //Delays code for some seconds
        
        measureRaw();
        
        accelGainPerSecond.X = (lastAccelVector.X - accelVector.X) / RobotMap.CALIBRATION_TIME;
        accelGainPerSecond.Y = (lastAccelVector.Y - accelVector.Y) / RobotMap.CALIBRATION_TIME;
        accelGainPerSecond.Z = (lastAccelVector.Z - accelVector.Z) / RobotMap.CALIBRATION_TIME;
        gyroGainPerSecond = (lastGyroAngle - gyroAngle) / RobotMap.CALIBRATION_TIME;
        
        startTime = Timer.getFPGATimestamp();
        
        setCalibrated(true);
    }
    
    private void measureRaw() {
        
        accelVector.X = robotPrimaryAccelerometer.getAcceleration(ADXL345_SPI.Axes.kX);
        accelVector.Y = robotPrimaryAccelerometer.getAcceleration(ADXL345_SPI.Axes.kY);
        accelVector.Z = robotPrimaryAccelerometer.getAcceleration(ADXL345_SPI.Axes.kZ);
        gyroAngle = robotGyro.getAngle();
        
    }
    
    public void measure() {
        
        if(!calibrated()) {
            calibrate();
        }
        
        measureRaw();
        
        double time = getTime();
        
        accelVector.X += (accelGainPerSecond.X * time);
        accelVector.Y += (accelGainPerSecond.Y * time);
        accelVector.Z += (accelGainPerSecond.Z * time);
        gyroAngle += (gyroGainPerSecond * time);
        
        relativeFieldPosition.X += (0.5) * accelVector.X * time * time; 
        relativeFieldPosition.Y += (0.5) * accelVector.Y * time * time;
        
    }
    
    public void reset() {
        
        accelVector.reset();
        relativeFieldPosition.reset();
        gyroAngle = 0;
        setCalibrated(false);
        
    }
        
    private double getTime() {
        return Timer.getFPGATimestamp() - startTime;
    }
    
    public double GsToFPS(double Gs) {
        
        return Gs * 32.174; // Takes Gs and returns approximate acceleration in ft/s/s
        
    }
    
    private void setCalibrated(boolean calibrated) {
        CALIBRATED = calibrated;
    }
    
    public boolean calibrated() {
        return CALIBRATED;
    }
}
