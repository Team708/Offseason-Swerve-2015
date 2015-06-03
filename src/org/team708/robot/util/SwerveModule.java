package org.team708.robot.util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

public class SwerveModule {
	
	// Hardware
	private final Encoder rotationEncoder, driveEncoder;			// Encoders for this module
	private final SpeedController rotationMotor, driveMotor;		// Motor controllers for this module
	
	// Software
	private final PIDController rotationPID, drivePID;				// PID controllers for this module
	
	private final double rotationP = 0.0;
	private final double rotationI = 0.0;
	private final double rotationD = 0.0;
	private final double rotationTolerance = 0.0;
	
	private final double driveP = 0.0;
	private final double driveI = 0.0;
	private final double driveD = 0.0;
	private final double driveTolerance = 0.0;
	
	/**
	 * Constructor
	 * @param driveMotor
	 * @param rotationMotor
	 * @param driveEncoder
	 * @param rotationEncoder
	 */
	public SwerveModule(SpeedController driveMotor, SpeedController rotationMotor, Encoder driveEncoder, Encoder rotationEncoder) {
		// Assigns the motors for this module
		this.driveMotor = driveMotor;
		this.rotationMotor = rotationMotor;
		
		// Assigns the encoders for this module
		this.driveEncoder = driveEncoder;
		this.rotationEncoder = rotationEncoder;
		
		// Initialises the PID controllers
		rotationPID = new PIDController(rotationP, rotationI, rotationD, this.rotationEncoder, this.rotationMotor);
		rotationPID.setAbsoluteTolerance(rotationTolerance);
		rotationPID.setContinuous();	// Does this so that the fastest route is taken to an angle (may be an issue later)
		rotationPID.enable();			// Turns on the PID controller
		
		drivePID = new PIDController(driveP, driveI, driveD, this.driveEncoder, this.driveMotor);
		drivePID.setAbsoluteTolerance(driveTolerance);
		drivePID.setContinuous();		// Does this so that the fastest route is taken to an angle (may be an issue later)
		drivePID.enable();				// Turns on the PID controller
	}
	
	/**
	 * @return Raw rotation encoder count
	 */
	public int getRotationEncoderCount() {
		return rotationEncoder.get();
	}
	
	/**
	 * @return Raw drive encoder count
	 */
	public int getDriveEncoderCount() {
		return driveEncoder.get();
	}
	
	/**
	 * @return Rotation based on a preset conversion value
	 */
	public double getRotation() {
		return rotationEncoder.getDistance();
	}
	
	/**
	 * @return Distance based on a preset conversion value
	 */
	public double getDistance() {
		return driveEncoder.getDistance();
	}
	
	/**
	 * Sets the speed for the module's rotation
	 * @param speed
	 */
	public void setRotationSpeed(double speed) {
		rotationMotor.set(speed);
	}
	
	/**
	 * Sets the speed for the module's driving
	 * @param speed
	 */
	public void setDriveSpeed(double speed) {
		driveMotor.set(speed);
	}
}
