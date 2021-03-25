// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

  private static final double kGearRatio = 10.71;
  private static final double kWheelRadiusInches = 3.0;
  private static final double kSensorUnitsPerRotation = 2048;
  private static final double kDriveEncoderToFt =((1.0/kSensorUnitsPerRotation)*(kGearRatio)*(2*kWheelRadiusInches*Math.PI)/12); //1rotation/encoder ticks(2048)*gear ratio(1/10)*wheel diameter*PI/12 in

  private static ADIS16448_IMU imu = new ADIS16448_IMU();


  private WPI_TalonFX falconFL;
  private WPI_TalonFX falconBL;
  private WPI_TalonFX falconFR;
  private WPI_TalonFX falconBR;
  
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28)); //TODO: check on wheel to wheel distance and update
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(.636, 2.35,.358); //ks kv ka

  PIDController leftPIDController = new PIDController(6.38, 0, 2.9); //TODO: GET from drive characterization + tune
  PIDController rightPIDController = new PIDController(6.38, 0, 2.9); 

  Pose2d pose;
  /** Creates a new Drive. */
  public Drive() {
    falconFL = new WPI_TalonFX(1);        
    falconBL = new WPI_TalonFX(4);
    falconBL.follow(falconFL);
    falconFR = new WPI_TalonFX(3);        
    falconBR = new WPI_TalonFX(2);
    falconBR.follow(falconFR);

    imu.reset();
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-imu.getAngle());
}

public DifferentialDriveKinematics getKinematics(){
  return kinematics;
}
public Pose2d getPose2d(){
  return pose;
}
public SimpleMotorFeedforward getFeedforward(){
  return feedforward;
}
public PIDController getLeftPIDController(){
  return leftPIDController;
}
public PIDController getRightPIDController(){
  return rightPIDController;
}
public double getMotor_RPM(WPI_TalonFX motor) {
  return ((motor.getSelectedSensorVelocity() * 600) / kSensorUnitsPerRotation); // actual
}
public double getDriveDistance(WPI_TalonFX motor){
  return Units.feetToMeters(motor.getSelectedSensorPosition()*kDriveEncoderToFt);// this is prob lazy
}
public void reset() {
  odometry.resetPosition(new Pose2d(), getHeading());
}
public DifferentialDriveWheelSpeeds getSpeeds(){ 
  return new DifferentialDriveWheelSpeeds(
      getMotor_RPM(falconFL)*kGearRatio*2*Math.PI*Units.inchesToMeters(3)/60, 
      getMotor_RPM(falconFR)*kGearRatio*2*Math.PI*Units.inchesToMeters(3)/60
  );
}
public void setOutput(double leftVolts,double rightVolts){
  falconFL.set(leftVolts/12);
  falconFR.set(rightVolts/12);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getDriveDistance(falconFL),getDriveDistance(falconFR));

  }
}
