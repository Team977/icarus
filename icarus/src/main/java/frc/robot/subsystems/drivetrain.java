/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

//      1 ======== 0
//      |          |
//      |          |
//      |          |
//      |          |
//      2 ======== 3

  private VictorSP driveMotor0;
  private VictorSP driveMotor1;
  private VictorSP driveMotor2;
  private VictorSP driveMotor3;

  private TalonSRX turnMotor0;
  private TalonSRX turnMotor1;
  private TalonSRX turnMotor2;
  private TalonSRX turnMotor3;

  private AnalogInput turnEncoder0;
  private AnalogInput turnEncoder1;
  private AnalogInput turnEncoder2;
  private AnalogInput turnEncoder3;

  private static double encoderMax = 4.95;

  public drivetrain(){
    super();

    driveMotor0 = new VictorSP(0);
    driveMotor1 = new VictorSP(1);
    driveMotor2 = new VictorSP(2);
    driveMotor3 = new VictorSP(3);

    turnMotor0 = new TalonSRX(0);
    turnMotor1 = new TalonSRX(1);
    turnMotor2 = new TalonSRX(2);
    turnMotor3 = new TalonSRX(3);
    
    turnEncoder0 = new AnalogInput(0);
    turnEncoder1 = new AnalogInput(1);
    turnEncoder2 = new AnalogInput(2);
    turnEncoder3 = new AnalogInput(3);

    turnEncoder0.setAverageBits(2);
    turnEncoder1.setAverageBits(2);
    turnEncoder2.setAverageBits(2);
    turnEncoder3.setAverageBits(2);
  }

  public double[] getEncoderVoltages(){
    double[] encoderVoltages = {turnEncoder0.getAverageVoltage(), turnEncoder1.getAverageVoltage(), turnEncoder2.getAverageVoltage(), turnEncoder3.getAverageVoltage()};
    return encoderVoltages;    
  }

  public double[] getEncoderRadians(){
    double[] encoderAngles = {turnEncoder0.getAverageVoltage(), turnEncoder1.getAverageVoltage(), turnEncoder2.getAverageVoltage(), turnEncoder3.getAverageVoltage()};
    for(int i=0; i<4; i++){
      encoderAngles[i] = Math.PI*(encoderAngles[i]*2/encoderMax-1);
    }
    return encoderAngles;    
  }

  public double[] getEncoderDegrees(){
    double[] encoderAngles = {turnEncoder0.getAverageVoltage(), turnEncoder1.getAverageVoltage(), turnEncoder2.getAverageVoltage(), turnEncoder3.getAverageVoltage()};
    for(int i=0; i<4; i++){
      encoderAngles[i] = 180*(encoderAngles[i]*2/encoderMax-1);
    }
    return encoderAngles;    
  }

  public void setTurnPower(double[] power){
    turnMotor0.set(ControlMode.PercentOutput, power[0]);
    turnMotor1.set(ControlMode.PercentOutput, power[1]);
    turnMotor2.set(ControlMode.PercentOutput, power[2]);
    turnMotor3.set(ControlMode.PercentOutput, power[3]);
  }

  public void setDrivePower(double[] power){
    driveMotor0.set(power[0]);
    driveMotor1.set(power[1]);
    driveMotor2.set(power[2]);
    driveMotor3.set(power[3]);
  }

  public void stopAllMotors(){
    driveMotor0.set(0);
    driveMotor1.set(0);
    driveMotor2.set(0);
    driveMotor3.set(0);
    turnMotor0.set(ControlMode.PercentOutput, 0);
    turnMotor1.set(ControlMode.PercentOutput, 0);
    turnMotor2.set(ControlMode.PercentOutput, 0);
    turnMotor3.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
