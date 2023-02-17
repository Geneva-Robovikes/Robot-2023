// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 public PowerDistribution pdp;

 public WPI_TalonFX motor0;
 public WPI_TalonFX motor1;
 public WPI_TalonFX motor2;
 public WPI_TalonFX motor3;
 public WPI_TalonFX motor4;
 public WPI_TalonFX motor5;
 public WPI_TalonFX motor6;
 public WPI_TalonFX motor7;
 public Orchestra orchestra;
 public XboxController xboxController;
 public int songselection;
 public String[] songList;
 public int btn;
 public int lastButton;
 public int selectedsong;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public Robot() {
    xboxController = new XboxController(0);

    orchestra = new Orchestra();
    motor0 = new WPI_TalonFX(0);
    motor1 = new WPI_TalonFX(1);
    motor2 = new WPI_TalonFX(2);
    motor3 = new WPI_TalonFX(3);
    motor4 = new WPI_TalonFX(4);
    motor5 = new WPI_TalonFX(5);
    motor6 = new WPI_TalonFX(6);
    motor7 = new WPI_TalonFX(7);
    
    motor0.setSafetyEnabled(false);
    motor1.setSafetyEnabled(false);
    motor2.setSafetyEnabled(false);
    motor3.setSafetyEnabled(false);
    motor4.setSafetyEnabled(false);
    motor5.setSafetyEnabled(false);
    motor6.setSafetyEnabled(false);
    motor7.setSafetyEnabled(false);
    

    //10 song max
    songList = new String[] {
      "ThroughTheFireAndFlames.chrp",
      "auuugh.chrp"
    };
    songselection = 0;
    lastButton = 0;
  }
  void LoadMusicSelection(int button) {
    songselection = button;
    if (songselection >= songList.length) {
      songselection = 0;
    }
    if (songselection < 0) {
      songselection = songList.length - 1;
    }
    orchestra.loadMusic(songList[songselection]);
    System.out.println(songList[songselection]);
  }
  int getButton() {
    for (int i = 1; i < 10; ++i) {
      if (xboxController.getRawButton(i)) {
      selectedsong = i-1;
     }
   }
   return(selectedsong);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override

  
  public void robotInit() {
    orchestra.loadMusic("ThroughTheFireAndFlames.chrp");
    orchestra.stop();
    
    orchestra.stop();
    orchestra.addInstrument(motor0);
    orchestra.addInstrument(motor1);
    orchestra.addInstrument(motor2);
    orchestra.addInstrument(motor3);
    orchestra.addInstrument(motor4);
    orchestra.addInstrument(motor5);
    orchestra.addInstrument(motor6);
    orchestra.addInstrument(motor7);
    orchestra.stop();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    btn = getButton();

    if (lastButton != btn) {
      //System.out.println(btn);
      orchestra.stop();
      lastButton = btn;
      LoadMusicSelection(btn);
      orchestra.play();
    }
  }


  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

