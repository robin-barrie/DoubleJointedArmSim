// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** This is a sample program to demonstrate the use of arm simulation with existing code. */
public class Robot extends TimedRobot {

  static double topArmLength = 27; //in
  static double bottomArmLength = 27; //in
  static double armPivotX = 55; //65 in = center of 30.5 bumper starting at 49.75
  static double armPivotY = 21.75;//in 

  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;

  // The P gain for the PID controller that drives this arm.
  private static final double kArmKp = 40.0;
  private static final double kArmKi = 0.0;

  double pidOutputTop, pidOutputBottom;
  int topSetpoint, bottomSetpoint;


  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_topController = new ProfiledPIDController(kArmKp, kArmKi, 0, new TrapezoidProfile.Constraints(2, 5));
  private final ProfiledPIDController m_bottomController = new ProfiledPIDController(kArmKp, kArmKi, 0, new TrapezoidProfile.Constraints(2, 5));
  private final Encoder m_topEncoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final Encoder m_bottomEncoder = new Encoder(kEncoderAChannel+2, kEncoderBChannel+2);

  private final PWMSparkMax m_topMotor = new PWMSparkMax(kMotorPort);
  private final PWMSparkMax m_bottomMotor = new PWMSparkMax(kMotorPort+1);
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_armReduction = 600;
  private static final double m_arm_topMass = 10.0; // Kilograms
  private static final double m_arm_topLength = Units.inchesToMeters(topArmLength);
  private static final double m_arm_bottomMass = 4.0; // Kilograms
  private static final double m_arm_bottomLength = Units.inchesToMeters(bottomArmLength);

  private static final int m_arm_top_min_angle = -175; 
  private static final int m_arm_top_max_angle = 175; 
  private static final int m_arm_bottom_min_angle = -30; 
  private static final int m_arm_bottom_max_angle = 210; 


  //SETPOINTS FOR PRESETS MODE (Uses Virtual 4 Bar Mode for smooth movement)
  private static final int startingPositionBottom = -30;
  private static final int startingPositionTop = 175;

  private static final int straightUpBottom = 90;
  private static final int straightUpTop = 0;

  private static final int scoreTravelBottom = -30;
  private static final int scoreTravelTop = 150;//260;

  private static final int intakeBottom = 35;
  private static final int intakeTop = -110;//265;

  private static final int intakeTravelBottom = 65;
  private static final int intakeTravelTop = -143;//265;

  private static final int doubleSubstationBottom = 98;//120//60//60
  private static final int doubleSubstationTop = -105;//-125//125//185;

  private static final int scoreFloorBottom = 120;
  private static final int scoreFloorTop = 150;//255;

  private static final int scoreMidBottom = 80;
  private static final int scoreMidTop = 115;//195;

  private static final int scoreHighBottom = 110;
  private static final int scoreHighTop = 60;//160;

  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_arm_topSim =
      new SingleJointedArmSim(
          m_armGearbox,
          m_armReduction,
          SingleJointedArmSim.estimateMOI(m_arm_topLength, m_arm_topMass),
          m_arm_topLength,
          Units.degreesToRadians(m_arm_top_min_angle),
          Units.degreesToRadians(m_arm_top_max_angle),
          m_arm_topMass,
          false,
          VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final SingleJointedArmSim m_arm_bottomSim =
          new SingleJointedArmSim(
              m_armGearbox,
              m_armReduction,
              SingleJointedArmSim.estimateMOI(m_arm_bottomLength, m_arm_bottomMass),
              m_arm_bottomLength,
              Units.degreesToRadians(m_arm_bottom_min_angle),
              Units.degreesToRadians(m_arm_bottom_max_angle),
              m_arm_bottomMass,
              true,
              VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
              );
  private final EncoderSim m_topEncoderSim = new EncoderSim(m_topEncoder);
  private final EncoderSim m_bottomEncoderSim = new EncoderSim(m_bottomEncoder);
  SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
  SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();


  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  
  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
  private final MechanismRoot2d midNodeHome = m_mech2d.getRoot("Mid Node", 27.83, 0);
  private final MechanismLigament2d MidNode = midNodeHome.append(new MechanismLigament2d("Mid Cone Node", 34, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d highNodeHome = m_mech2d.getRoot("High Node", 10.58, 0);
  private final MechanismLigament2d HighNode = highNodeHome.append(new MechanismLigament2d("High Cone Node", 46, 90, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 49.75, 0);
  private final MechanismLigament2d GridNode = gridHome.append(new MechanismLigament2d("Grid Wall", 49.75, 180, 50, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d dsHome = m_mech2d.getRoot("Double Substation Home", 80.25, 37);//49.75 + bumper length of 30.5
  private final MechanismLigament2d DSRamp = dsHome.append(new MechanismLigament2d("Double Substation Ramp", 13.75, 0, 10, new Color8Bit(Color.kWhite)));
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", armPivotX, armPivotY);
  private final MechanismLigament2d m_arm_bottom =
      m_armPivot.append(
            new MechanismLigament2d(
              "Arm Bottom",
              bottomArmLength, 
              -90, 
              10, 
              new Color8Bit(Color.kHotPink)));
  private final MechanismLigament2d m_arm_tower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));

  private final MechanismLigament2d m_aframe_1 =
      m_armPivot.append(new MechanismLigament2d("aframe1", 24, -50, 10, new Color8Bit(Color.kSilver)));
  private final MechanismLigament2d m_bumper =
      gridHome.append(new MechanismLigament2d("Bumper", 30.5, 0, 60, new Color8Bit(Color.kRed)));
  private final MechanismLigament2d m_arm_top =
      m_arm_bottom.append(
          new MechanismLigament2d(
              "Arm Top",
              topArmLength + 3.0,
              Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
              10,
              new Color8Bit(Color.kPurple)));
    private final MechanismLigament2d m_intake =
    m_arm_top.append(
        new MechanismLigament2d(
            "Intake",
            7,
            Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
            40,
            new Color8Bit(Color.kYellow)));
        

  @Override
  public void robotInit() {
    m_topEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    m_bottomEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);

    SmartDashboard.putNumber("Setpoint top (degrees)", 90);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);

    controlMode.setDefaultOption("Presets (Setpoints)", 0);
    controlMode.addOption("Virtual Four Bar", 1);
    controlMode.addOption("Manual Angle Adjust", 2);
    controlMode.addOption("Joystick X:Y Adjust", 3);

    presetChooser.setDefaultOption("Starting Position", 0);
    presetChooser.addOption("Floor Intake Position", 1);
    presetChooser.addOption("Double Substation Intake", 2);
    presetChooser.addOption("Floor Node Score", 3);
    presetChooser.addOption("Mid Node Score", 4);
    presetChooser.addOption("High Node Score", 5);
    presetChooser.addOption("Straight Up", 6);
    presetChooser.addOption("Score Travel", 7);
    presetChooser.addOption("Intake Travel", 8);

    SmartDashboard.putData(controlMode);
    SmartDashboard.putData(presetChooser);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_arm_topSim.setInput(m_topMotor.get() * RobotController.getBatteryVoltage());
    m_arm_bottomSim.setInput(m_bottomMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_arm_topSim.update(0.020);
    m_arm_bottomSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
    m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
    m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));
  }

  @Override
  public void teleopPeriodic() {

    switch(controlMode.getSelected()){
      
      case 1:
        // Here, we run PID control where the top arm acts like a four-bar relative to the bottom. Maybe?
        topSetpoint = (int) (MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0) - MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 150), m_arm_bottom_min_angle, m_arm_bottom_max_angle), m_arm_top_min_angle, m_arm_top_max_angle));
        bottomSetpoint = (int) MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0), m_arm_bottom_min_angle, m_arm_bottom_max_angle);
        break;
      case 2:
        //Set setpoints manually in SmartDashboard
        topSetpoint = (int) MathUtil.clamp(SmartDashboard.getNumber("Setpoint top (degrees)", 0), m_arm_top_min_angle, m_arm_top_max_angle);
        bottomSetpoint = (int) MathUtil.clamp(SmartDashboard.getNumber("Setpoint bottom (degrees)", 0), m_arm_bottom_min_angle, m_arm_bottom_max_angle);
        break;
      case 3:
        // calculate X and/or Y based on current angles - adjust X and/or Y based on joystick - calaculate new angles (topSetpoint,bottomSetpoint),


        break;
      default: //also case 0, use predefined setpoints

        switch(presetChooser.getSelected()){
          case 0:
            topSetpoint = startingPositionTop;
            bottomSetpoint = startingPositionBottom;
            break;
          case 1:
            topSetpoint = intakeTop;
            bottomSetpoint = intakeBottom;
            break;
          case 2:
            topSetpoint = doubleSubstationTop;
            bottomSetpoint = doubleSubstationBottom;
            break;
          case 3:
            topSetpoint = scoreFloorTop;
            bottomSetpoint = scoreFloorBottom;
            break;
          case 4:
            topSetpoint = scoreMidTop;
            bottomSetpoint = scoreMidBottom;
            break;
          case 5:
            topSetpoint = scoreHighTop;
            bottomSetpoint = scoreHighBottom;
            break;
          case 6:
            topSetpoint = straightUpTop;
            bottomSetpoint = straightUpBottom;
            break;
          case 7:
            topSetpoint = scoreTravelTop;
            bottomSetpoint = scoreTravelBottom;
            break;
          case 8:
            topSetpoint = intakeTravelTop;
            bottomSetpoint = intakeTravelBottom;
            break;
          default:
            topSetpoint = scoreTravelTop;
            bottomSetpoint = scoreTravelBottom;
            break;
        }
        break; 
    }
        // Here, we run PID control where the arm moves to the selected setpoint.
        pidOutputTop = m_topController.calculate(m_topEncoder.getDistance(), Units.degreesToRadians(topSetpoint));
        m_topMotor.setVoltage(pidOutputTop);

        pidOutputBottom = m_bottomController.calculate(m_bottomEncoder.getDistance(), Units.degreesToRadians(bottomSetpoint));
        m_bottomMotor.setVoltage(pidOutputBottom);

        SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
        SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_topMotor.set(0.0);
  }
}
