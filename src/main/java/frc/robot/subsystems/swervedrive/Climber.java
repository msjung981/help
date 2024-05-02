package frc.robot.subsystems.swervedrive;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static Climber m_Instance;

    public static Climber getInstance() {
        if (m_Instance == null) {
            m_Instance = new Climber();
        }
        return m_Instance;
    }

    final CANSparkMax leftMotor;
    final CANSparkMax rightMotor;
    final RelativeEncoder m_rightEncoder;
    final RelativeEncoder m_leftEncoder;

    private Climber() {
        leftMotor = new CANSparkMax(Constants.Climber.m_climberLeftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.Climber.m_climberRightMotorID, MotorType.kBrushless);


        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(true);
        m_rightEncoder = rightMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        m_leftEncoder = leftMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    }

    public void stopClimber() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public Command getClimberCommand() {
        return this.startEnd(
            () -> {
                leftMotor.set(Constants.Climber.m_climberRaiseSpeed);
                rightMotor.set(Constants.Climber.m_climberRaiseSpeed);
            },
            () -> {
                leftMotor.set(0);
                rightMotor.set(0);
            }
        );
    }

    public Command retractLeftClimber() {
        return this.startEnd(
            () -> {
                leftMotor.set(Constants.Climber.m_climberLowerSpeed);
            },
            () -> {
                leftMotor.set(0);
            }
        );
    }

    public Command retractRightClimber() {
        return this.startEnd(
            () -> {
                rightMotor.set(Constants.Climber.m_climberLowerSpeed);
            },
            () -> {
                rightMotor.set(0);
            }
        );
    }
    @Override
    public void periodic() {
        // Update any subsystem-specific periodic tasks here
        //System.out.println( m_rightEncoder.getPosition());
        SmartDashboard.putNumber("Encoder Position:", m_rightEncoder.getPosition());
    }

}