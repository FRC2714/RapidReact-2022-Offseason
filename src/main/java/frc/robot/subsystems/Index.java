package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;
import com.revrobotics.CANSparkMaxLowLevel;

public class Index extends SubsystemBase {
    private CANSparkMax indexMotor;
    private CANSparkMax rollerMotor;
    private DigitalInput indexBeam;
    private DigitalInput rollerBeam;

    public enum IndexState {
        SHOOTING,
        INTAKING,
        EXTAKING,
        DEFAULT
    }

    private IndexState indexState = IndexState.DEFAULT;

    /** Creates a new Index. */
    public Index() {
        indexMotor = new CANSparkMax(IndexConstants.kIndexMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerMotor = new CANSparkMax(IndexConstants.kRollerMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        indexBeam = new DigitalInput(IndexConstants.kIndexBeamChannel);
        rollerBeam = new DigitalInput(IndexConstants.kRollerBeamChannel);
    }

    public void setRollerPower(double power) {
        rollerMotor.set(power);
    }

    public void setIndexPower(double power) {
        indexMotor.set(-power);
    }

    public void moveAll(double power) {
       setIndexPower(.5);
       setRollerPower(.5);
    }

    public void disable() {
        moveAll(0);
        System.out.println("disabled(?)");
        setIndexState(IndexState.DEFAULT);
    }

    public boolean getIndexBreakbeam() {
        return indexBeam.get();
    }

    public boolean getRollerBreakbeam() {
        return rollerBeam.get();
    }

    public void setIndexState(IndexState indexState) {
        this.indexState = indexState;
    }

    public void 
    IndexMotion() {
        if (indexState == IndexState.SHOOTING) {
            moveAll(.5);
        } else {
            moveAll(0);
        }
        if (indexState == IndexState.EXTAKING) {
            moveAll(-.5);
        }
        if (indexState == IndexState.DEFAULT) {
            moveAll(0);
        }
        if (indexState == IndexState.INTAKING) {

            setRollerPower(.5);

            if (getIndexBreakbeam()) {
                setIndexPower(0);

                if (getRollerBreakbeam()) {
                    moveAll(.5);
                }
                
            } else {
                moveAll(0);
            }
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("INDEX BEAM", getIndexBreakbeam());
        SmartDashboard.putBoolean("ROLLER BEAM", getRollerBreakbeam());
    }
}