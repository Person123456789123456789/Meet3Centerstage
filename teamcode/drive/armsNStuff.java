package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.ftccommon.configuration.RobotConfigMap;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;

public class armsNStuff {


    private HardwareMap hardwareMap;
    CRServo Intake;
    //CRServo outtake;
    public armsNStuff(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        Intake = hardwareMap.get(CRServo.class, "Intake");
        //outtake = hardwareMap.get(CRServo.class, "outtake");
    }
    public void spikeMark(double power) {
        Intake.setPower(power);
    }

    public void backdrop(double power) {
       // outtake.setPower(power);


    }

    public void stopPower() {
        Intake.setPower(.3);
       // outtake.setPower(0.0);
    }
}
