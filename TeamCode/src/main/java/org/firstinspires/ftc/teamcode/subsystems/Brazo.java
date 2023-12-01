package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Brazo extends SubsystemBase {
    DcMotorEx brazo;
    ServoEx avion, avion2;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public Brazo(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        brazo = hardwareMap.get(DcMotorEx.class, "brazo");
        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        avion = new SimpleServo(hardwareMap, "avion", 0, 180, AngleUnit.DEGREES);
        avion2 = new SimpleServo(hardwareMap, "avion2", 0, 180, AngleUnit.DEGREES);
        avion.setInverted(true);
        brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        agarrar();
    }

    public void setPosition(int pos){
        brazo.setTargetPosition(pos);
        brazo.setPower(1);
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void launch(){
        avion.turnToAngle(100);
        avion2.turnToAngle(100);
    }

    public void agarrar(){
        avion.turnToAngle(157);
        avion2.turnToAngle(157);
    }

    public void setPower(int power){
        brazo.setPower(power);
    }

    public boolean isAtSetpoint(){
        boolean isAtPosition = brazo.getCurrentPosition() - brazo.getTargetPosition() < brazo.getTargetPositionTolerance();
        return  isAtPosition;
    }

    public int getPosition(){
        return brazo.getCurrentPosition();
    }

    @Override
    public void periodic(){
        telemetry.addData("brazo", brazo.getCurrentPosition());
    }

}
