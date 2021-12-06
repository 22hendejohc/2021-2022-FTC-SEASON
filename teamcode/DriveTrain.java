package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    Telemetry telemetry;
    DcMotor lw;
    DcMotor rw;
    DcMotor blw;
    DcMotor brw;
    DcMotor frontTwist;
    DcMotor backTwist;

    public void move(String variation, int ticCount){
        int[] ticks = new int[4];
        double lwBasePower;
        double blwBasePower;
        double rwBasePower;
        double brwBasePower;
        lw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switch (variation) {
            case "rotate":
                lw.setTargetPosition(ticCount);
                blw.setTargetPosition(ticCount);
                rw.setTargetPosition(-ticCount);
                brw.setTargetPosition(-ticCount);
                break;
            case "straight":
                lw.setTargetPosition(ticCount);
                blw.setTargetPosition(ticCount);
                rw.setTargetPosition(ticCount);
                brw.setTargetPosition(ticCount);
                break;
            case "diagonalRight":
                blw.setTargetPosition(ticCount);
                rw.setTargetPosition(ticCount);
                break;
            case "diagonalLeft":
                lw.setTargetPosition(ticCount);
                brw.setTargetPosition(ticCount);
                break;
            case "side":
                lw.setTargetPosition(-ticCount);
                blw.setTargetPosition(ticCount);
                rw.setTargetPosition(ticCount);
                brw.setTargetPosition(-ticCount);
                break;
        }
        lw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(lw.isBusy() || blw.isBusy() || rw.isBusy() || brw.isBusy()){
            ticks[0] = Math.abs(lw.getCurrentPosition());
            ticks[1] = Math.abs(blw.getCurrentPosition());
            ticks[2] = Math.abs(brw.getCurrentPosition());
            ticks[3] = Math.abs(rw.getCurrentPosition());
            Arrays.sort(ticks);

            lwBasePower = lw.getCurrentPosition() * 1.0 / ticks[3];
            blwBasePower = blw.getCurrentPosition() * 1.0 / ticks[3];
            rwBasePower = rw.getCurrentPosition() * 1.0 / ticks[3];
            brwBasePower = brw.getCurrentPosition() * 1.0 / ticks[3];

            if(ticks[3] > 1000) {
                lw.setPower(lwBasePower / 3);
                blw.setPower(blwBasePower / 3);
                rw.setPower(rwBasePower / 3);
                brw.setPower(brwBasePower / 2);
            }
            else if(ticks[3] < 500){
                lw.setPower(lwBasePower / 8);
                blw.setPower(blwBasePower / 8);
                rw.setPower(rwBasePower / 8);
                brw.setPower(brwBasePower / 8);
            }
            else {
                lw.setPower(lwBasePower / 5);
                blw.setPower(blwBasePower / 5);
                rw.setPower(rwBasePower / 5);
                brw.setPower(brwBasePower / 5);
            }
            telemetry.addData("tics", lw.getCurrentPosition());
        }

            telemetry.addData("LW Tics", lw.getCurrentPosition());
            telemetry.addData("BLW Tics", blw.getCurrentPosition());
            telemetry.addData("RW Tics", rw.getCurrentPosition());
            telemetry.addData("BRW Tics", brw.getCurrentPosition());
            telemetry.update();

        lw.setPower(0);
        rw.setPower(0);
        blw.setPower(0);
        brw.setPower(0);
    }
}
