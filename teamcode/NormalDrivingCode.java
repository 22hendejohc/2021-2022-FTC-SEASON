package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;


@TeleOp(name = "TestDrive", group =  "Telep")
public class NormalDrivingCode extends LinearOpMode {

    //Motors
    DcMotor lw;
    DcMotor rw;
    DcMotor blw;
    DcMotor brw;
    DcMotor lwR;
    DcMotor rwR;
    DcMotor extendo;

    Servo rotate;

    //GyroSensor gyro;

    //Speed
    double speedAdjust = 7.5;

    int notdone = 0;

    // Gyro settings (works fine but can be improved
    double adjSpeed = 0.027;
    double minTurn = 0.007;
    int windowSize = 1;
    int targetDegree = 0;
    // piecewise graph function settings
    int piecewiseWindow = 30;
    double piecewiseSpeed = 0.007517647057771725;
    double piecewiseMinTurn = 0.004;


    // Line to make the power variables public
    public double leftPower, rightPower, backLeftPower, backRightPower, leftPowerR, rightPowerR;
    boolean normalMove = true;
    // button variables used for button processing

    double timeARunTime;

    // Function to make invalid degrees valid
    public int degreeCalc(int degree){
        int returnDegree = degree;
        if(returnDegree < 0){
            returnDegree = returnDegree + 360;
        }
        if(returnDegree >= 360){
            returnDegree = returnDegree - 360;
        }
        return returnDegree;

    }

    // Turn function
    // when called to it sets the turn power variables to turn right the desired amount
    public void turnPower(double amount){
        // use a positive parameter to turn right (clockwise)
        // use a negative parameter to turn left (counterclockwise)
        leftPower += amount;
        rightPower -= amount;
        backLeftPower += amount;
        backRightPower -= amount;
    }

    public void toPosition(){

        int[] ticks = new int[4];
        //Set the target position and tell motors to go to it
        lw.setTargetPosition(0);
        blw.setTargetPosition(0);
        brw.setTargetPosition(0);
        rw.setTargetPosition(0);
        lw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //While motors are going to target position, update the speed at different rates based on their ticks
        while(lw.isBusy() || blw.isBusy() || rw.isBusy() || brw.isBusy()){
            ticks[0] = Math.abs(lw.getCurrentPosition());
            ticks[1] = Math.abs(blw.getCurrentPosition());
            ticks[2] = Math.abs(rw.getCurrentPosition());
            ticks[3] = Math.abs(brw.getCurrentPosition());
            Arrays.sort(ticks);
            lw.setPower(-lw.getCurrentPosition()/ticks[3]);
            blw.setPower(-blw.getCurrentPosition()/ticks[3]);
            rw.setPower(-rw.getCurrentPosition()/ticks[3]);
            brw.setPower(-brw.getCurrentPosition()/ticks[3]);
            telemetry.addData("tics", lw.getCurrentPosition());
        }
        //After motors are done, set power to 0 to all of them.
        lw.setPower(0);
        blw.setPower(0);
        rw.setPower(0);
        brw.setPower(0);

    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lw = hardwareMap.dcMotor.get("lw");
        rw = hardwareMap.dcMotor.get("rw");
        rw.setDirection(DcMotor.Direction.REVERSE);
        brw = hardwareMap.dcMotor.get("brw");
        brw.setDirection(DcMotor.Direction.REVERSE);
        blw = hardwareMap.dcMotor.get("blw");
        lwR = hardwareMap.dcMotor.get("lwR");
        rwR = hardwareMap.dcMotor.get("rwR");
        rwR.setDirection(DcMotor.Direction.REVERSE);
        extendo = hardwareMap.dcMotor.get("extend");

        rotate = hardwareMap.servo.get("rotate");
        rotate.setPosition(0.5);
        //gyro = hardwareMap.gyroSensor.get("gyro");
        //gyro.calibrate();
        boolean buttonA = false;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            //int heading = gyro.getHeading();

           //telemetry.addData("1. Heading: ", heading);

            telemetry.addData("lw ticks:", lw.getCurrentPosition());
            telemetry.addData("lw ticks:", rw.getCurrentPosition());
            telemetry.addData("rightControlX", gamepad1.right_stick_x);
            telemetry.addData("rightControlY", gamepad1.right_stick_y);
            telemetry.addData("Time", time);
            telemetry.addData("runtimeA", timeARunTime);

            if (gamepad1.a) {
                if (buttonA) {
                    buttonA = false;
                    if (normalMove) {
                        normalMove = false;
                    }
                    else {
                        normalMove = true;
                    }
                }
            }
            else{

                buttonA = true;
            }
            if (normalMove) {
                leftPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10);
                rightPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10);
                backLeftPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10);
                backRightPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10);

                leftPowerR = (0);
                rightPowerR = (0);
            }
            else {
                leftPower = (0);
                rightPower = (0);
                backLeftPower = (0);
                backRightPower = (0);

                leftPowerR = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10);
                rightPowerR = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10);
            }

            if(gamepad1.x){
                rotate.setPosition(1);
            }
            if(gamepad1.y){
                rotate.setPosition(.5);
            }
            if(gamepad1.b){
                rotate.setPosition(0);
            }

            // Inputs to the motors

            // targetDegree is currently affected by the right stick and it changes the degrees the robot goes towards
            // targetDegree += gamepad1.right_stick_x;
            // targetDegree = degreeCalc(targetDegree);

            /* if (degreeCalc(gyro.getHeading() - targetDegree) > windowSize + piecewiseWindow && degreeCalc(gyro.getHeading() - targetDegree) <= 180) {
                if ((Math.pow(degreeCalc(gyro.getHeading() - targetDegree) * adjSpeed, 2)) >= minTurn) {
                    turnPower(-( Math.pow(degreeCalc(gyro.getHeading() - targetDegree) * adjSpeed, 2)));
                }
                else{
                    turnPower(-minTurn);
                }
            }

            if (degreeCalc(gyro.getHeading() - targetDegree) < 360 - windowSize - piecewiseWindow && degreeCalc(gyro.getHeading() - targetDegree) > 180) {
                if ((Math.pow((360 - degreeCalc(gyro.getHeading() - targetDegree)) * adjSpeed, 2) >= minTurn)) {
                    turnPower(Math.pow((360 - degreeCalc(gyro.getHeading() - targetDegree)) * adjSpeed, 2));
                }
                else {
                    turnPower(minTurn);
                }
            }
            // Second graph function (piecewise) the one that is closer to 0 degrees
            if (degreeCalc(gyro.getHeading() - targetDegree) > windowSize && degreeCalc(gyro.getHeading() - targetDegree) <= piecewiseWindow + windowSize) {
                if ((Math.sqrt(degreeCalc(gyro.getHeading() - targetDegree) * piecewiseSpeed)) >= minTurn) {
                    turnPower(-(Math.sqrt(degreeCalc(gyro.getHeading() - targetDegree) * piecewiseSpeed)));
                }
                else {
                    turnPower(-minTurn);
                }
            }

            if (degreeCalc(gyro.getHeading() - targetDegree) < 360 - windowSize && degreeCalc(gyro.getHeading() - targetDegree) > 360 - piecewiseWindow - windowSize) {
                if (Math.sqrt((360 - degreeCalc(gyro.getHeading() - targetDegree)) * piecewiseSpeed) >= minTurn) {
                    turnPower(Math.sqrt((360 - degreeCalc(gyro.getHeading() - targetDegree)) * piecewiseSpeed) + piecewiseMinTurn);
                }
                else {
                    turnPower(minTurn);
                }
            }*/

            lw.setPower(leftPower);
            rw.setPower(rightPower);
            blw.setPower(backLeftPower);
            brw.setPower(backRightPower);
            rwR.setPower(rightPowerR);
            lwR.setPower(leftPowerR);

        }
    }
}