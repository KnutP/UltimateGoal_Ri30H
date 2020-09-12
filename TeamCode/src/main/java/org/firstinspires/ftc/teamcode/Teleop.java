package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Ri30HTeleop", group = "Iterative Opmode")
//@Disabled
public class Teleop extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = new Robot();
    private enum PivotPosition {in, out}
    private PivotPosition pivotPosition = PivotPosition.in;

    double xVelocity;
    double yVelocity;
    double wVelocity;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap, this);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // ***** DRIVER CODE *****

        // Drivetrain
        yVelocity = gamepad1.left_stick_y*Math.abs(gamepad1.left_stick_y);
        xVelocity = gamepad1.left_stick_x*Math.abs(gamepad1.left_stick_x);
        wVelocity = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x)/2;
        yVelocity = Range.clip(yVelocity, -1.0, 1.0);
        xVelocity = Range.clip(xVelocity, -1.0, 1.0);
        wVelocity = Range.clip(wVelocity, -1.0, 1.0);

        robot.mechanumDrive(xVelocity, yVelocity, wVelocity);


        // ***** OPERATOR CODE *****

        // Pivot positions
        if(gamepad2.dpad_down){
            pivotPosition = PivotPosition.in;
        } else if(gamepad2.dpad_up){
            pivotPosition = PivotPosition.out;
        }
        switch(pivotPosition){
            case in:
                robot.setPivotPosition(1);
                break;
            case out:
                robot.setPivotPosition(0.17);
                break;
            default:
                robot.setPivotPosition(1);
                break;
        }


    }

    @Override
    public void stop() {
    }

}