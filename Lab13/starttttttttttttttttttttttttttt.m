robot.encoders.NewMessageFcn = @newEncoderDataCallback;

global encoderL,encoderL0;
encoderL0=0;
for i=1:5
    robot.sendVelocity(0.1,0.1);
    display(encoderL);
end