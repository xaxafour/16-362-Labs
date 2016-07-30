function newEncoderDataCallback(obj, msg)
global encoderL0 encoderR0 encoderT0 encoderL encoderR encoderT i

if i == 1
    encoderL0 = double(msg.Left) / 1000.0;
    encoderR0 = double(msg.Right) / 1000.0;
    encoderT0 = double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec) / 1000000000.0;
    encoderL = 0;
    encoderR = 0;
    encoderT = 0;
else
    encoderL = double(msg.Left)/1000.0 - encoderL0;
    encoderR = double(msg.Right)/1000.0 - encoderR0;
    encoderT = (double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nsec) / 1000000000.0) - encoderT0;
end