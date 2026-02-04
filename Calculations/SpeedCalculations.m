%Near replica of Teensy sketch driveMotorsJ function
clear;clc;close all
[t,x] = driveMotorsJ(1000,1000,100,100,100,100,"s",5,10,10,20);
stairs(t,x)
ylabel("HighSteps")
xlabel("t (seconds)")
dx_dt = gradient(x,t);
dx_dt_2 = gradient(dx_dt);
figure
plot(t,dx_dt)
ylabel("Steps/second")
xlabel("t (seconds)")
function [t,x] = driveMotorsJ(J1step,J2step,J3step,J4step,J5step,J6step,SpeedType,SpeedVal,ACCspd,DCCspd,ACCramp)
    x = 0;
    t = 0;
    minSpeedDelay = 200; %200microsecs/step minimum 5000 steps per second?
    steps = [J1step,J2step,J3step,J4step,J5step,J6step];
    cur = zeros(1,9);
    HighStep = steps(1);
    active = 0;
    Jactive = 0;
    for i = 2:6
        if steps(i)> HighStep
            HighStep = steps(i);
        end
        if steps(i)>=1
            active(i)=1;
            Jactive = Jactive + 1; % Count the number of active motors
        end
    end
    [t,x]=delayMicroSeconds(15,t,x,0);
    calcStepGap = 0;
    ACCStep = HighStep * (ACCspd/100.0);
    DCCStep = HighStep * (DCCspd/100.0);
    NORStep = HighStep - ACCStep - DCCStep;%crusing steps
    speedSP = (SpeedVal * 1000000.0) * 1.0; %speed in microseconds
    if ACCramp <10
        ACCramp = 10;
    end
    k_acc = ACCramp/10;
    k_dec = ACCramp/10;
    
    if (SpeedType == "s" || SpeedType == "m") 
    denom = NORStep + (ACCStep * (1.0 + k_acc) + DCCStep * (1.0 + k_dec)) * 0.5;
    
    if (denom <= 0.0) 
      calcStepGap = speedSP / max(HighStep, 1.0);
    else
      %time delay = (total time)/(number of steps) [with adjustment for
      %acceleration/deceleration]
      calcStepGap = speedSP / denom;
    end
    
    if (calcStepGap < minSpeedDelay)
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    end
    elseif (SpeedType == "p")
    calcStepGap = minSpeedDelay / (SpeedVal / 100.0);
    end
    startDelay = calcStepGap * k_acc;  % slower than cruise
    endDelay = calcStepGap * k_dec;    % slower than cruise
    if ACCStep>0
        calcACCstepInc = (startDelay-calcStepGap)/ACCStep;
    else
        calcACCstepInc = 0;
    end
    if DCCStep>0
        calcDCCstepInc = (endDelay-calcStepGap)/DCCStep;
    else
        calcDCCstepInc = 0;
    end
    
    calcACCstartDel = startDelay;
    curDelay = calcACCstartDel; %if not rounding
    highStepCur = 0;
    %main driving loop
    disp(calcStepGap)
    while ((cur(1) < steps(1) || cur(2) < steps(2) || cur(3) < steps(3) || cur(4) < steps(4) || cur(5) < steps(5) || cur(6) < steps(6)))
        %Determine whether to accelerate, decellerate or cruise
        if (highStepCur <= ACCStep)
            curDelay = curDelay - calcACCstepInc;
        elseif (highStepCur >= (HighStep-DCCStep))
            curDelay = curDelay + calcDCCstepInc;
        else
            curDelay = calcStepGap;
        end
    distDelay = 30;
    disDelayCur = 0;
    for i = 1:6
        if cur(i)<steps(i)
            cur(i)=cur(i)+1;
            %high step remains the same while other joints move
            [t,x] = delayMicroSeconds(distDelay,t,x,highStepCur);
            disDelayCur = disDelayCur + distDelay;
        end
    end
    highStepCur = highStepCur + 1;
    delay = curDelay - disDelayCur;
    if delay < minSpeedDelay
        delay = minSpeedDelay;
    end
    [t,x] = delayMicroSeconds(delay,t,x,highStepCur);
    end 
end
function [t,x] = delayMicroSeconds(microseconds,t,x,highStepCur)
    t(end+1) = t(end)+microseconds*1e-6;
    x(end+1) = highStepCur;
end

