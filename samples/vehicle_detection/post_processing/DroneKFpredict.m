function posPred = DroneKFpredict(XOld, dt, invisbleCount)


dt = (1+invisbleCount)*dt;


SystemModel=[1,0,dt,0,(dt^2)/2,0,0,0;
    0,1,0,dt,0,(dt^2)/2,0,0;
    0,0,1,0,dt,0,0,0;
    0,0,0,1,0,dt,0,0;
    0,0,0,0,1,0,0,0;
    0,0,0,0,0,1,0,0;
    0,0,0,0,0,0,1,dt;
    0,0,0,0,0,0,0,1];

%%1. predicting the motion of the vehicle
xpred=SystemModel*XOld;

posPred = xpred(1:2);