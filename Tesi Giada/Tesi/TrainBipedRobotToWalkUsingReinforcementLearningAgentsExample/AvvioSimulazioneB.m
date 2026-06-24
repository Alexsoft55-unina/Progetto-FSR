clear
load('rlWalkingBipedRobotDDPG.mat')
load('trainingDDPGParam2.mat')
run('robotParametersRL.m')
obsInfo = getObservationInfo(agent);
actInfo = getActionInfo(agent);

env = rlSimulinkEnv('rlWalkingBipedRobot','rlWalkingBipedRobot/RL Agent',obsInfo,actInfo);
simOptions = rlSimulationOptions('MaxSteps',500, 'NumSimulations',2);
experience = sim(env,agent,simOptions);