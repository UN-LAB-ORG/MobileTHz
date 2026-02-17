#pragma once

#include "Experimental/imuExp/imuExp.h"
#include "Experimental/powerExp/powerExp.h"
#include "Experimental/rotaryExp/rotaryExp.h"
#include "Experimental/tcpExp/tcpExp.h"
#include "Simulation/imuSim/imuSim.h"
#include "Simulation/positionSim/positionSim.h"
#include "Simulation/powerSim/powerSim.h"
#include "Simulation/rotarySim/rotarySim.h"
#include "Software/kpiClassifier/kpiClassifier.h"
#include "Software/mobileTHzEngine/mobileTHzEngine.h"
#include "Software/plotSaver/plotSaver.h"

struct SharedPointers
{
    std::unique_ptr<rotaryExp> rotaryExp;
    std::unique_ptr<imuExp> imuExp;
    std::unique_ptr<powerExp> powerExp;
    std::unique_ptr<tcpExp> tcpExp;
    std::unique_ptr<rotarySim> rotarySim;
    std::unique_ptr<powerSim> powerSim;
    std::unique_ptr<imuSim> imuSim;
    std::unique_ptr<positionSim> posSim;
    std::unique_ptr<mobileTHzEngine> engine;
    std::unique_ptr<kpiClassifier> kpiClassifier;
    std::unique_ptr<plotSaver> plotSaver;
#if ENABLE_DSO
    ViSession dsoViSession = VI_NULL;
    ViSession defaultRMViSession = VI_NULL;
#endif
};

struct SimulationObjects
{
    // We use unique_ptr for ownership management within the struct
    std::unique_ptr<mobileTHzEngine> engine;
    std::unique_ptr<positionSim> posSim;
    std::unique_ptr<imuSim> imuSim;
    std::unique_ptr<powerSim> powerSim;
    std::unique_ptr<rotarySim> rotarySim;
    std::unique_ptr<kpiClassifier> kpiClassifier;
    std::unique_ptr<plotSaver> plotSaver;
};
