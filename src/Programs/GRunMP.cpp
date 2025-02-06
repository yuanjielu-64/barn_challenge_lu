#include "Programs/GManagerDecomposition.hpp"
#include "Programs/GManagerGoals.hpp"
#include "Programs/GManagerMP.hpp"
#include "Programs/GManagerSimulator.hpp"
#include "Utils/GManager.hpp"
#include "Robot/Jackal.hpp"
#include <iostream>
#include <fstream>

using namespace Antipatrea;

extern "C" int GRunMP(int argc, char **argv) {
    GManager gManager;
    GManagerMP gMP;
    GManagerDecomposition gDecomposition;
    GManagerGoals gGoals;
    GManagerSimulator gSimulator;
    Setup setup;
    Params params;

    ros::init(argc, argv, "runner_node");

    params.ReadFromFile(argv[1]);
    params.ProcessArgs(2, argc - 1, argv);

    Robot_config robot;

    ros::Rate rate(20);

    while (ros::ok()) {
        robot.getCostMap()->updateMap();
        ros::spinOnce();

        if (!robot.setup()) {
            if (robot.getRobotState() == Robot_config::BRAKE_PLANNING)
                rate.sleep();
            continue;
        }

        setup.SetupFromParams(params, robot);
        //setup.UpdateFromParams(params, robot);
        setup.RunConstructDecomposition(params);
        setup.GetDecomposition()->AddGoals();
        setup.GetSimulator()->SetRobot(&robot);

        gDecomposition.SetSetup(&setup);
        gDecomposition.SetManager(&gManager);
        gManager.GetComponents()->push_back(&gDecomposition);

        gGoals.SetSetup(&setup);
        gGoals.SetManager(&gManager);
        gManager.GetComponents()->push_back(&gGoals);

        gSimulator.SetSetup(&setup);
        gSimulator.SetManager(&gManager);
        gManager.GetComponents()->push_back(&gSimulator);

        gMP.SetSetup(&setup);
        gMP.SetManager(&gManager);
        gManager.GetComponents()->push_back(&gMP);

        auto data = params.GetData(Antipatrea::Constants::KW_Graphics);
        if (data && data->m_params) {
            GDrawSetupFromParams(*(data->m_params));
            gManager.SetupFromParams(*(data->m_params));
            gMP.SetupFromParams(*(data->m_params));
            gDecomposition.SetupFromParams(*(data->m_params));
            gGoals.SetupFromParams(*(data->m_params));
            gSimulator.SetupFromParams(*(data->m_params));
        }

        gManager.MainLoop("GRunMP");

        rate.sleep();

    }

    return 0;
}
