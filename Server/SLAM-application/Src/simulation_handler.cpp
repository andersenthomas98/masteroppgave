#include "simulation_handler.h"
#include "MQTT.h"

/*
Robot Simulation
*/

void robot_simulation_thread(NTNU::gui::panel::simulation_panel* simulation_panel, 
								NTNU::application::SLAM::robot_simulation_config* config, 
								boost::fibers::buffered_channel<NTNU::application::SLAM::message>* simulator_ch) 
{
	NTNU::application::SLAM::robot_simulation robot_sim{ *config, *simulator_ch };
	// Grace time (for e.g. MQTT connection)

	boost::this_thread::sleep_for(boost::chrono::seconds{ 3 });

	auto scan_sent = false;

	for (;;)
	{
		//Integrate waiting time into simulation now that simulator is own thread.
		if (simulation_panel->get_robot_sim_enable()) {
			scan_sent = robot_sim.new_run();
			//if (scan_sent)
			//	boost::this_thread::sleep_for(boost::chrono::seconds{ 5 }); //Simulate time to drive to new destination. Should implement receiving messages to simulator.
		}

		boost::this_thread::sleep_for(boost::chrono::milliseconds{ 100 });
	}
}
