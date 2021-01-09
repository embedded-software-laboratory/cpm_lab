//Main page design
/** \mainpage CPM Lab Software
 * 
 * \section intro Introduction
 * Welcome to the official Doxygen Documentation for the CPM Lab Software. 
 * 
 * Use the search bar to search for further information for a particular class,
 * or take a look at the categories below to find out more about the code modules.
 * 
 * Please refer to the official Confluence documentation https://cpm.embedded.rwth-aachen.de/doc/
 * for a detailed explanation of the usage and ideas behind these files
 * 
 * \section projects Software Projects
 * \subsection middleware_section Middleware
 * You can find out more about the Middleware here: \ref middleware
 * 
 * \subsection cpm_section CPM Library
 * You can find out more about the CPM Library here: \ref cpmlib
 * 
 * \subsection lcc_section LCC
 * You can find out more about the LCC here: \ref lcc
 * 
 * You can find out more about the LCC's UI here: \ref lcc_ui
 * 
 * You can find out more about the LCC's Commonroad files here: \ref lcc_commonroad
 * 
 * You can find out more about the LCC's Labcam files here: \ref lcc_labcam
 * 
 * You can find out more about the LCC' Bash and further files here: \ref lcc_further
 * 
 * \subsection ips_section IPS
 * You can find out more about the IPS files here: \ref ips
 * 
 * \subsection vehicle_section Vehicle
 * You can find out more about the Vehicle files here: \ref vehicle
 * 
 * Furthermore, the vehicle's low-level ATMEGA implementation part can be found here: \ref low_level_controller
 * 
 * \subsection tools_section Additional tools
 * Additional tools e.g. for camera calibration can be found in \ref tools
 * 
 * \subsection hlc_section HLC
 * HLC example programs, additional information can be found in \ref examples
 * 
 * You can find out more about the Autostart files here: \ref autostart
 * 
 * Further bash scripts are described in \ref hlc_bash
 */

//Namespace definitions
/**
 * \namespace cpm
 * \brief A namespace that contains all cpm lib functions and classes. See \ref cpmlib.
 */

//Folder definitions
/**
 * \dir ./low_level_controller
 * \brief Folder that contains the low level controller software for the real vehicle's ATMEGA
 * \ingroup low_level_controller
 */

/**
 * \dir ./tools
 * \brief Folder that contains additional Lab Tools
 * \ingroup tools
 */

/**
 * \dir ./high_level_controller/examples
 * \brief Folder that contains examples for HLC programs
 * \ingroup examples
 */

// Group definitions
/**
 * \defgroup middleware
 * These files are part of the Middleware. 
 * 
 * You can find out more about the Middleware in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Middleware+Usage
 * The Middleware relies on QOS_LOCAL_COMMUNICATION.xml - without this configuration file next to the executable, local configuration is not configured properly
 */

//This group is used in Doxygen_middleware_file_descriptions.hpp
/**
 * \defgroup middleware_files
 * \brief Additional files for the middleware software, e.g. Bash scripts
 * \ingroup middleware
 */

/**
 * \defgroup cpmlib
 * These files are all part of the CPM Library. The library was written to be used by all other programs that require DDS, timing or some other
 * common functionality. 
 * 
 * You can find out more about it here: https://cpm.embedded.rwth-aachen.de/doc/display/CLD/CPM+Library
 */

/**
 * \defgroup lcc
 * These files are part of the LCC and not UI or Commonroad Definitions. 
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup lcc_ui
 * These files are part of the LCC UI files. 
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup lcc_commonroad
 * These files are part of the LCC Commonroad files, for parsing and displaying commonroad data. (Except for some additional classes in LCC UI)
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup lcc_labcam
 * These files are part of the LCC Labcam files, for using the cam in the lab (if it is not used in simulated mode).
 * 
 * There is a Readme for setup: \ref LabcamReadme
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

//WARNING: The file Doxygen_lcc_file_descriptions includes further group definitions based on this definition
/**
 * \defgroup lcc_further
 * These files are part of the LCC Bash and other files, for starting/setting up the Lab and for remote Deployment.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup ips
 * These files are part of the Indoor Positioning System Software, which is used in combination with a camera to track the vehicles in the Lab,
 * functioning as an indoor GPS alternative.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Indoor+Positioning+System
 */

/**
 * \defgroup vehicle
 * These files are part of the Vehicle, which is used either for the real vehicles in the Lab on the Raspberry Pis, or
 * for simulated vehicles on the PC that simulates them.
 * 
 * The vehicle also uses the Broadcom library, which is described here: \ref broadcom_page
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Vehicle
 * and in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Vehicle+Setup
 */

//This group is used in Doxygen_mlc_file_descriptions.hpp
/**
 * \defgroup vehicle_files
 * \brief Additional files for the vehicle software, e.g. Bash scripts
 * \ingroup vehicle
 */

/**
 * \defgroup hlc_bash
 * Includes HLC Bash Scripts
 */

/**
 * \defgroup autostart
 * This program is supposed to be set up as an autostart program for the HLC.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Setup+Without+a+NUC+Image
 */

/**
 * \defgroup error_logger
 * \ingroup autostart
 * This program is also used in context with autostart, which is supposed to be set up as an autostart program for the HLC.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Setup+Without+a+NUC+Image
 */

/**
 * \defgroup low_level_controller
 * These files are to be used on the real vehicle's ATMEGA, see https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Vehicle+Setup
 */

/**
 * \defgroup tools
 * These files are additional tools to be used in the Lab.
 */

/**
 * \defgroup examples
 * These files are example files for High Level Controllers that control one or more vehicle(s).
 * 
 * The programs can be very simple, e.g. making a vehicle drive a circle, or include more logic, i.e. to let multiple vehicles
 * drive more complex trajectories while avoiding collisions.
 */

/**
 * \defgroup cpp_examples
 * \ingroup examples
 * C++ examples
 */

/**
 * \defgroup basic_circle
 * \ingroup cpp_examples
 * This is a basic circle-following program for a vehicle.
 */

/**
 * \defgroup basic_line
 * \ingroup cpp_examples
 * This is a basic line-following program for a vehicle.
 */

//WARNING: Sub-groups are defined in the include files, to distinguish the different lane-graphs, so mind that before changing this group definition
/**
 * \defgroup central_routing
 * \ingroup cpp_examples
 * In this scenario, multiple vehicles drive on the Lab map following trajectories by a central routing mechanism that makes sure that the vehicles don't crash.
 */

/**
 * \defgroup controller_test_loop
 * \ingroup cpp_examples
 * TODO
 */

/**
 * \defgroup diagonal_figure_eight
 * \ingroup cpp_examples
 * TODO
 */

/**
 * \defgroup eight_zero
 * \ingroup cpp_examples
 * TODO
 */

/**
 * \defgroup two_vehicles_drive
 * \ingroup cpp_examples
 * TODO
 */

//WARNING: Sub-groups are defined in the Doxygen_description.hpp in the matlab examples folder
/**
 * \defgroup matlab
 * \ingroup examples
 * Matlab examples
 */