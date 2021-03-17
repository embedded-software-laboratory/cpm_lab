//Main page design
//Also: Most of the group definitions can be found here
//Most file descriptions are placed in according Doxygen....hpp files in the according directories
//Descriptions for the top directory can be found in Doxygen_software_files.hpp


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
 * \section setup_section Setup Files
 * The setup of the CPM Lab Software is explained in detail in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Setup
 * 
 * Setup files, mostly bash scripts, are shortly described in \ref software_files
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
 * \defgroup software_files Software Files
 * These files are part of the top directory and mostly belong to the overall project setup
 */

/**
 * \defgroup middleware Middleware
 * These files are part of the Middleware. 
 * 
 * You can find out more about the Middleware in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Middleware+Usage
 * The Middleware relies on QOS_LOCAL_COMMUNICATION.xml - without this configuration file next to the executable, local configuration is not configured properly
 */

//This group is used in Doxygen_middleware_file_descriptions.hpp
/**
 * \defgroup middleware_files Middleware Files
 * \brief Additional files for the middleware software, e.g. Bash scripts
 * \ingroup middleware
 */

/**
 * \defgroup cpmlib CPM Lib
 * \brief These files are all part of the CPM Library. The library was written to be used by all other programs that require DDS, timing or some other
 * common functionality. 
 * 
 * You can find out more about it here: https://cpm.embedded.rwth-aachen.de/doc/display/CLD/CPM+Library
 */

/**
 * \defgroup cpmlib_idl CPM Lib IDL Files
 * \brief These files are IDL files that define message types for DDS; they should also link to their auto-generated C++ equivalents
 * \ingroup cpmlib
 */

/**
 * \defgroup lcc Lab Control Center
 * \brief These files are part of the LCC and not UI or Commonroad Definitions. 
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup lcc_ui Lab Control Center UI
 * \brief These files are part of the LCC UI files. 
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup lcc_commonroad Lab Control Center Commonroad
 * \brief These files are part of the LCC Commonroad files, for parsing and displaying commonroad data. (Except for some additional classes in LCC UI)
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup lcc_labcam Lab Control Center Labcam
 * \brief These files are part of the LCC Labcam files, for using the cam in the lab (if it is not used in simulated mode).
 * 
 * There is a Readme for setup in this folder as well (not included in the Docs)
 * 
 * The LCC is a graphical program for simulation and control within the Lab.
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

//WARNING: The file Doxygen_lcc_file_descriptions includes further group definitions based on this definition
/**
 * \defgroup lcc_further Further Lab Control Center Files
 * \brief These files are part of the LCC Bash and other files, for starting/setting up the Lab and for remote Deployment.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Lab+Control+Center
 */

/**
 * \defgroup ips Indoor Positioning System
 * \brief These files are part of the Indoor Positioning System Software, which is used in combination with a camera to track the vehicles in the Lab,
 * functioning as an indoor GPS alternative.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Indoor+Positioning+System
 */

/**
 * \defgroup vehicle Vehicle / Mid Level Controller
 * \brief These files are part of the Vehicle, which is used either for the real vehicles in the Lab on the Raspberry Pis, or
 * for simulated vehicles on the PC that simulates them.
 * 
 * The vehicle also uses the Broadcom library.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Vehicle
 * and in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Vehicle+Setup
 */

//This group is used in Doxygen_mlc_file_descriptions.hpp
/**
 * \defgroup vehicle_files Vehicle Files
 * \brief Additional files for the vehicle software, e.g. Bash scripts
 * \ingroup vehicle
 */

/**
 * \defgroup hlc_bash High Level Controller Bash Scripts
 * \brief Includes HLC Bash Scripts
 */

/**
 * \defgroup autostart Autostart
 * \brief This program is supposed to be set up as an autostart program for the HLC.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Setup+Without+a+NUC+Image
 */

/**
 * \defgroup error_logger Error Logger
 * \ingroup autostart
 * \brief This program is also used in context with autostart, which is supposed to be set up as an autostart program for the HLC.
 * 
 * You can find out more in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Setup+Without+a+NUC+Image
 */

/**
 * \defgroup low_level_controller Low Level Controller
 * \brief These files are to be used on the real vehicle's ATMEGA, see https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Vehicle+Setup
 */

//Subgroups for this group are defined in the according folders in tools, where file descriptions are placed as well
/**
 * \defgroup tools Tools
 * \brief These files are additional tools to be used in the Lab.
 */

/**
 * \defgroup examples High Level Controller Examples
 * \brief These files are example files for High Level Controllers that control one or more vehicle(s).
 * 
 * The programs can be very simple, e.g. making a vehicle drive a circle, or include more logic, i.e. to let multiple vehicles
 * drive more complex trajectories while avoiding collisions.
 */

/**
 * \defgroup cpp_examples HLC C++ Examples
 * \ingroup examples
 * \brief C++ examples
 */

/**
 * \defgroup basic_circle Basic Circle Example
 * \ingroup cpp_examples
 * \brief This is a basic circle-following program for a vehicle.
 */

/**
 * \defgroup basic_line Basic Line Example
 * \ingroup cpp_examples
 * \brief This is a basic line-following program for a vehicle.
 */

//WARNING: Sub-groups are defined in the include files, to distinguish the different lane-graphs, so mind that before changing this group definition
/**
 * \defgroup central_routing Central Routing Example
 * \ingroup cpp_examples
 * \brief In this scenario, multiple vehicles drive on the Lab map following trajectories by a central routing mechanism that makes sure that the vehicles don't crash.
 */

/**
 * \defgroup decentral_routing Decentral Routing Example
 * \ingroup cpp_examples
 * \brief In this scenario, multiple vehicles drive on the Lab map following trajectories by a decentral routing mechanism that makes sure that the vehicles don't crash.
 */

/**
 * \defgroup controller_test_loop Controller Test Loop
 * \ingroup cpp_examples
 * \brief TODO
 */

/**
 * \defgroup diagonal_figure_eight Diagonal Figure Eight Example
 * \ingroup cpp_examples
 * \brief TODO
 */

/**
 * \defgroup eight_zero Eight Zero Example
 * \ingroup cpp_examples
 * \brief TODO
 */

/**
 * \defgroup two_vehicles_drive Two Vehicles Drive Example
 * \ingroup cpp_examples
 * \brief TODO
 */

//WARNING: Sub-groups are defined in the Doxygen_description.hpp in the matlab examples folder
/**
 * \defgroup matlab HLC Matlab Examples
 * \ingroup examples
 * \brief Matlab examples
 */