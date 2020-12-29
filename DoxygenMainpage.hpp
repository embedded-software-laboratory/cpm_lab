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
 * \subsection Middleware
 * You can find out more about the Middleware here: \ref middleware
 * 
 * \subsection CPM Library
 * You can find out more about the CPM Library here: \ref cpmlib
 */

//Namespace definitions
/**
 * \namespace cpm
 * \brief A namespace that contains all cpm lib functions and classes. See \ref cpmlib.
 */

// Group definitions
/**
 * \defgroup middleware
 * These files are part of the Middleware. You can find out more about the Middleware in https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Middleware+Usage
 * The Middleware relies on QOS_LOCAL_COMMUNICATION.xml - without this configuration file next to the executable, local configuration is not configured properly
 */

/**
 * \defgroup cpmlib
 * These files are all part of the CPM Library. The library was written to be used by all other programs that require DDS, timing or some other
 * common functionality. You can find out more about it here: https://cpm.embedded.rwth-aachen.de/doc/display/CLD/CPM+Library
 */