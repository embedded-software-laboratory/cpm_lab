Settings:

INPUT
RECURSIVE
FILE_PATTERNS
GENERATE_TREEVIEW

Set EXTRACT_ALL to YES if you want undocumented files to appear in the documentation as well

USE_MDFILE_AS_MAINPAGE can also be used instead of the current DoxygenMainpage.hpp

I used EXCLUDE to exclude bcm2835.h, as it contains its own mainpage

Graphviz needs to be installed as well
(so: apt install doxygen graphviz)

TODO: Use \brief more often, add definitions for modules for LCC UI, Commonroad, IPS etc.