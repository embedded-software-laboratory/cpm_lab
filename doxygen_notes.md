Settings:

INPUT
RECURSIVE
FILE_PATTERNS
GENERATE_TREEVIEW

Set EXTRACT_ALL to YES if you want undocumented files to appear in the documentation as well

USE_MDFILE_AS_MAINPAGE can also be used instead of the current DoxygenMainpage.hpp

I used EXCLUDE to exclude bcm2835.h, as it contains its own mainpage

SORT_BRIEF_DOCS was set to YES by me - even though this also changes the class list, it is irritating to see functions in a group overview in a non-alphabetical order, so I changed that

Graphviz needs to be installed as well
(so: apt install doxygen graphviz)

TODO: Use \brief more often, add definitions for modules for LCC UI, Commonroad, IPS etc.
TODO: Some things in commonroad do not seem to be parsed properly