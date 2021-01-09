Settings:

INPUT
RECURSIVE
FILE_PATTERNS (added .bash, .xml, .yaml, .glade, .css)
GENERATE_TREEVIEW
EXTRACT_STATIC (set to YES)

PROJECT_NAME
PROJECT_BRIEF
PROJECT_LOGO
OUTPUT_DIRECTORY

Set EXTRACT_ALL to YES if you want undocumented files to appear in the documentation as well

USE_MDFILE_AS_MAINPAGE can also be used instead of the current DoxygenMainpage.hpp

I used EXCLUDE to exclude bcm2835.h, as it contains its own mainpage

SORT_BRIEF_DOCS was set to YES by me - even though this also changes the class list, it is irritating to see functions in a group overview in a non-alphabetical order, so I changed that

Graphviz needs to be installed as well
(so: apt install doxygen graphviz)

The file output is set to ../doxygen, maybe the folder must be generated first (~/dev/doxygen)

TODO: Use \brief more often, add definitions for modules for LCC UI, Commonroad, IPS etc.
- LCC Labcam korrekt dokumentieren (zurzeit: Nur Gerüst)
- IPS korrekt dokumentieren (zurzeit: Nur Gerüst)
- In IPS: Main functions entweder nicht dokumentieren oder anders benennen (Doxygen kann nicht mit gleichen Funktionsnamen umgehen)
- Vehicle korrekt dokumentieren (zurzeit: Nur Gerüst)
- Further .bash files? 
- Should I include the cpp examples as well?