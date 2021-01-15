Required packages: apt install doxygen graphviz
The file output is set to ../doxygen, maybe the folder must be generated first (~/dev/doxygen)

Settings:

INPUT (currently redundant, as ./ is given in the end; only ./LICENSE is relevant additionally, because else the file without ending would not be included due to the set file patterns)
RECURSIVE
FILE_PATTERNS (added .bash, .xml, .yaml, .glade, .css, .template, .config, .sh)
GENERATE_TREEVIEW
EXTRACT_STATIC (set to YES)

PROJECT_NAME
PROJECT_BRIEF
PROJECT_LOGO
OUTPUT_DIRECTORY

For struct definitions with attribute in the low_level_controller, to recognize the struct name I had to enable:
ENABLE_PREPROCESSING   = YES
MACRO_EXPANSION        = YES
EXPAND_ONLY_PREDEF     = YES
PREDEFINED             = __attribute__(x)=

Set to yes:
EXTRACT_PRIVATE
EXTRACT_PRIV_VIRTUAL

Set EXTRACT_ALL to YES if you want undocumented files to appear in the documentation as well - currently: NO

USE_MDFILE_AS_MAINPAGE can also be used instead of the current DoxygenMainpage.hpp - currently: NO

EXCLUDE to exclude bcm2835.h, as it contains its own mainpage, as well as the catch.hpp/cpp files, and most .md files, and most package files

SORT_BRIEF_DOCS was set to YES by me - even though this also changes the class list, it is irritating to see functions in a group overview in a non-alphabetical order, so I changed that

TODO:
- LCC Labcam korrekt dokumentieren (zurzeit: Nur Gerüst)
- IPS korrekt dokumentieren (zurzeit: Nur Gerüst)
- In IPS: Main functions entweder nicht dokumentieren oder anders benennen (Doxygen kann nicht mit gleichen Funktionsnamen umgehen)
- Vehicle korrekt dokumentieren (zurzeit: Nur Gerüst)