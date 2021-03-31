How to C++ Bibliothek für Go to formation aus Matlab Code generieren

- Pfad zu Dateien: software/tools/go_to_formation/matlab/Coder
- Benötigt Matlab Version R2020a oder neuer für Kompatibilität und DDS Blockset

1.) example.m einmal laufen lassen, um Matlab Coder mit DDS Blockset bekannt zu machen
2.) Klick auf planTrajectory.prj -> öffnet Matlab Coder GUI mit gespeicherten Einstellungen. 
    Diese können übernommen werden, wenn nicht anders gewünscht.
3.) Wichtige Einstellungen für Kompatibilität mit go_to_formation.cpp/.hpp und 
    entsprechender CMakeLists.txt -> Abweichungen von diesen Eintellungen müssten auch dort angepasst werden
	- Build type: Static library
	- Language: C++
	- Interace style: Functions
	- More Settings > Code Appearance > C++ Language Features > C++ namespace: "mgen"

Darstellung der einzelnen GUI Schritte s. Matlab_Coder_Steps.pdf

