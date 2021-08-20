import networkx as nx
import matplotlib.pyplot as plt

G = nx.DiGraph()

G.add_node("Joystick")
G.add_node("HLCReadyAggregator")
G.add_node("LCCErrorLogger")
G.add_node("LogLevelSetter")
G.add_node("LogStorage")
G.add_node("ObstacleAggregator")
G.add_node("ObstacleSimulation")
G.add_node("ObstacleSimulationManager")
G.add_node("ParameterServer")
G.add_node("ParameterStorage")
G.add_node("ParameterWithDescription")
G.add_node("ProgramExecutor")
G.add_node("RTTAggregator")
G.add_node("TimerTrigger")
G.add_node("TimeSeries")
G.add_node("TimeSeriesAggregator")
G.add_node("TrajectoryCommand")
G.add_node("VehicleAutomatedControl")
G.add_node("VehicleManualControl")
G.add_node("VisualizationCommandsAggregator")
G.add_node("CommonroadViewUI")
G.add_node("ObstacleToggle")
G.add_node("FileChooserUI")
G.add_node("FileSaverUI")
G.add_node("LCCErrorViewUI")
G.add_node("LoggerViewUI")
G.add_node("VehicleManualControlUi")
G.add_node("MapViewUi")
G.add_node("MonitoringUi")
G.add_node("ParamsCreateView")
G.add_node("ParamsViewUI")
G.add_node("TabsViewUI")
G.add_node("CrashChecker")
G.add_node("Deploy")
G.add_node("SetupViewUI")
G.add_node("Upload")
G.add_node("UploadWindow")
G.add_node("VehicleToggle")
G.add_node("TimerViewUI")
G.add_node("MainWindow")
G.add_node("VehicleData")

G.add_edge("ObstacleSimulationManager", "CommonRoadScenario")

G.add_edge("ParameterServer", "ParameterStorage")

G.add_edge("TimeSeriesAggregator", "VehicleData")

G.add_edge("VehicleManualControl", "Joystick")

G.add_edge("CommonroadViewUI", "CommonroadScenario")
G.add_edge("CommonroadViewUI", "ObstacleSimulationManager")
G.add_edge("CommonroadViewUI", "ObstacleToggle")

G.add_edge("LoggerViewUI", "LogStorage")

G.add_edge("VehicleManualControlUi", "VehicleManualControl")

G.add_edge("MapViewUi", "CommonRoadScenario")
G.add_edge("MapViewUi", "VehicleData")
G.add_edge("MapViewUi", "TrajectoryCommand")

G.add_edge("MonitoringUi", "VehicleData")
G.add_edge("MonitoringUi", "Deploy")
G.add_edge("MonitoringUi", "CrashChecker")

G.add_edge("ParamsViewUI", "ParameterStorage")
G.add_edge("ParamsViewUI", "ParamsCreateView")

G.add_edge("TabsViewUI", "SetupViewUI")
G.add_edge("TabsViewUI", "VehicleManualControlUi")
G.add_edge("TabsViewUI", "ParamViewUI")
G.add_edge("TabsViewUI", "TimerViewUI")
G.add_edge("TabsViewUI", "LCCErrorViewUI")
G.add_edge("TabsViewUI", "LoggerViewUI")
G.add_edge("TabsViewUI", "CommonroadViewUI")

G.add_edge("CrashChecker", "Deploy")
G.add_edge("CrashChecker", "HLCReadyAggregator")
G.add_edge("CrashChecker", "Upload")

G.add_edge("Deploy", "ProgramExecutor")

G.add_edge("SetupViewUI", "VehicleToggle")
G.add_edge("SetupViewUI", "Deploy")
G.add_edge("SetupViewUI", "VehicleAutomatedControl")
G.add_edge("SetupViewUI", "HLCReadyAggregator")
G.add_edge("SetupViewUI", "Upload")
G.add_edge("SetupViewUI", "CrashChecker")
G.add_edge("SetupViewUI", "FileChooserUI")

G.add_edge("Upload", "Deploy")
G.add_edge("Upload", "UploadWindow")

G.add_edge("TimerViewUI", "TimerTrigger")
G.add_edge("TimerViewUI", "CrashChecker")

G.add_edge("MainWindow", "FileChooserUI")
G.add_edge("MainWindow", "FileSaverUI")
G.add_edge("MainWindow", "TabsViewUI")
G.add_edge("MainWindow", "MonitoringUi")
G.add_edge("MainWindow", "MapViewUi")

G.add_node("ObstacleAggregator")
G.add_edge("ObstacleAggregator", "CommonRoadScenario")

subax2 = plt.subplot(122)
print(nx.find_cycle(G, orientation="original"))
nx.draw(G, with_labels=True, font_weight='bold')
plt.show()