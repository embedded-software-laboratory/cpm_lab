#pragma once

#include "defaults.hpp"
#include <cassert>
#include <memory>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/commonroad/CommonroadViewUI.hpp"
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/params/ParamViewUI.hpp"
#include "ui/timer/TimerViewUI.hpp"
#include "ui/logger/LoggerViewUI.hpp"
#include "ui/lcc_errors/LCCErrorViewUI.hpp"
#include "ui/setup/SetupViewUI.hpp"

/**
 * \class TabsViewUI
 * \brief LCC UI class for tabs on the right side, which hold most of the pages of the UI, e.g. params, commonroad or the timer
 * \ingroup lcc_ui
 */
class TabsViewUI {
private:
    //! GTK UI builder
    Glib::RefPtr<Gtk::Builder> tabs_builder;
    //! GTK notebook, contains all tabs shown in the UI
    Gtk::Notebook* right_notebook;

public:
    /**
     * \brief Constructor, creates a TabsViewUI object in which the given UI elements are shown as Tabs between which the user can switch by clicking on
     * the tab notebook.
     * \param setupViewUi Setup Tab
     * \param vehicleManualControlUi Vehicle Manual Control Tab
     * \param paramViewUI Param View / Edit / Create Tab
     * \param timerViewUi Timer Tab
     * \param lccErrorViewUi LCC Error Tab
     * \param loggerViewUi Log Tab
     * \param commonroadViewUi Commonroad Tab
     */
    TabsViewUI(
        std::shared_ptr<SetupViewUI> setupViewUi, 
        std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, 
        std::shared_ptr<ParamViewUI> paramViewUI, 
        std::shared_ptr<TimerViewUI> timerViewUi, 
        std::shared_ptr<LCCErrorViewUI> lccErrorViewUi, 
        std::shared_ptr<LoggerViewUI> loggerViewUi,
        std::shared_ptr<CommonroadViewUI> commonroadViewUi
    );

    /**
     * \brief Returns the parent widget of this UI element, to be able to put it within another UI element.
     */
    Gtk::Widget* get_parent();
};