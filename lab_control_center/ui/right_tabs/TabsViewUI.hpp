// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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
    //! Shared pointer to the object for the Setup Tab
    std::shared_ptr<SetupViewUI> setup_view_ui;
    //! Shared pointer to the object for the Vehicle Manual Control Tab
    std::shared_ptr<VehicleManualControlUi> vehicle_manual_control_ui;
    //! Shared pointer to the object for the Param View/Edit/Create Tab
    std::shared_ptr<ParamViewUI> param_view_ui;
    //! Shared pointer to the object for the Timer Tab
    std::shared_ptr<TimerViewUI> timer_view_ui;
    //! Shared pointer to the object for the LCC Error Tab
    std::shared_ptr<LCCErrorViewUI> lcc_error_view_ui;
    //! Shared pointer to the object for the Log Tab
    std::shared_ptr<LoggerViewUI> logger_view_ui;
    //! Shared pointer to the object for the Commonroad Tab
    std::shared_ptr<CommonroadViewUI> commonroad_view_ui;
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
     * \brief Provides access to the param tab. Used by the main window, where buttons to load / save / ... parameters from / to files are positioned
     * in the top bar.
     */
    std::shared_ptr<ParamViewUI> get_param_view();

    /**
     * \brief Deprecated. Was true if the current page is the manual control page. No longer workable due to changed structure.
     */
    bool manual_control_page_active();

    /**
     * \brief Deprecated. Was true if the current page is the param page. No longer workable due to changed structure.
     */
    bool param_page_active();

    /**
     * \brief Returns the parent widget of this UI element, to be able to put it within another UI element.
     */
    Gtk::Widget* get_parent();
};