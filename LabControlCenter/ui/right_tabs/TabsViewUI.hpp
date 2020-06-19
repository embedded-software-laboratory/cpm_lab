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
#include "ui/setup/SetupViewUI.hpp"

class TabsViewUI {
private:
    Glib::RefPtr<Gtk::Builder> tabs_builder;
    std::shared_ptr<SetupViewUI> setup_view_ui;
    std::shared_ptr<VehicleManualControlUi> vehicle_manual_control_ui;
    std::shared_ptr<ParamViewUI> param_view_ui;
    std::shared_ptr<TimerViewUI> timer_view_ui;
    std::shared_ptr<LoggerViewUI> logger_view_ui;
    std::shared_ptr<CommonroadViewUI> commonroad_view_ui;
    Gtk::Notebook* right_notebook;

public:
    TabsViewUI(
        std::shared_ptr<SetupViewUI> setupViewUi, 
        std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi, 
        std::shared_ptr<ParamViewUI> paramViewUI, 
        std::shared_ptr<TimerViewUI> timerViewUi, 
        std::shared_ptr<LoggerViewUI> loggerViewUi,
        std::shared_ptr<CommonroadViewUI> commonroadViewUi
    );
    std::shared_ptr<ParamViewUI> get_param_view();
    bool manual_control_page_active();
    bool param_page_active();
    Gtk::Widget* get_parent();
};