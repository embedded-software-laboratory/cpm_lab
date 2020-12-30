#pragma once

#include "defaults.hpp"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <ctime>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <glib.h>

#include "LCCErrorModelRecord.hpp"
#include "LCCErrorLogger.hpp"

/**
 * \brief UI Class for the internal Errors that occured within the LCC, mostly Commonroad-related, that do not lead to crashes and are shown to the user
 * \ingroup lcc_ui
 */
class LCCErrorViewUI {
private:
    Glib::RefPtr<Gtk::Builder> ui_builder;
    Gtk::Box* parent;
    Gtk::TreeView* error_treeview;
    Gtk::Label* error_label_header;
    Gtk::ScrolledWindow* error_scrolled_window;
    Gtk::CheckButton* autoscroll_check_button;
    Gtk::Button* error_button_reset;

    //TreeView Layout, status storage for the UI
    LCCErrorModelRecord error_record;
    Glib::RefPtr<Gtk::ListStore> error_list_store;

    //UI update functions and objects
    void update_ui();
    void dispatcher_callback();
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;

    //Callback function for autoscroll
    void on_size_change_autoscroll(Gtk::Allocation& allocation);

    //Delete old logs
    void reset_list_store();

    //Check for scroll event to turn off automatic scrolling
    bool scroll_callback(GdkEventScroll* scroll_event);

    //Callback for tooltip (to show full message without scrolling)
    bool tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip);

    //Variable for the reset action (is performed within the UI)
    std::atomic_bool reset_logs;

public:
    LCCErrorViewUI();
    ~LCCErrorViewUI();
    Gtk::Widget* get_parent();

    //Might be called from outside, e.g. when a new 'simulation' is run
    void reset();
};