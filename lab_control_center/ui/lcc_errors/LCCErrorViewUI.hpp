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
    //! GTK builder for the UI
    Glib::RefPtr<Gtk::Builder> ui_builder;
    //! Parent box of the view, to integrate it into the overall UI
    Gtk::Box* parent;
    //! TreeView that contains the LCC error messages
    Gtk::TreeView* error_treeview;
    //! Label for the treeview
    Gtk::Label* error_label_header;
    //! Window that contains error_treeview to make it scrollable
    Gtk::ScrolledWindow* error_scrolled_window;
    //! Check button to automatically scroll to the latest error message when active
    Gtk::CheckButton* autoscroll_check_button;
    //! Reset button to reset/clear the currently shown error messages
    Gtk::Button* error_button_reset;

    //! Defines the TreeView layout
    LCCErrorModelRecord error_record;
    //! Status storage for the UI, contains all entries of error_treeview
    Glib::RefPtr<Gtk::ListStore> error_list_store;

    //UI update functions and objects
    /**
     * \brief Function called by ui_thread to periodically activate the dispatcher that in turn calls GTK's UI thread, to perform UI updates
     */
    void update_ui();
    /**
     * \brief Callback for GTK's UI dispatcher, all UI changes done during runtime should be performed within this function
     */
    void dispatcher_callback();
    //! To communicate between the current thread and GTK's UI thread
    Glib::Dispatcher ui_dispatcher;
    //! UI thread that periodically ativates the GTK dispatcher to perform an update in the UI
    std::thread ui_thread;
    //! Tells ui_thread if it should still be running
    std::atomic_bool run_thread;

    /**
     * \brief Callback function to enable autoscroll if autoscroll_check_button is used
     * \param allocation Irrelevant parameter that is part of the callback
     */
    void on_size_change_autoscroll(Gtk::Allocation& allocation);

    /**
     * \brief Delete all currently shown error logs
     */
    void reset_list_store();

    /**
     * \brief Check for scroll event to turn off automatic scrolling in case the user scrolls manually through the error list
     * \param scroll_event The scroll event
     */
    bool scroll_callback(GdkEventScroll* scroll_event);

    /**
     * \brief Callback for tooltip (to show full message on mouse hover)
     * \param x x coordinate e.g. of the mouse pointer
     * \param y y coordinate e.g. of the mouse pointer
     * \param keyboard_tooltip If the tooltip was triggered by the keyboard
     * \param tooltip Reference to the tooltip to be shown
     */
    bool tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip);

    //! Variable for the reset action - if true, reset the logs (is performed within the UI)
    std::atomic_bool reset_logs;

public:
    /**
     * \brief Constructor to create the UI element
     */
    LCCErrorViewUI();

    /**
     * \brief Destructor to destroy the UI thread on object destruction
     */
    ~LCCErrorViewUI();

    /**
     * \brief Function to get the parent widget, so that this UI element can be placed within another UI element
     */
    Gtk::Widget* get_parent();

    /**
     * \brief Might be called from outside, e.g. when a new 'simulation' is run, to reset the current error messages
     */
    void reset();
};