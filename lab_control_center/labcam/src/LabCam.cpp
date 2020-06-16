#include "labcam/LabCam.hpp"

using namespace std;


// handler for bus call messages
gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
	bus = bus;
	try
	{
		GMainLoop *loop = (GMainLoop *)data;

		switch (GST_MESSAGE_TYPE(msg)) {

		case GST_MESSAGE_EOS:
			g_print("End of stream\n");
			g_main_loop_quit(loop);
			break;

		case GST_MESSAGE_ERROR: {
			gchar  *debug;
			GError *error;

			gst_message_parse_error(msg, &error, &debug);
            g_printerr ("ERROR from element %s: %s\n", GST_OBJECT_NAME (msg->src), error->message);
            g_printerr ("Debugging info: %s\n", (debug) ? debug : "none");
      
			g_error_free(error);
            g_free(debug);
      
			g_main_loop_quit(loop);
			break;
		}

		default:
			break;
		}

		return TRUE;
	}
	catch (std::exception &e)
	{
		cerr << "An exception occurred in bus_call(): " << endl << e.what() << endl;
		return FALSE;
	}
}

void LabCam::jumppad(void* arg){
	LabCam* cam = (LabCam*)arg;
	cam->startRecordingImpl();

}

void LabCam::startRecording(std::string path, std::string file_name){
	file_name_ = file_name;
	path_ = path;
	cam_thread_ = std::thread(LabCam::jumppad, this);
	cam_thread_.detach();
}


bool LabCam::startRecordingImpl(){

	try
	{

		// initialize GStreamer 
		gst_init(NULL, NULL);

		// create the mainloop
		loop = g_main_loop_new(NULL, FALSE);

		// The InstantCameraForAppSrc will manage the physical camera and pylon driver
		// and provide a source element to the GStreamer pipeline.
		
		// Initialize the camera and driver
		cout << "Initializing camera and driver..." << endl;
		
		// use the GenICam API to access features because it supports any interface (USB, GigE, etc.)
		// reset the camera to defaults if you like
		CInstantCameraAppSrc camera_;
		camera_.ResetCamera();
		// use maximum width and height
		int width = GenApi::CIntegerPtr(camera_.GetNodeMap().GetNode("Width"))->GetMax();
		int height = GenApi::CIntegerPtr(camera_.GetNodeMap().GetNode("Height"))->GetMax();
		int frameRate = 30; // We will try for 30fps. The actual camera capabilities depend on it's settings...
		
		camera_.InitCamera(width, height, frameRate, false, false);		

		cout << "Using Camera             : " << camera_.GetDeviceInfo().GetFriendlyName() << endl;
		cout << "Camera Area Of Interest  : " << camera_.GetWidth() << "x" << camera_.GetHeight() << endl;
		cout << "Camera Speed             : " << camera_.GetFrameRate() << " fps" << endl;

		// create a new pipeline to add elements too
		pipeline = gst_pipeline_new("pipeline");

		// prepare a handler (watcher) for messages from the pipeline
		bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
		bus_watch_id = gst_bus_add_watch(bus, bus_call, loop);
		gst_object_unref(bus);


	    // A pipeline needs a source element. The InstantCameraForAppSrc will create, configure, and provide an AppSrc which fits the camera.
	    GstElement *source = camera_.GetSource();
	    
	    // Create the other needed gstreamer pipeline elements
	    GstElement *convert;
	    GstElement *sink;
	    GstElement *x264enc;

	    convert = gst_element_factory_make("videoconvert", "converter");
	    x264enc = gst_element_factory_make("x264enc", "h264encoder");
	    sink = gst_element_factory_make("filesink", "videosink"); // depending on your platform, you may have to use some alternative here, like ("autovideosink", "sink")

	    std::stringstream filename_stream;
	    filename_stream << path_ << "/" << file_name_ << ".mp4";

	    g_object_set(G_OBJECT(sink), "location", filename_stream.str().c_str(), NULL);

	    if (!convert){ cout << "Could not make convert" << endl; return false; }
	    if (!sink){ cout << "Could not make sink" << endl; return false; }
	    
	    // add and link the pipeline elements
	    gst_bin_add_many(GST_BIN(pipeline), source, convert, x264enc, sink, NULL);
	    if(!gst_element_link_many(source, convert, x264enc, sink, NULL)){
	        std::cout << "FAILED: gst_element_link_many(source, convert, x264enc, sink, NULL)" << std::endl;
	        return false;
	    } 

		cout << "Starting Camera" << endl;
    	camera_.StartCamera();   

		// Start the pipeline.
		cout << "Starting pipeline..." << endl;
		auto stateChangeStatus = gst_element_set_state(pipeline, GST_STATE_PLAYING);

		if(!(stateChangeStatus == GST_STATE_CHANGE_SUCCESS || stateChangeStatus == GST_STATE_CHANGE_ASYNC)){
			std::cout << "FAILED: !(stateChangeStatus == GST_STATE_CHANGE_SUCCESS || stateChangeStatus == GST_STATE_CHANGE_ASYNC)" << std::endl;
			return false;
		}

		g_main_loop_run(loop);

		camera_.StopCamera();
		camera_.CloseCamera();

		return true;

	}
	catch (GenICam::GenericException &e)
	{
		cerr << "An exception occured in main(): " << endl << e.GetDescription() << endl;
		return false;
	}
	catch (std::exception &e)
	{
		cerr << "An exception occurred in main(): " << endl << e.what() << endl;
		return false;
	}
}

void LabCam::stopRecording(){
	if(pipeline == nullptr){
		return;
	}
	std::cout << "sending stop event to pipeline" << std::endl;
	gst_element_send_event(pipeline, gst_event_new_eos());
}
