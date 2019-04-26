// ParametrizeCamera_UserSets.cpp
/*
    Note: Before getting started, Basler recommends reading the Programmer's Guide topic
    in the pylon C++ API documentation that gets installed with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the Migration topic in the pylon C++ API documentation.

    Demonstrates how to use user configuration sets (user sets) and how to configure the camera
    to start up with the user defined settings of user set 1.

    You can also configure your camera using the pylon Viewer and
    store your custom settings in a user set of your choice.


    ATTENTION:
    Executing this sample will overwrite all current settings in user set 1.
*/

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

// Namespace for using pylon objects.
using namespace Pylon;

#if defined( USE_1394 )
// Setting for using  Basler IEEE 1394 cameras.
#include <pylon/1394/Basler1394InstantCamera.h>
typedef Pylon::CBasler1394InstantCamera Camera_t;
using namespace Basler_IIDC1394CameraParams;
#elif defined ( USE_GIGE )
// Setting for using Basler GigE cameras.
#include <pylon/gige/BaslerGigEInstantCamera.h>
typedef Pylon::CBaslerGigEInstantCamera Camera_t;
using namespace Basler_GigECameraParams;
#elif defined ( USE_CAMERALINK )
// Setting for using Basler Camera Link cameras.
#include <pylon/cameralink/BaslerCameraLinkInstantCamera.h>
typedef Pylon::CBaslerCameraLinkInstantCamera Camera_t;
using namespace Basler_CLCameraParams;
#elif defined ( USE_USB )
// Setting for using Basler USB cameras.
#include <pylon/usb/BaslerUsbInstantCamera.h>
typedef Pylon::CBaslerUsbInstantCamera Camera_t;
using namespace Basler_UsbCameraParams;
#elif defined ( USE_BCON )
// Settings for using Basler BCON cameras.
#include <pylon/bcon/BaslerBconInstantCamera.h>
typedef Pylon::CBaslerBconInstantCamera Camera_t;
using namespace Basler_BconCameraParams;
#else
#error Camera type is not specified. For example, define USE_GIGE for using GigE cameras.
#endif

// Namespace for using cout.
using namespace std;

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Before using any pylon methods, the pylon runtime must be initialized. 
    PylonInitialize();

    try
    {
        // Only look for cameras supported by Camera_t
        CDeviceInfo info;
        info.SetDeviceClass( Camera_t::DeviceClass());

        // Create an instant camera object with the first found camera device matching the specified device class.
        Camera_t camera( CTlFactory::GetInstance().CreateFirstDevice( info));

        // Print the model name of the camera.
        cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

        // Open the camera.
        camera.Open();

        // Remember the current default user set selector so we can restore it later when cleaning up.
#if defined( USE_USB ) || defined( USE_BCON )
        UserSetDefaultEnums oldDefaultUserSet = camera.UserSetDefault.GetValue();
#else
        UserSetDefaultSelectorEnums oldDefaultUserSet = camera.UserSetDefaultSelector.GetValue();
#endif

        // Load default settings.
        cout << "Loading default settings" << endl;
        camera.UserSetSelector.SetValue(UserSetSelector_Default);
        camera.UserSetLoad.Execute();

        // Set gain and exposure time values.
        // The camera won't let you set specific values when related auto functions are active.
        // So we need to disable the related auto functions before setting the values.
        cout << "Turning off Gain Auto and Exposure Auto." << endl;
#if defined( USE_USB ) || defined( USE_BCON )
        camera.GainAuto.SetValue(GainAuto_Off);
        camera.Gain.SetValue(camera.Gain.GetMin());
        camera.ExposureAuto.SetValue(ExposureAuto_Off);
        camera.ExposureTime.SetValue(camera.ExposureTime.GetMin());
#else
        camera.GainAuto.SetValue(GainAuto_Off);
        camera.GainRaw.SetValue(camera.GainRaw.GetMin());
        camera.ExposureAuto.SetValue(ExposureAuto_Off);
        camera.ExposureTimeRaw.SetValue(camera.ExposureTimeRaw.GetMin());
#endif
        // Save to user set 1.
        //
        // ATTENTION:
        // This will overwrite all settings previously saved in user set 1.
        cout << "Saving currently active settings to user set 1." << endl;
        camera.UserSetSelector.SetValue(UserSetSelector_UserSet1);
        camera.UserSetSave.Execute();

        // Show default settings.
        cout << endl << "Loading default settings." << endl;
        camera.UserSetSelector.SetValue(UserSetSelector_Default);
        camera.UserSetLoad.Execute();
        cout << "Default settings" << endl;
        cout << "================" << endl;
#if defined( USE_USB ) || defined( USE_BCON )
        cout << "Gain          : " << camera.Gain.GetValue() << endl;
        cout << "Exposure time : " << camera.ExposureTime.GetValue() << endl;
#else
        cout << "Gain          : " << camera.GainRaw.GetValue() << endl;
        cout << "Exposure time : " << camera.ExposureTimeRaw.GetValue() << endl;
#endif

        // Show user set 1 settings.
        cout << endl << "Loading user set 1 settings." << endl;
        camera.UserSetSelector.SetValue(UserSetSelector_UserSet1);
        camera.UserSetLoad.Execute();
        cout << "User set 1 settings" << endl;
        cout << "===================" << endl;
#if defined( USE_USB ) || defined( USE_BCON )
        cout << "Gain          : " << camera.Gain.GetValue() << endl;
        cout << "Exposure time : " << camera.ExposureTime.GetValue() << endl;
#else
        cout << "Gain          : " << camera.GainRaw.GetValue() << endl;
        cout << "Exposure time : " << camera.ExposureTimeRaw.GetValue() << endl;
#endif

#if defined( USE_USB ) || defined( USE_BCON )
        // Set user set 1 as default user set:
        // When the camera wakes up it will be configured
        // with the settings from user set 1.
        camera.UserSetDefault.SetValue(UserSetDefault_UserSet1);

        // Restore the default user set selector.
        camera.UserSetDefault.SetValue(oldDefaultUserSet);
#else
        // Set user set 1 as default user set:
        // When the camera wakes up it will be configured
        // with the settings from user set 1.
        camera.UserSetDefaultSelector.SetValue(UserSetDefaultSelector_UserSet1);

        // Restore the default user set selector.
        camera.UserSetDefaultSelector.SetValue(oldDefaultUserSet);
#endif

        // Close the camera.
        camera.Close();
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while( cin.get() != '\n');

    // Releases all pylon resources. 
    PylonTerminate(); 

    return exitCode;
}

