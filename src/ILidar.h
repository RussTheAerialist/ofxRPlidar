#pragma once

#include "ofMain.h"
#include "rplidar.h"

#define MEASUREMENT_COUNT 8192

namespace ofx {
namespace rplidar {

    struct Measurement {
        rplidar_response_measurement_node_hq_t *data;
        size_t count;
        bool updated;
    };

    //! Currently only A1 has been implemented.
    //!
    //! If you would like to see other devices implemented, please consider contributing a device
    enum class DeviceType {
        A1,
        A2,
        A3
    };

    //! ILidar defines a generic interface for SLAMTEC style lidar devices.
    //!
    //! Since the A1, A2, A3 series are different from each other, this class defines a common interface, and
    //! the "opne" constructor will return the appropriate one based on which device is found and detected.
    class ILidar {
    public:
        ILidar() { measurements.data = nodes; measurements.count = MEASUREMENT_COUNT; measurements.updated = false; }
        virtual ~ILidar() { } // Destructor must be virtual so that subclasses can implement.

        static std::unique_ptr<ILidar> create(const string& serial_port);

        //! Starts a background thread and begins collecting scans
        //!
        //! *Note: This will start the motor running*
        //!
        //! When the scan is running, onNewFrameAvailable will trigger
        //! so you can add a listener to retrieve the measurements and do
        //! what is needed.
        virtual bool startScan() = 0;

        //! Kills the background thread to stop collecting scans
        //!
        //! *Note: This will stop the motor running*
        //!
        //! The current measurement will stay static until you start scanning again.
        virtual bool stopScan() = 0;

        //! This draws a debugging view of the measurements from the lidar.
        //!
        //! It draws a polyline connecting all of the measurements in a 360.
        void drawDebug();

        virtual bool scanning() = 0;

        //! Fired when a new scan is available.
        //!
        //! Under the hoods, this uses ascendScanData with `rplidar_response_measurement_node_hq_t` datatypes
        ofEvent<const Measurement&> onNewFrameAvailable;

    protected:
        Measurement measurements;
        rplidar_response_measurement_node_hq_t nodes[MEASUREMENT_COUNT];
    };

} // namespace rplidar
} // namespace ofx
