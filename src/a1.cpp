#include "a1.h"
#include "rplidar.h"

namespace ofx { namespace rplidar { namespace impl {

A1::A1(rp::standalone::rplidar::RPlidarDriver *driver) : driver(driver), is_scanning(false) {
	driver->stop();
	driver->stopMotor();
}

A1::~A1() {
	stopScan();
	rp::standalone::rplidar::RPlidarDriver::DisposeDriver(driver);
}

bool A1::startScan() {
	startThread(true);
	return false;
}

bool A1::stopScan() {
	is_scanning = false;
	waitForThread(true);

	driver->stop();
	driver->stopMotor();
	return false;
}

bool A1::scanning() {
	return is_scanning;
}

void A1::threadedFunction() {
	ofLog(OF_LOG_VERBOSE) << "Starting Background scanning";
	is_scanning = true;
	driver->startMotor();
	driver->startScan(0, 1);

	while(isThreadRunning()) {
		auto result = driver->grabScanDataHq(nodes, measurements.count);
		measurements.updated = true;
		if (IS_OK(result)) {
			driver->ascendScanData(nodes, measurements.count);
			ofNotifyEvent(onNewFrameAvailable, measurements);
		}
	}
}

}}}