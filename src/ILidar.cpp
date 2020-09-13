#include "ILidar.h"
#include "rplidar.h"
#include "a1.h"

using  RPlidarDriver = rp::standalone::rplidar::RPlidarDriver;

#ifdef HIGH_SPEED
#define BAUDRATE 256000
#else
#define BAUDRATE 115200
#endif

#define SUCCESS_OR(expr, err, block) \
  if (!IS_OK(expr)) {                 \
		ofLog(OF_LOG_ERROR) << err;      \
		block                            \
	}

namespace {
	string formatSerialNumber(uint8_t serialnum[16]) {
	  string ret;
		ret.reserve(16);
    for (int pos = 0; pos < 16; ++pos) {
        ret += ofToHex(serialnum[pos]);
    }
    return ret;
	}
}

namespace ofx {
namespace rplidar {

void ILidar::drawDebug() {
	if (!measurements.updated) { return; }
	ofPushMatrix();
	ofTranslate(ofGetWidth() / 2.0f, ofGetHeight() / 2.0f);
	ofSetColor(ofColor::white);
	ofFill();
	ofDrawCircle(0, 0, 10, 10);
	ofNoFill();
	ofPath path;
	for(size_t i = 0; i < measurements.count; i++) {
		float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
		float radius = ofMap(nodes[i].dist_mm_q2, 0, 60000, 0, 1000);
		// ofLog() << radius;
		float x = radius * cos(ofDegToRad(angle));
		float y = radius * sin(ofDegToRad(angle));
		if (i == 0) {
			path.moveTo(x, y);
		} else if ((x != 0 || y != 0) && radius < 1000) {
			path.lineTo(x, y);
		}
	}
	path.close();
	path.simplify();
	path.draw();
	ofPopMatrix();
}

std::unique_ptr<ILidar> ILidar::create(const string& serial_port) {
	RPlidarDriver *driver = RPlidarDriver::CreateDriver();
	if (driver == nullptr) {
		ofLog(OF_LOG_ERROR) << "Unable to create lidar driver";
		return nullptr;
	}

	SUCCESS_OR(driver->connect(serial_port.c_str(), BAUDRATE), "Unable to open " << serial_port, {
		RPlidarDriver::DisposeDriver(driver);
		return nullptr;
	})

	rplidar_response_device_info_t device_info;
	SUCCESS_OR(driver->getDeviceInfo(device_info), "Unable to get device information", {
		RPlidarDriver::DisposeDriver(driver);
		return nullptr;
	})

	ofLog() << (int)device_info.model << ":" << device_info.firmware_version
			    << ":" << device_info.hardware_version << ":" << ::formatSerialNumber(device_info.serialnum);

	switch(device_info.model) {
		case 24:
			return std::make_unique<ofx::rplidar::impl::A1>(driver);
			break;

		default:
			ofLog(OF_LOG_WARNING) << "Unknown model, falling back to A1";
			return std::make_unique<ofx::rplidar::impl::A1>(driver);
			break;
	}
}

} // namespace rplidar
} // namespace ofx