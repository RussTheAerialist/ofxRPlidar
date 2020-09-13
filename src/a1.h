#pragma once

#include "ofMain.h"
#include "ILidar.h"
#include "rplidar.h"

namespace ofx {
	namespace rplidar {
		namespace impl {
			class A1 : public ofx::rplidar::ILidar, public ofThread {
				public:
					A1(rp::standalone::rplidar::RPlidarDriver *driver);
					virtual ~A1();
					virtual bool startScan();
					virtual bool stopScan();
					virtual bool scanning();
					void threadedFunction();
				private:
					rp::standalone::rplidar::RPlidarDriver *driver;
					bool is_scanning;

			};
		} // namespace impl
	} // namespace rplidar
} // namespace ofx