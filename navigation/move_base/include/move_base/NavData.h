#ifndef _NAVDATA_STRUCT_
#define _NAVDATA_STRUCT_

// Type define
typedef float   float32_t;
#ifdef WIN32
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int8 uint8_t;
#else
typedef int int32_t;
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
#endif

#include <string.h>
/**************CONSTANT DEFINITION************/

// Define constants for accelerometers handling
typedef enum {
	ACC_X   = 0,
	ACC_Y   = 1,
	ACC_Z   = 2,
	NB_ACCS = 3
} def_acc_t;

// Define constants for gyrometers handling
typedef enum {
	GYRO_X    = 0,
	GYRO_Y    = 1,
	GYRO_Z    = 2,
	NB_GYROS  = 3
} def_gyro_t;

// tag index for each option
typedef enum _navdata_tag_t {
	   NAVDATA_DEMO_TAG = 0,
	   NAVDATA_TIME_TAG,
	   NAVDATA_RAW_MEASURES_TAG,
	   NAVDATA_PHYS_MEASURES_TAG,
	   NAVDATA_GYROS_OFFSETS_TAG,
	   NAVDATA_EULER_ANGLES_TAG,
	   NAVDATA_REFERENCES_TAG,
	   NAVDATA_TRIMS_TAG,
	   NAVDATA_RC_REFERENCES_TAG,
	   NAVDATA_PWM_TAG,
	   NAVDATA_ALTITUDE_TAG,
	   NAVDATA_VISION_RAW_TAG,
	   NAVDATA_VISION_OF_TAG,
	   NAVDATA_VISION_TAG,
	   NAVDATA_VISION_PERF_TAG,
	   NAVDATA_TRACKERS_SEND_TAG,
	   NAVDATA_VISION_DETECT_TAG,
	   NAVDATA_WATCHDOG_TAG,
	   NAVDATA_ADC_DATA_FRAME_TAG,
	   NAVDATA_VIDEO_STREAM_TAG,
	   NAVDATA_CKS_TAG = 0xFFFF
} navdata_tag_t;
/**********************************************/

// navdata header
typedef struct navdata_header_t {
	uint32_t    header;                   // header:55667788 
	uint32_t    state;                    // the state of the drone 
	uint32_t    seq;                      // sequence number 
	uint32_t    vision;                   // vision flag 
} navdata_header_t;

//	template for each option
typedef struct _navdata_option_t {
	uint16_t  tag;
	uint16_t  size;
#if defined _MSC_VER || defined (__ARMCC_VERSION)
	/* Do not use flexible arrays (C99 feature) with these compilers */
	uint8_t   data[1];
#else
	uint8_t   data[];
#endif
} navdata_option_t;

// navdata option: nav_demo mode
typedef struct navdata_demo_t {
	uint16_t    tag;                        // Navdata block ('option') identifier 
	uint16_t    size;                      // set this to the size of this structure 

	uint32_t    ctrl_state;               // Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. 
	uint32_t    vbat_flying_percentage;   // battery voltage filtered (mV) 

	float32_t   theta;                    // pitch angle in milli-degrees 
	float32_t   phi;                      // roll  angle
	float32_t   psi;                      // yaw   angle

	int32_t     altitude;                 // altitude in centimeters[??] / should be milimeter?

	float32_t   vx;                       // estimated linear velocity
	float32_t   vy;                       // estimated linear velocity
	float32_t   vz;                       // estimated linear velocity

	uint32_t    num_frames;               //!< streamed frame index  // Not used -> To integrate in video stage.
} navdata_demo_t;

typedef struct nav_timestamp {
	unsigned int microsecond: 21;
	unsigned int second: 11;
}nav_timestamp_t;

typedef struct navdata_time_t {
	uint16_t  tag;
	uint16_t  size; 
	//nav_timestamp tm_stamp;
	uint32_t tm_stamp;
} navdata_time_t;

typedef struct navdata_raw_measures_t {
	uint16_t  tag;
	uint16_t  size;

	// +12 bytes
	uint16_t  raw_accs[NB_ACCS];    // filtered accelerometers
	uint16_t  raw_gyros[NB_GYROS];  // filtered gyrometers
	uint16_t  raw_gyros_110[2];     // gyrometers  x/y 110 deg/s
	uint32_t  vbat_raw;             // battery voltage raw (mV)
	uint16_t  us_debut_echo;
	uint16_t  us_fin_echo;
	uint16_t  us_association_echo;
	uint16_t  us_distance_echo;
	uint16_t  us_courbe_temps;
	uint16_t  us_courbe_valeur;
	uint16_t  us_courbe_ref;
} navdata_raw_measures_t;

typedef struct navdata_phys_measures_t {
	uint16_t   tag;
	uint16_t   size;

	float32_t   accs_temp;
	uint16_t    gyro_temp;
	float32_t   phys_accs[NB_ACCS];
	float32_t   phys_gyros[NB_GYROS];
	uint32_t    alim3V3;              // 3.3volt alim [LSB]
	uint32_t    vrefEpson;            // ref volt Epson gyro [LSB]
	uint32_t    vrefIDG;              // ref volt IDG gyro [LSB]
}navdata_phys_measures_t;

class NavData{

public:
	NavData(){};
	~NavData(){currPos = NULL;}
	char rawData[2048];
	navdata_header_t nav_header;
	navdata_demo_t nav_demo;
	navdata_time_t nav_time;
	navdata_raw_measures_t nav_raw_measures;
	navdata_phys_measures_t nav_phys_measures;

private:
	char* currPos;
	
	void setHeader(){
		memcpy(&nav_header, rawData, 16);
		currPos = rawData + sizeof(nav_header); // read until nav_demo
	}

	void setDemoInfo(){
		memcpy(&(nav_demo.tag), currPos, sizeof(nav_demo.tag));
		if ( nav_demo.tag == NAVDATA_DEMO_TAG ){
		memcpy(&(nav_demo.size), currPos + 2, sizeof(nav_demo.size));
		memcpy(&(nav_demo.ctrl_state), currPos + 4, sizeof(nav_demo.ctrl_state));
		memcpy(&(nav_demo.vbat_flying_percentage), currPos + 8, sizeof(nav_demo.vbat_flying_percentage));
		memcpy(&(nav_demo.theta), currPos + 12, sizeof(nav_demo.theta));
		memcpy(&(nav_demo.phi), currPos + 16, sizeof(nav_demo.phi));
		memcpy(&(nav_demo.psi), currPos + 20, sizeof(nav_demo.psi));
		memcpy(&(nav_demo.altitude), currPos + 24, sizeof(nav_demo.altitude));
		memcpy(&(nav_demo.vx), currPos + 28, sizeof(nav_demo.vx));
		memcpy(&(nav_demo.vy), currPos + 32, sizeof(nav_demo.vy));
		memcpy(&(nav_demo.vz), currPos + 36, sizeof(nav_demo.vz));
		currPos = currPos + nav_demo.size;
		}
		else
			return;
	}

	void setTimeInfo(){
		memcpy(&(nav_time.tag), currPos, sizeof(nav_time.tag));
		if (nav_time.tag == NAVDATA_TIME_TAG){
		memcpy(&(nav_time.size), currPos + 2, sizeof(nav_time.size));
		memcpy(&(nav_time.tm_stamp), currPos + 4, sizeof(nav_time.tm_stamp));
		currPos = currPos + nav_time.size;
		}
		else
			return;
	}

	void setRawMeasurement(){
		memcpy(&(nav_raw_measures.tag), currPos, sizeof(nav_raw_measures.tag));
		if (nav_raw_measures.tag == NAVDATA_RAW_MEASURES_TAG){
			memcpy(&(nav_raw_measures.size), currPos + 2, sizeof(nav_raw_measures.size));
			memcpy(&(nav_raw_measures.raw_accs[0]), currPos + 4 , sizeof(nav_raw_measures.raw_accs) * NB_ACCS);
			memcpy(&(nav_raw_measures.raw_gyros[0]), currPos + 10 , sizeof(nav_raw_measures.raw_gyros) * NB_GYROS);
			memcpy(&(nav_raw_measures.raw_gyros_110[0]), currPos + 16 , sizeof(nav_raw_measures.raw_gyros_110) * 2);
			memcpy(&(nav_raw_measures.vbat_raw), currPos + 20, sizeof(nav_raw_measures.vbat_raw));
			currPos = currPos + nav_raw_measures.size;
		}
		else
			return;
	}

	void setPhysMeasurement(){
		memcpy(&(nav_phys_measures.tag), currPos, sizeof(nav_phys_measures.tag));
		if (nav_phys_measures.tag == NAVDATA_PHYS_MEASURES_TAG){
			memcpy(&(nav_phys_measures.size), currPos + 2, sizeof(nav_phys_measures.size));
			memcpy(&(nav_phys_measures.accs_temp), currPos + 4, sizeof(nav_phys_measures.accs_temp));
			memcpy(&(nav_phys_measures.gyro_temp), currPos + 8 , sizeof(nav_phys_measures.gyro_temp));
			memcpy(&(nav_phys_measures.phys_accs[0]), currPos + 10 , sizeof(nav_phys_measures.phys_accs[0]) * NB_ACCS);
			memcpy(&(nav_phys_measures.phys_gyros[0]), currPos + 22 , sizeof(nav_phys_measures.phys_gyros[0]) * NB_GYROS);
			currPos = currPos + nav_phys_measures.size;
		}
		else
			return;
	}
public:

	void setOptions(){
		setHeader();
		setDemoInfo();
		setTimeInfo();
		setRawMeasurement();
		setPhysMeasurement();
	}
};

class MyNavData{
public:
	int _f;	//frame number
	float _vx, _vy, _altitude;
	float _roll, _pitch, _yaw;
	MyNavData(int f, float vx,float vy, float altitude, float roll, float pitch, float yaw)
		:_f(f), _vx(vx), _vy(vy), _altitude(altitude),_roll(roll), _pitch(pitch), _yaw(yaw){}
};
#endif