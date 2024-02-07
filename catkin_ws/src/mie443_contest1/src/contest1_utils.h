#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180./M_PI)
#define DEG2RAD(deg) ((deg)*M_PI /180.)

extern uint8_t bumper[3];
extern float minLaserDist;
extern int32_t nLasers, desiredNLasers, desiredAngle;
double posX, posY, yaw;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

void odomCallback (const nav_msgs::Odometry::ConstPtr& msg);
