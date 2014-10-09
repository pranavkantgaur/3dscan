#include"../PROJECT_GLOBAL/global_cv.h"

//Pattern generate.
void generate_pattern();
//Pattern project.

void project_and_capture_pattern(int pattern_type);

//Compute wrapped phase.
void compute_wrapped_phase(int pattern_type);

//Compute unwrapped phase.
int unwrap_phase(int pattern_type);

//Compute correspondance.
void compute_c_p_map();

//Calibrate system.
void system_calibration();

//Triangulate.
void triangulate();

//Point cloud save & visualize.
void save_point_cloud(unsigned);
void visualize_point_cloud();

// registration function
void register_point_clouds(unsigned,float,float,float, float);


//To measure errors in camera & projector calibration.
void measure_calibration_errors();


// V4L2-access functions
int print_caps();
int init_mmap();
void preview_and_capture();
void stop_capturing();
void uninit_device();
void close_device();
