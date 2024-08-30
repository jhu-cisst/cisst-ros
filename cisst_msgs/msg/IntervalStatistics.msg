std_msgs/Header header

# average period
float64 period_avg
# deviation on periodicity
float64 period_std_dev
# min and max time between
float64 period_min
float64 period_max
# same 4 values for compute time
float64 compute_time_avg
float64 compute_time_std_dev
float64 compute_time_min
float64 compute_time_max
# number of samples used to compute these status
int64 number_of_samples
# number of overruns base on previous interval average
int64 number_of_overruns
# time window covered by these stats
float64 statistics_interval

# user data
int64 user_data
