function n = function_delay(sampling_time,neural_time_delay,k)
%FUNCTION_DELAY Summary of this function goes here
%   Detailed explanation goes here 

neural_time_delay = neural_time_delay/1000;
if sampling_time*k<=neural_time_delay
    n = 1;
else
    Delay_discrete_time = round(neural_time_delay/sampling_time);  
    n =k-Delay_discrete_time;
end

