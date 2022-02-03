/************************************************************************/
// File:            PID_controller.h                                    //
// Author:          Eivind Jølsgard                                     //
// Date:            2020                                                //
// Purpose:         Controlling the motor speed based on encoder reading//
//                                                                      //
/************************************************************************/

#ifndef PID_CONTROLLER_Hs
#define PID_CONTROLLER_H

typedef enum{
    ERROR,
    MEASUREMENT
} DERIVATIVE_SELECT;

typedef struct PID_parameters
{
    double K_P;
    double K_I;
    double K_D;
    
    double error_integral;
    double integral_boundary;
    
    double max_output; 
    double min_output; 

    double reference_previous;
    double measurement_previous; 

    double error_previous;
    double output_previous;

    DERIVATIVE_SELECT derivative_select;

} PID_parameters_t;

/**
 * @brief Discrete PID controller with integral windup prevention and output limitation
 * @details 
 *
 * @param[in]   pid			PID_parameters configuration and storage of internal values
 * @param[in]   reference 	Desired value of input parameter
 * @param[in]   measurement Actual value of input parameter
 * @param[in]   period 	    Time period (in s) of PID controller
 * @param[out]  output 	    Output of the PID controller
 */
double PID_controller(PID_parameters_t* pid, double reference, double measurement, double period);

/**
 * @brief Discrete PID controller with integral windup prevention and output limitation
 * @details For taking in error when "reference - measurement" is not directly applicable as the actual error.
 *
 * @param[in]   pid			PID_parameters configuration and storage of internal values
 * @param[in]   error 	    Error of input parameter
 * @param[in]   reference 	Desired value of input parameter
 * @param[in]   measurement Actual value of input parameter
 * @param[in]   period 	    Time period (in s) of PID controller
 * @param[out]  output 	    Output of the PID controller
 */
double PID_controller_with_error_as_input(PID_parameters_t* pid, double error, double reference, double measurement, double period);

int PID_steady_state(PID_parameters_t* pid, double error, double margin, int change_in_reference);


#endif //PID_CONTROLLER_H