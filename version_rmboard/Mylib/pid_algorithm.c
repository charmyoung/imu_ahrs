#include "main.h"

#define GAP 0.0
/********************************************************************************
                         pitch轴电调板的速度环控制
                    输入 pitch轴当前速度 pitch轴目标速度
*********************************************************************************/
float Velocity_Control_206(float current_velocity_206,float target_velocity_206)
{
    const float v_p = PITCH_206_VEL_P;
    const float v_d = PITCH_206_VEL_D;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    
    if(abs(current_velocity_206) < GAP)
    {
        current_velocity_206 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_206 - current_velocity_206;
    
    output = error_v[1] * v_p             
             + (error_v[1] - error_v[0]) * v_d;
     
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return -output;//cyq:for6015 反向
}


/********************************************************************************
                         pitch轴电调板的位置环控制 
                    输入 pitch轴当前位置 pitch轴目标位置
*********************************************************************************/
float Position_Control_206(float current_position_206,float target_position_206)
{
    //2车 p 3.0 d 1.0
    const float l_p = PITCH_206_POS_P;
    const float l_i = PITCH_206_POS_I;
    const float l_d = PITCH_206_POS_D;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_position_206 - current_position_206;
    inte += error_l[1]; 
    
    output = error_l[1] * l_p 
            + inte * l_i 
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    		
    return -output;
}
/********************************************************************************
                           yaw轴电调板的速度环控制
                      输入 yaw轴当前速度 yaw轴目标速度
*********************************************************************************/
float Velocity_Control_205(float current_velocity_205,float target_velocity_205)
{
    const float v_p = YAW_205_VEL_P;
    const float v_d = YAW_205_VEL_D;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
		
    if(abs(current_velocity_205) < GAP)
    {
        current_velocity_205 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_205 - current_velocity_205;
    
    output = error_v[1] * v_p
             + (error_v[1] - error_v[0]) * v_d;
     
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return -output;//cyq:for6015 反向
}

/********************************************************************************
                           yaw轴电调板的位置环控制
                      输入 yaw轴当前位置 yaw轴目标位置
*********************************************************************************/
float Position_Control_205(float current_position_205,float target_position_205)
{
    const float l_p = YAW_205_POS_P;//0.750
		const float l_i = YAW_205_POS_I;//0.000035;
    const float l_d = YAW_205_POS_D;//0.2;
    
    static float error_l[3] = {0.0,0.0,0.0};
    static float output = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = error_l[2];    
    error_l[2] = target_position_205 - current_position_205;
 
    output = error_l[2] * l_p 
							+ error_l[2] * l_i 
							+ (error_l[2] - error_l[1]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return -output;
}
/********************************************************************************
                           820R轴电调板的速度环控制
                      输入 820R轴当前速度 820R轴目标速度
*********************************************************************************/
float Velocity_Control_820R(float current_velocity_820R,float target_velocity_820R)
{
    const float l_p = ESC_820R_VEL_P;//7.0
    const float l_i = ESC_820R_VEL_I;//0.5
    const float l_d = ESC_820R_VEL_D;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_velocity_820R - current_velocity_820R;
    inte += error_l[1]; 
    
    output = error_l[1] * l_p 
            + inte * l_i 
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    		
    return output;
}
/********************************************************************************
                           820R电调板的位置环控制
                      输入 820R轴当前位置 820R轴目标位置
*********************************************************************************/
float Position_Control_820R(float current_position_820R,float target_position_820R)
{
    const float l_p = ESC_820R_POS_P;
    const float l_i = ESC_820R_POS_I;
    const float l_d = ESC_820R_POS_D;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_position_820R - current_position_820R;
    inte += error_l[1]; 
    
    output = error_l[1] * l_p 
            + inte * l_i 
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    		
    return output;
}
