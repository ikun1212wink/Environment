/**
 *  @file       rc.c
 *  @author     @RobotPilots
 *  @version    v1.0
 *  @brief      Rc Module.
 *  @update     
 *              v1.0(8-Feburary-2022)
 */
 
/* Includes ------------------------------------------------------------------*/
#include "rc.h"

#include "drv_haltick.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static button_flip_t rc_update_button(button_t *btn, button_state_t new_btn_state);
static switch_flip_t rc_update_switch(switch_t *sw, switch_state_t new_sw_state);
static void rc_update_stick(stick_t *stick, int16_t val);
static void rc_update_mouse(mouse_speed_t *mouse_speed, int16_t val);
static button_flip_t rc_copy_button(button_t *dst, button_t *src);
static switch_flip_t rc_copy_switch(switch_t *dst, switch_t *src);
static void rc_reset_button(button_t *btn, button_state_t new_btn_state);
static void rc_reset_switch(switch_t *sw, switch_state_t new_sw_state);
static void rc_reset_stick(stick_t *stick, int16_t val);
static void rc_reset_mouse(mouse_speed_t *mouse_speed, int16_t val);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
// rc_info
rc_info_t rc_info;
// rc
rc_t rc = {
    .dev = &rc_sensor,
    .info = &rc_info,
    .init = rc_init,
    .update = rc_update,
    .reset = rc_reset,
    .update_button = rc_update_button,
    .update_switch = rc_update_switch,
    .update_stick = rc_update_stick,
    .update_mouse = rc_update_mouse,
    .copy_button = rc_copy_button,
    .copy_switch = rc_copy_switch,
    .reset_button = rc_reset_button,
    .reset_switch = rc_reset_switch,
    .reset_stick = rc_reset_stick,
    .reset_mouse = rc_reset_mouse
};

/* Private functions ---------------------------------------------------------*/
static button_flip_t rc_update_button(button_t *btn, button_state_t new_btn_state)
{
    button_flip_t trig = BTN_NONE_FLIP;
    
    /* 按键状态跳变 */
    if(new_btn_state != btn->state) {
        btn->hold_time = 0;
        /* RELEASE -> PRESS */
        if(new_btn_state == PRESS) {
            trig = RELEASE_TO_PRESS;
        }
        /* PRESS -> RELEASE */
        else if(new_btn_state == RELEASE) {
            trig = PRESS_TO_RELEASE;
        }
    } 
    /* 按键状态保持 */
    else {
        btn->hold_time += millis() - btn->update_time;
    }
    
    btn->update_time = millis();
    btn->state = new_btn_state;
    btn->flip = trig;
    
    return trig;
}

static switch_flip_t  rc_update_switch(switch_t *sw, switch_state_t new_sw_state)
{
    switch_flip_t trig = SW_NONE_FLIP;
    
    /* 拨杆状态跳变 */
    if(new_sw_state != sw->state) {
        // 保持时间清零
        sw->hold_time = 0;
        if(sw->state == RC_SW_UP) {
            /* MID -> UP */
            if(new_sw_state == RC_SW_MID) {
                trig = SW_MID_TO_UP;
            }
        }
        else if(sw->state == RC_SW_MID) {
            /* UP -> MID */
            if(new_sw_state == RC_SW_UP) {
                trig = SW_UP_TO_MID;
            }
            /* DOWN -> MID */
            else if(new_sw_state == RC_SW_DOWN) {
                trig = SW_DOWN_TO_MID;
            }
        }
        else if(sw->state == RC_SW_DOWN) {
            /* MID -> DOWN */
            if(new_sw_state == RC_SW_MID) {
                trig = SW_MID_TO_DOWN;
            }
        }
    }
    /* 拨杆状态保持 */
    else {
        sw->hold_time += millis() - sw->update_time;
    }
    
    sw->update_time = millis();
    sw->state = new_sw_state;
    sw->flip = trig;
    
    return trig;
}

static void rc_update_stick(stick_t *stick, int16_t val)
{
    stick->val = val;
}

static void rc_update_mouse(mouse_speed_t *mouse_speed, int16_t val)
{
    mouse_speed->val = val;
}

static button_flip_t rc_copy_button(button_t *dst, button_t *src)
{
    button_flip_t trig = BTN_NONE_FLIP;
    
    /* 按键状态跳变 */
    if(dst->state != src->state) {
        /* RELEASE -> PRESS */
        if(src->state == PRESS) {
            trig = RELEASE_TO_PRESS;
        } 
        /* PRESS -> RELEASE */
        else if(src->state == RELEASE) {
            trig = PRESS_TO_RELEASE;
        }
    }
    dst->state = src->state;
    dst->hold_time = src->hold_time;
    
    return trig;    
}

static switch_flip_t rc_copy_switch(switch_t *dst, switch_t *src)
{
    switch_flip_t trig = SW_NONE_FLIP;
    
    /* 拨杆状态跳变 */
    if(dst->state != src->state) {
        if(src->state == RC_SW_UP) {
            /* MID -> UP */
            if(dst->state == RC_SW_MID) {
                trig = SW_MID_TO_UP;
            }
        }
        else if(src->state == RC_SW_MID) {
            /* UP -> MID */
            if(dst->state == RC_SW_UP) {
                trig = SW_UP_TO_MID;
            }
            /* DOWN -> MID */
            else if(dst->state == RC_SW_DOWN) {
                trig = SW_DOWN_TO_MID;
            }
        }
        else if(src->state == RC_SW_DOWN) {
            /* MID -> DOWN */
            if(dst->state == RC_SW_MID) {
                trig = SW_MID_TO_DOWN;
            }
        }
    }
    dst->state = src->state;
    dst->hold_time = src->hold_time;
    
    return trig;    
}

static void rc_reset_button(button_t *btn, button_state_t rst_btn_state)
{
    btn->state = rst_btn_state;
    btn->flip = BTN_NONE_FLIP;
    btn->hold_time = 0;
    btn->update_time = millis();
}

static void rc_reset_switch(switch_t *sw, switch_state_t rst_sw_state)
{
    sw->state = rst_sw_state;
    sw->flip = SW_NONE_FLIP;
    sw->hold_time = 0;
    sw->update_time = millis();
}

static void rc_reset_stick(stick_t *stick, int16_t val)
{
    stick->val = val;
}

static void rc_reset_mouse(mouse_speed_t *mouse_speed, int16_t val)
{
    mouse_speed->val = val;
}

/* Exported functions --------------------------------------------------------*/
void rc_init(void)
{
    rc.reset();
}

void rc_update(void)
{
    rc_sensor_info_t *rc_sen = rc.dev->info;
    
    // update rc buttons
    rc.update_button(&rc_info.W, (rc_sen->key_v & KEY_PRESSED_OFFSET_W) ? PRESS : RELEASE);
    rc.update_button(&rc_info.S, (rc_sen->key_v & KEY_PRESSED_OFFSET_S) ? PRESS : RELEASE);
    rc.update_button(&rc_info.A, (rc_sen->key_v & KEY_PRESSED_OFFSET_A) ? PRESS : RELEASE);
    rc.update_button(&rc_info.D, (rc_sen->key_v & KEY_PRESSED_OFFSET_D) ? PRESS : RELEASE);
    rc.update_button(&rc_info.SHIFT, (rc_sen->key_v & KEY_PRESSED_OFFSET_SHIFT) ? PRESS : RELEASE);
    rc.update_button(&rc_info.CTRL, (rc_sen->key_v & KEY_PRESSED_OFFSET_CTRL) ? PRESS : RELEASE);
    rc.update_button(&rc_info.Q, (rc_sen->key_v & KEY_PRESSED_OFFSET_Q) ? PRESS : RELEASE);
    rc.update_button(&rc_info.E, (rc_sen->key_v & KEY_PRESSED_OFFSET_E) ? PRESS : RELEASE);
    rc.update_button(&rc_info.R, (rc_sen->key_v & KEY_PRESSED_OFFSET_R) ? PRESS : RELEASE);
    rc.update_button(&rc_info.F, (rc_sen->key_v & KEY_PRESSED_OFFSET_F) ? PRESS : RELEASE);
    rc.update_button(&rc_info.G, (rc_sen->key_v & KEY_PRESSED_OFFSET_G) ? PRESS : RELEASE);
    rc.update_button(&rc_info.Z, (rc_sen->key_v & KEY_PRESSED_OFFSET_Z) ? PRESS : RELEASE);
    rc.update_button(&rc_info.X, (rc_sen->key_v & KEY_PRESSED_OFFSET_X) ? PRESS : RELEASE);
    rc.update_button(&rc_info.C, (rc_sen->key_v & KEY_PRESSED_OFFSET_C) ? PRESS : RELEASE);
    rc.update_button(&rc_info.V, (rc_sen->key_v & KEY_PRESSED_OFFSET_V) ? PRESS : RELEASE);
    rc.update_button(&rc_info.B, (rc_sen->key_v & KEY_PRESSED_OFFSET_B) ? PRESS : RELEASE);
    rc.update_button(&rc_info.MOUSE_L, (rc_sen->mouse_btn_l) ? PRESS : RELEASE);
    rc.update_button(&rc_info.MOUSE_R, (rc_sen->mouse_btn_r) ? PRESS : RELEASE);   
    // update rc switches
    rc.update_switch(&rc_info.SW1, (switch_state_t)rc_sen->s1);
    rc.update_switch(&rc_info.SW2, (switch_state_t)rc_sen->s2);
    // update rc sticks
    rc.update_stick(&rc_info.CH0, rc_sen->ch0);
    rc.update_stick(&rc_info.CH1, rc_sen->ch1);
    rc.update_stick(&rc_info.CH2, rc_sen->ch2);
    rc.update_stick(&rc_info.CH3, rc_sen->ch3);
    // update rc mouses
    rc.update_mouse(&rc_info.MOUSE_VX, rc_sen->mouse_vx);
    rc.update_mouse(&rc_info.MOUSE_VY, rc_sen->mouse_vy);
    rc.update_mouse(&rc_info.MOUSE_VZ, rc_sen->mouse_vz);    
}

void rc_reset(void)
{
    // reset rc buttons
    rc.reset_button(&rc_info.W, RELEASE);
    rc.reset_button(&rc_info.S, RELEASE);
    rc.reset_button(&rc_info.A, RELEASE);
    rc.reset_button(&rc_info.D, RELEASE);
    rc.reset_button(&rc_info.SHIFT, RELEASE);
    rc.reset_button(&rc_info.CTRL, RELEASE);
    rc.reset_button(&rc_info.Q, RELEASE);
    rc.reset_button(&rc_info.E, RELEASE);
    rc.reset_button(&rc_info.R, RELEASE);
    rc.reset_button(&rc_info.F, RELEASE);
    rc.reset_button(&rc_info.G, RELEASE);
    rc.reset_button(&rc_info.Z, RELEASE);
    rc.reset_button(&rc_info.X, RELEASE);
    rc.reset_button(&rc_info.C, RELEASE);
    rc.reset_button(&rc_info.V, RELEASE);
    rc.reset_button(&rc_info.B, RELEASE);
    rc.reset_button(&rc_info.MOUSE_L, RELEASE);
    rc.reset_button(&rc_info.MOUSE_R, RELEASE);   
    // reset rc switches
    rc.reset_switch(&rc_info.SW1, SW_MID);
    rc.reset_switch(&rc_info.SW2, SW_MID);
    // reset rc sticks
    rc.reset_stick(&rc_info.CH0, 0);
    rc.reset_stick(&rc_info.CH1, 0);
    rc.reset_stick(&rc_info.CH2, 0);
    rc.reset_stick(&rc_info.CH3, 0);
    // reset rc mouses
    rc.reset_mouse(&rc_info.MOUSE_VX, 0);
    rc.reset_mouse(&rc_info.MOUSE_VY, 0);
    rc.reset_mouse(&rc_info.MOUSE_VZ, 0); 
}

/* 信息层 --------------------------------------------------------------------*/
/* 应用层 --------------------------------------------------------------------*/
/* 任务层 --------------------------------------------------------------------*/
